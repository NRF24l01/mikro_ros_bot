#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
esp_bridge_ws_only.py  •  LiDAR (+ yaw / mirror / static TF)  •  /cmd_vel → ESP (через WebSocket)  •  /odom (ROS 2 Jazzy)

* WebSocket-скан от ESP32  →  /scan   (sensor_msgs/LaserScan)
* WebSocket-состояние      →  /odom  (nav_msgs/Odometry + динамический odom→base_link)
* WebSocket setSpeed       ← /cmd_vel (geometry_msgs/Twist)

* Нет HTTP! Только WebSocket.
"""

import asyncio
import math
import struct
import threading
import socket
import json
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import (
    TransformBroadcaster,
    StaticTransformBroadcaster,
    TransformStamped,
)
import websockets

FRAME_FMT = "<HHHHHHHHHH"          # 20 B – угол начала, угол конца, 8×дистанция
FRAME_SZ = 20
CRC_SZ = 2
MAX_PKTS = 64
MAX_SIZE = MAX_PKTS * FRAME_SZ + CRC_SZ
BASE_MM = 97.0                     # база робота (колёс), мм (из ESP кода: 0.097m)

def crc16(buf: bytes, crc: int = 0xFFFF) -> int:
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc & 0xFFFF

def q_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q

def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

class ESPBridge(Node):
    def __init__(self) -> None:
        super().__init__("esp_bridge")

        # ── параметры подключения ──────────────────────────────────
        self.declare_parameter("host", "192.168.1.42")
        self.declare_parameter("main_port", 80)
        self.declare_parameter("lidar_port", 81)
        self.declare_parameter("ws_path", "/ws")

        # ── имена фреймов и диапазон LiDAR-a ───────────────────────
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 16.0)

        # ── настройка ориентации LiDAR-а ───────────────────────────
        self.declare_parameter("lidar_yaw_deg", 180.0)
        self.declare_parameter("lidar_mirror", True)
        self.declare_parameter("lidar_xyz", [0.0, 0.0, 0.10])
        self.declare_parameter("lidar_rpy_deg", [0.0, 0.0, 0.0])

        host = self.get_parameter("host").value
        main_port = self.get_parameter("main_port").value
        lidar_port = self.get_parameter("lidar_port").value
        path = self.get_parameter("ws_path").value

        self.lidar_ws_url = f"ws://{host}:{lidar_port}{path}"
        self.cmd_ws_url = f"ws://{host}:{main_port}{path}"

        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)

        self.tf_br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        self.lidar_loop = asyncio.new_event_loop()
        threading.Thread(
            target=self._run_lidar_ws_loop, args=(self.lidar_loop,), daemon=True
        ).start()

        self.cmd_ws = None
        self.cmd_ws_lock = threading.Lock()
        self.cmd_ws_loop = asyncio.new_event_loop()
        threading.Thread(
            target=self._run_cmd_ws_loop, args=(self.cmd_ws_loop,), daemon=True
        ).start()

        self.get_logger().info(f"LiDAR WebSocket →  {self.lidar_ws_url}")
        self.get_logger().info(f"CMD WebSocket →  {self.cmd_ws_url}")

    def _publish_static_tf(self) -> None:
        xyz = self.get_parameter("lidar_xyz").value
        rpy_deg = self.get_parameter("lidar_rpy_deg").value

        roll, pitch, yaw = [math.radians(a) for a in rpy_deg]
        q = rpy_to_quat(roll, pitch, yaw)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.get_parameter("base_frame").value
        tf.child_frame_id = self.get_parameter("frame_id").value
        tf.transform.translation.x = float(xyz[0])
        tf.transform.translation.y = float(xyz[1])
        tf.transform.translation.z = float(xyz[2])
        tf.transform.rotation = q
        self.static_br.sendTransform(tf)
        self.get_logger().info("Published static TF base_link → laser")

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation = rpy_to_quat(0.0, 0.0, 0.0)
        self.static_br.sendTransform(tf)
        self.get_logger().info("Published static TF map → odom")

    def _run_lidar_ws_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.lidar_ws_task())

    async def lidar_ws_task(self) -> None:
        while rclpy.ok():
            try:
                async with websockets.connect(
                    self.lidar_ws_url, max_size=None, ping_interval=None
                ) as ws:
                    sock: socket.socket = ws.transport.get_extra_info("socket")
                    if sock:
                        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self.get_logger().info("LiDAR WebSocket connected")

                    while rclpy.ok():
                        buf = await ws.recv()
                        if not isinstance(buf, (bytes, bytearray)):
                            continue
                        if not (FRAME_SZ + CRC_SZ <= len(buf) <= MAX_SIZE):
                            self.get_logger().debug(f"Invalid buffer size: {len(buf)}")
                            continue
                        expected_crc = crc16(buf[:-2])
                        received_crc = int.from_bytes(buf[-2:], "little")
                        if expected_crc != received_crc:
                            self.get_logger().debug(f"CRC mismatch: expected {expected_crc:04X}, got {received_crc:04X}")
                            continue
                        self.publish_scan(buf[:-2])
            except Exception as e:
                self.get_logger().warn(f"LiDAR WebSocket error: {e}")
                await asyncio.sleep(1.0)

    def _run_cmd_ws_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.cmd_ws_task())

    async def cmd_ws_task(self) -> None:
        while rclpy.ok():
            try:
                async with websockets.connect(
                    self.cmd_ws_url, ping_interval=None
                ) as ws:
                    sock: socket.socket = ws.transport.get_extra_info("socket")
                    if sock:
                        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self.get_logger().info("CMD WebSocket connected")
                    with self.cmd_ws_lock:
                        self.cmd_ws = ws
                    while rclpy.ok():
                        try:
                            msg = await asyncio.wait_for(ws.recv(), timeout=1.0)
                            if msg:
                                self.handle_ws_message(msg)
                        except asyncio.TimeoutError:
                            pass
            except Exception as e:
                self.get_logger().warn(f"CMD WebSocket error: {e}")
                with self.cmd_ws_lock:
                    self.cmd_ws = None
                time.sleep(1)

    def handle_ws_message(self, msg):
        try:
            data = json.loads(msg)
            if data.get("type") == "state":
                self.publish_state_from_ws(data)
        except Exception as e:
            self.get_logger().warn(f"WS message parse error: {e}")

    def publish_state_from_ws(self, js):
        try:
            odom_data = js["odom"]
            speed_data = js["speed"]
            x, y, th = odom_data["x"], odom_data["y"], odom_data["th"]
            speed_left = speed_data["left"]
            speed_right = speed_data["right"]

            v_linear = (speed_left + speed_right) / 2000.0
            v_angular = (speed_right - speed_left) / BASE_MM

            od = Odometry()
            od.header.stamp = self.get_clock().now().to_msg()
            od.header.frame_id = self.get_parameter("odom_frame").value
            od.child_frame_id = self.get_parameter("base_frame").value
            od.pose.pose.position.x = float(x)
            od.pose.pose.position.y = float(y)
            od.pose.pose.orientation = q_yaw(float(th))
            od.twist.twist.linear.x = v_linear
            od.twist.twist.angular.z = v_angular
            self.odom_pub.publish(od)

            tf = TransformStamped()
            tf.header.stamp = od.header.stamp
            tf.header.frame_id = od.header.frame_id
            tf.child_frame_id = od.child_frame_id
            tf.transform.translation.x = float(x)
            tf.transform.translation.y = float(y)
            tf.transform.translation.z = 0.0
            tf.transform.rotation = od.pose.pose.orientation
            self.tf_br.sendTransform(tf)
        except Exception as e:
            self.get_logger().warn(f"WS state publish error: {e}")

    def cmd_cb(self, msg: Twist) -> None:
        v_mm = msg.linear.x * 1000.0
        w = msg.angular.z
        left = int(v_mm - w * BASE_MM / 2.0)
        right = int(v_mm + w * BASE_MM / 2.0)
        cmd = {"type": "setSpeed", "left": left, "right": right}
        msg_txt = json.dumps(cmd)
        with self.cmd_ws_lock:
            ws = self.cmd_ws
        if ws is not None:
            coro = ws.send(msg_txt)
            fut = asyncio.run_coroutine_threadsafe(coro, self.cmd_ws_loop)
            try:
                fut.result(timeout=0.2)
            except Exception as e:
                self.get_logger().warn(f"CMD WebSocket send error: {e}")
        else:
            self.get_logger().warn("CMD WebSocket not connected")

    def publish_scan(self, raw: bytes) -> None:
        pkt_cnt = len(raw) // FRAME_SZ
        if pkt_cnt < 10:
            self.get_logger().debug(f"Too few packets: {pkt_cnt}")
            return

        ang, rng, inten = [], [], []
        prev = None
        off = 0.0

        for i in range(pkt_cnt):
            try:
                s, e, *dist = struct.unpack_from(FRAME_FMT, raw, i * FRAME_SZ)
                s /= 100.0
                e /= 100.0
                if e < s:
                    e += 360.0
                for j, d in enumerate(dist):
                    a = s + (e - s) * j / 7
                    if prev is not None and a + off < prev - 300:
                        off += 360.0
                    a += off
                    prev = a
                    ang.append(a)
                    rng.append(d / 1000.0 if d > 0 else float("inf"))
                    inten.append(1.0 if d > 0 else 0.0)
            except struct.error as e:
                self.get_logger().warn(f"Packet {i} unpack error: {e}")
                continue

        if len(ang) < 10:
            self.get_logger().debug("No valid points in scan")
            return

        if self.get_parameter("lidar_mirror").value:
            ang.reverse()
            rng.reverse()
            inten.reverse()
            ang = [-a for a in ang]

        yaw_deg = self.get_parameter("lidar_yaw_deg").value
        if yaw_deg != 0.0:
            ang = [a + yaw_deg for a in ang]

        while ang[0] > 180.0:
            ang = [a - 360.0 for a in ang]
        while ang[0] < -180.0:
            ang = [a + 360.0 for a in ang]

        rad = [math.radians(a) for a in ang]

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.get_parameter("frame_id").value
        scan.angle_min = rad[0]
        scan.angle_max = rad[-1]
        scan.angle_increment = (rad[-1] - rad[0]) / (len(rad) - 1) if len(rad) > 1 else 0.0
        scan.range_min = self.get_parameter("range_min").value
        scan.range_max = self.get_parameter("range_max").value
        scan.ranges = rng
        scan.intensities = inten

        self.scan_pub.publish(scan)
        self.get_logger().debug(f"Published scan with {len(rng)} points")

def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        bridge = ESPBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()