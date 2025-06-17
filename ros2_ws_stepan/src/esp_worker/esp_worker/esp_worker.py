#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
esp_bridge.py  •  LiDAR (+ yaw / mirror / static TF)  •  /cmd_vel → ESP  •  /odom  (ROS 2 Jazzy)

* WebSocket-скан от ESP32  →  /scan   (sensor_msgs/LaserScan)
* REST-команда /setSpeed   ← /cmd_vel (geometry_msgs/Twist)
* HTTP /state → /odom + TF (nav_msgs/Odometry + динамический odom→base_link)

Дополнительные параметры конфигурируют датчик:

  lidar_yaw_deg     (double, ° CW) – повернуть облако
  lidar_mirror      (bool)         – отразить (лево↔право)
  lidar_xyz         (double[3])    – смещение лидара в базе  [м]
  lidar_rpy_deg     (double[3])    – ориентация лидара       [° R P Y]

При старте узел публикует *статический* трансформ base_link→laser,
поэтому RViz/ Nav2 сразу видят полную цепочку `odom → base_link → laser`.
"""

import asyncio
import math
import struct
import threading
import socket
import urllib.parse
import requests
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

# ――― константы LDS пакетов ―――――――――――――――――――――――――――――――――――――――――――――――
FRAME_FMT = "<HHHHHHHHHH"          # 20 B – угол начала, угол конца, 8×дистанция
FRAME_SZ = 20
CRC_SZ = 2
MAX_PKTS = 64                      # макс. пакетов на оборот
MAX_SIZE = MAX_PKTS * FRAME_SZ + CRC_SZ

BASE_MM = 97.0                     # база робота (колёс), мм (из ESP кода: 0.097m)


def crc16(buf: bytes, crc: int = 0xFFFF) -> int:
    """Modbus-совместимый CRC-16"""
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc & 0xFFFF


def q_yaw(yaw: float) -> Quaternion:
    """Quaternion из Yaw (рад)"""
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    """RPY (рад) → Quaternion"""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


# ―――― основной узел ―――――――――――――――――――――――――――――――――――――――――――――――――――
class ESPBridge(Node):
    def __init__(self) -> None:
        super().__init__("esp_bridge")

        # ── параметры подключения ──────────────────────────────────
        self.declare_parameter("host", "192.168.1.42")          # IP ESP32
        self.declare_parameter("main_port", 80)                  # основной HTTP/WS порт
        self.declare_parameter("lidar_port", 81)                 # порт лидара WebSocket
        self.declare_parameter("ws_path", "/ws")                 # путь WebSocket

        # ── имена фреймов и диапазон LiDAR-a ───────────────────────
        self.declare_parameter("frame_id", "sts_laser")
        self.declare_parameter("odom_frame", "sts_odom")
        self.declare_parameter("base_frame", "sts_base_link")
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 16.0)

        # ── настройка ориентации LiDAR-а ───────────────────────────
        self.declare_parameter("lidar_yaw_deg", 180.0)          # сдвиг облака CW
        self.declare_parameter("lidar_mirror", True)            # зеркально (True/False)
        self.declare_parameter("lidar_xyz", [0.0, 0.0, 0.10])   # положение на раме
        self.declare_parameter("lidar_rpy_deg", [0.0, 0.0, 0.0]) # ориентация

        # ── формируем URL-ы ―――――――――――――――――――――――――――――――――――――
        host = self.get_parameter("host").value
        main_port = self.get_parameter("main_port").value
        lidar_port = self.get_parameter("lidar_port").value
        path = self.get_parameter("ws_path").value
        
        # WebSocket для лидара подключается к отдельному порту
        self.lidar_ws_url = f"ws://{host}:{lidar_port}{path}"
        # HTTP для команд и состояния использует основной порт
        self.http_base_url = f"http://{host}:{main_port}"

        # ── ROS паблишеры/сабскрайберы ────────────────────────────
        self.scan_pub = self.create_publisher(LaserScan, "/sts/scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/sts/odom", 10)
        self.create_subscription(Twist, "/sts/cmd_vel", self.cmd_cb, 10)

        # динамический TF (odom→base_link) + статический (base_link→laser)
        self.tf_br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        # ― запускаем отдельный event-loop для WebSocket (лидар) ―
        self.ws_loop = asyncio.new_event_loop()
        threading.Thread(
            target=self._run_ws_loop, args=(self.ws_loop,), daemon=True
        ).start()

        # ― периодический HTTP-опрос /state ―
        self.create_timer(0.10, self.poll_state)

        self.get_logger().info(f"LiDAR WebSocket →  {self.lidar_ws_url}")
        self.get_logger().info(f"HTTP commands →  {self.http_base_url}")

    # =============================================================
    #                 статический TF  base_link → laser
    # =============================================================
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

        # latched – публикуем один раз
        self.static_br.sendTransform(tf)
        self.get_logger().info("Published static TF base_link → laser")
        
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "sts_map"
        tf.child_frame_id = "sts_odom"
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation = rpy_to_quat(0.0, 0.0, 0.0)

        # latched – публикуем один раз
        self.static_br.sendTransform(tf)
        self.get_logger().info("Published static TF map → odom")

    # =============================================================
    #                    WebSocket (LiDAR поток)
    # =============================================================
    def _run_ws_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.ws_task())

    async def ws_task(self) -> None:
        while rclpy.ok():
            try:
                async with websockets.connect(
                    self.lidar_ws_url, max_size=None, ping_interval=None
                ) as ws:
                    # disable Nagle
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
                        
                        # Проверяем CRC16
                        expected_crc = crc16(buf[:-2])
                        received_crc = int.from_bytes(buf[-2:], "little")
                        if expected_crc != received_crc:
                            self.get_logger().debug(f"CRC mismatch: expected {expected_crc:04X}, got {received_crc:04X}")
                            continue
                            
                        self.publish_scan(buf[:-2])

            except Exception as e:
                self.get_logger().warn(f"LiDAR WebSocket error: {e}")
                await asyncio.sleep(1.0)

    # =============================================================
    #               /cmd_vel  →  HTTP /setSpeed
    # =============================================================
    def cmd_cb(self, msg: Twist) -> None:
        # Преобразуем twist в скорости колёс (мм/с)
        v_mm = msg.linear.x * 1000.0      # м/с → мм/с
        w = msg.angular.z                 # рад/с
        
        # Дифференциальная кинематика
        left = int(v_mm - w * BASE_MM / 2.0)
        right = int(v_mm + w * BASE_MM / 2.0)

        qs = "/setSpeed?" + urllib.parse.urlencode({"l": left, "r": right})
        try:
            response = requests.get(f"{self.http_base_url}{qs}", timeout=0.15)
            if response.status_code != 200:
                self.get_logger().warn(f"/setSpeed returned {response.status_code}")
        except Exception as e:
            self.get_logger().warn(f"/setSpeed request failed: {e}")

    # =============================================================
    #               преобразуем пакеты LDS → LaserScan
    # =============================================================
    def publish_scan(self, raw: bytes) -> None:
        pkt_cnt = len(raw) // FRAME_SZ
        if pkt_cnt < 10:  # минимальное количество пакетов
            self.get_logger().debug(f"Too few packets: {pkt_cnt}")
            return

        ang, rng, inten = [], [], []
        prev = None
        off = 0.0

        for i in range(pkt_cnt):
            try:
                s, e, *dist = struct.unpack_from(FRAME_FMT, raw, i * FRAME_SZ)
                s /= 100.0  # угол начала в градусах
                e /= 100.0  # угол конца в градусах
                
                # Обработка перехода через 360°
                if e < s:
                    e += 360.0

                # 8 точек в пакете
                for j, d in enumerate(dist):
                    a = s + (e - s) * j / 7  # интерполяция угла
                    
                    # Обработка непрерывности углов
                    if prev is not None and a + off < prev - 300:
                        off += 360.0
                    a += off
                    prev = a

                    ang.append(a)
                    # Дистанция в метрах, 0 означает отсутствие сигнала
                    rng.append(d / 1000.0 if d > 0 else float("inf"))
                    inten.append(1.0 if d > 0 else 0.0)
                    
            except struct.error as e:
                self.get_logger().warn(f"Packet {i} unpack error: {e}")
                continue

        if len(ang) < 10:
            self.get_logger().debug("No valid points in scan")
            return

        # Применяем зеркальное отражение если включено
        if self.get_parameter("lidar_mirror").value:
            ang.reverse()
            rng.reverse()
            inten.reverse()
            ang = [-a for a in ang]

        # Добавляем поворот по yaw
        yaw_deg = self.get_parameter("lidar_yaw_deg").value
        if yaw_deg != 0.0:
            ang = [a + yaw_deg for a in ang]

        # Нормализуем углы чтобы начало было около -π
        while ang[0] > 180.0:
            ang = [a - 360.0 for a in ang]
        while ang[0] < -180.0:
            ang = [a + 360.0 for a in ang]

        # Преобразуем в радианы
        rad = [math.radians(a) for a in ang]

        # Создаём LaserScan сообщение
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

    # =============================================================
    #                 HTTP /state → /odom  (+ TF)
    # =============================================================
    def poll_state(self) -> None:
        try:
            response = requests.get(f"{self.http_base_url}/state", timeout=0.25)
            if response.status_code != 200:
                self.get_logger().debug(f"/state returned {response.status_code}")
                return
            js = response.json()
        except Exception as e:
            self.get_logger().debug(f"/state request failed: {e}")
            return

        try:
            odom_data = js["odom"]
            speed_data = js["speed"]
            
            x, y, th = odom_data["x"], odom_data["y"], odom_data["th"]
            speed_left = speed_data["left"]   # мм/с
            speed_right = speed_data["right"] # мм/с

            # Вычисляем линейную и угловую скорости
            v_linear = (speed_left + speed_right) / 2000.0  # м/с
            v_angular = (speed_right - speed_left) / BASE_MM  # рад/с

            # Публикуем одометрию
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

            # Публикуем динамический TF odom → base_link
            tf = TransformStamped()
            tf.header.stamp = od.header.stamp
            tf.header.frame_id = od.header.frame_id
            tf.child_frame_id = od.child_frame_id
            tf.transform.translation.x = float(x)
            tf.transform.translation.y = float(y)
            tf.transform.translation.z = 0.0
            tf.transform.rotation = od.pose.pose.orientation
            self.tf_br.sendTransform(tf)
            
        except KeyError as e:
            self.get_logger().warn(f"Missing field in /state response: {e}")
        except (TypeError, ValueError) as e:
            self.get_logger().warn(f"Invalid data in /state response: {e}")


# ――― точка входа ――――――――――――――――――――――――――――――――――――――――――――――――――
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