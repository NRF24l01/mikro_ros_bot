import asyncio, struct, math, threading, socket, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import websockets
import time

# --- tf2 imports ---
from geometry_msgs.msg import Quaternion
from tf2_ros import (
    StaticTransformBroadcaster,
    TransformStamped,
)

POINTS_PER_PACKET = 80
FRAME_FMT = "<HH" + "H"*POINTS_PER_PACKET
FRAME_SZ  = 4 + 2*POINTS_PER_PACKET  # 164

READ_TIMEOUT   = 1.2
MIN_FPS        = 10
LOW_FPS_STREAK = 3
MAX_BACKOFF    = 30.0

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

class WSBridge(Node):
    def __init__(self):
        super().__init__("lidar_ws_bridge")

        self.declare_parameter("host", "192.168.0.109")
        self.declare_parameter("port", 8888)
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 16.0)
        self.declare_parameter("lidar_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("lidar_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("base_frame", "base_frame")

        self.url = f"ws://{self.get_parameter('host').value}:" \
                   f"{self.get_parameter('port').value}/ws"

        self.scan_pub = self.create_publisher(LaserScan, "/sts/esp/scan", 10)
        
        loop = asyncio.new_event_loop()
        threading.Thread(target=self.ws_thread,
                         args=(loop,), daemon=True).start()
        
        # -------- добавляем tf broadcaster --------
        self.static_br = StaticTransformBroadcaster(self)
        self._publish_static_tf()

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
        
        # tf = TransformStamped()
        # tf.header.stamp = self.get_clock().now().to_msg()
        # tf.header.frame_id = "sts_map"
        # tf.child_frame_id = "sts_odom"
        # tf.transform.translation.x = 0.0
        # tf.transform.translation.y = 0.0
        # tf.transform.translation.z = 0.0
        # tf.transform.rotation = rpy_to_quat(0.0,0.0,0.0)

        # # latched – публикуем один раз
        # self.static_br.sendTransform(tf)
        # self.get_logger().info("Published static TF map → odom")

    def ws_thread(self, loop):
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.ws_loop())
    
    async def ws_loop(self):
        backoff          = 1.0
        low_fps_counter  = 0

        while rclpy.ok():
            try:
                async with websockets.connect(
                    self.url,
                    max_size=None,
                    open_timeout=3.0,
                    ping_interval=None
                ) as ws:
                    sock = ws.transport.get_extra_info("socket")
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 10)
                    self.get_logger().info(f"connected → {self.url}")

                    backoff = 1.0
                    t0, frames = time.time(), 0
                    low_fps_counter = 0

                    while rclpy.ok():
                        try:
                            pkt = await asyncio.wait_for(ws.recv(), timeout=READ_TIMEOUT)
                        except asyncio.TimeoutError:
                            pass
                        else:
                            if len(pkt) == FRAME_SZ:
                                self.publish_scan(pkt)
                                frames += 1
                            else:
                                self.get_logger().warn(f"Unexpected pkt size: {len(pkt)}")

                        if (time.time() - t0) >= 1.0:
                            fps = frames
                            # self.get_logger().info(f"ESP: {fps} пакетов/с")
                            if fps < MIN_FPS:
                                low_fps_counter += 1
                                if low_fps_counter >= LOW_FPS_STREAK:
                                    self.get_logger().warn(
                                        f"FPS < {MIN_FPS} {LOW_FPS_STREAK} с подряд — reconnect")
                                    break
                            else:
                                low_fps_counter = 0
                            t0, frames = time.time(), 0

                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, MAX_BACKOFF)

            except Exception as e:
                self.get_logger().error(f"WS error: {e}")
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, MAX_BACKOFF)

    def publish_scan(self, pkt: bytes):
        stamp = self.get_clock().now().to_msg()
        unpacked = struct.unpack_from(FRAME_FMT, pkt)
        start_deg, end_deg = unpacked[0] / 100.0, unpacked[1] / 100.0
        dist_mm = unpacked[2:]

        if end_deg < start_deg:
            end_deg += 360.0
        angle_min = math.radians(start_deg)
        angle_max = math.radians(end_deg)
        if POINTS_PER_PACKET > 1:
            angle_inc = (angle_max - angle_min) / (POINTS_PER_PACKET - 1)
        else:
            angle_inc = 0.0

        scan = LaserScan()
        scan.header.stamp    = stamp
        scan.header.frame_id = self.get_parameter('frame_id').value
        scan.angle_min       = angle_min
        scan.angle_max       = angle_max
        scan.angle_increment = angle_inc
        scan.scan_time       = 0.004
        scan.time_increment  = 0.0
        scan.range_min       = self.get_parameter('range_min').value
        scan.range_max       = self.get_parameter('range_max').value
        scan.ranges          = [
            d/1000.0 if d else float('inf') for d in dist_mm]
        scan.intensities     = [0.0]*POINTS_PER_PACKET
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = WSBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()