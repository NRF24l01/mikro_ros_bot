import asyncio, struct, math, threading, socket, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import websockets                    # pip install websockets
import time 

FRAME_FMT = "<HHHHHHHHHH"            # 10 × uint16
FRAME_SZ  = 20
READ_TIMEOUT   = 1.2      # ждать пакет не дольше этого
MIN_FPS        = 10
LOW_FPS_STREAK = 3        # нужно 3 сек подряд < MIN_FPS
MAX_BACKOFF    = 30.0

class WSBridge(Node):
    def __init__(self):
        super().__init__("lidar_ws_bridge")

        # -------- параметры, можно менять через ros2 param --------
        self.declare_parameter("host", "192.168.0.109")
        self.declare_parameter("port", 8888)
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 16.0)

        self.url = f"ws://{self.get_parameter('host').value}:" \
                   f"{self.get_parameter('port').value}/ws"

        # -------- паблишер LaserScan --------
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)

        # -------- подписка на /cmd_vel (пока заглушка) --------
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_cb, 10)

        # -------- запускаем asyncio в отдельном треде --------
        loop = asyncio.new_event_loop()
        threading.Thread(target=self.ws_thread,
                         args=(loop,), daemon=True).start()

    # ---------------------------------------------------------
    def cmd_vel_cb(self, msg: Twist):
        # TODO: сформировать бинарный пакет управления роботом
        self.get_logger().debug(
            f"got cmd_vel lin={msg.linear.x:.2f} ang={msg.angular.z:.2f}")

    # ---------------------------------------------------------
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
                                for i in range(0, FRAME_SZ, 20):
                                    self.publish_scan(pkt[i:i+20])
                                    frames += 1

                        if (time.time() - t0) >= 1.0:
                            fps = frames
                            self.get_logger().info(f"ESP: {fps} кадр/с")

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


    # ---------------------------------------------------------
    def publish_scan(self, pkt: bytes):
        stamp = self.get_clock().now().to_msg()

        start_deg, end_deg, *dist_mm = struct.unpack_from(FRAME_FMT, pkt)
        start_deg /= 100.0
        end_deg   /= 100.0
        if end_deg < start_deg:
            end_deg += 360.0
        angle_min = math.radians(start_deg)
        angle_max = math.radians(end_deg)
        angle_inc = (angle_max - angle_min) / 7.0  # 8 точек

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
        scan.intensities     = [0.0]*8
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