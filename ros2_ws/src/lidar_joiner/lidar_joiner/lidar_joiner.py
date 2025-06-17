import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from math import pi

class RotationalAssembler(Node):
    def __init__(self):
        super().__init__('rotational_lidar_assembler')

        self.declare_parameter('lidar_mirror', True)  # Новый параметр

        self.subscription = self.create_subscription(
            LaserScan,
            '/sts/esp/scan',
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(LaserScan, '/sts/scan', 10)

        self.buffer = []
        self.last_processed_angle = None

    def scan_callback(self, msg: LaserScan):
        if self.last_processed_angle is None:
            self.last_processed_angle = msg.angle_max
        elif self.last_processed_angle >= msg.angle_min:
            self.publish_scan()
        
        for i, dist in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            self.buffer.append((angle, dist))
            
        self.last_processed_angle = msg.angle_max

    def publish_scan(self):
        # Sort buffer by angle
        self.buffer.sort(key=lambda x: x[0])
        angles, dists = zip(*self.buffer)

        # Create evenly spaced angle bins
        angle_min = min(angles)
        angle_max = max(angles)
        num_points = len(self.buffer)
        if num_points < 2:
            self.buffer.clear()
            return

        if angle_max - angle_min == 0:
            return
        
        angle_increment = (angle_max - angle_min) / (num_points - 1)
        ranges = [float('nan')] * num_points

        # Fill ranges with distances at corresponding bins
        for angle, dist in self.buffer:
            idx = round((angle - angle_min) / angle_increment)
            if 0 <= idx < num_points:
                ranges[idx] = dist

        # --- РЕВЕРС, если требуется ---
        if self.get_parameter('lidar_mirror').value:
            # reverse angles and ranges, change sign of angles
            ranges = ranges[::-1]
            # Пересчитаем углы
            angles_reversed = [angle_min + i * angle_increment for i in range(num_points)][::-1]
            angles_reversed = [-a for a in angles_reversed]
            angle_min = min(angles_reversed)
            angle_max = max(angles_reversed)
            # Пересчитаем angle_increment (может стать отрицательным, но для последовательности важно abs)
            if num_points > 1:
                angle_increment = (angle_max - angle_min) / (num_points - 1)
            else:
                angle_increment = 0.0

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'sts_laser'
        scan_msg.angle_min = angle_min
        scan_msg.angle_max = angle_max
        scan_msg.angle_increment = angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.0
        scan_msg.range_min = 0.0
        scan_msg.range_max = max([d for d in dists if d == d], default=0.0)
        scan_msg.ranges = ranges

        self.publisher.publish(scan_msg)
        self.buffer.clear()
        
        self.get_logger().info(f"Published assembled scan with {len(ranges)} points.")

def main(args=None):
    rclpy.init(args=args)
    node = RotationalAssembler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()