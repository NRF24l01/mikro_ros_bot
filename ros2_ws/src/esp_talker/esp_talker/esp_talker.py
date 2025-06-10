import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry # ADDED
import requests # For HTTP requests
import time
import math # For quaternion conversion

def euler_to_quaternion(yaw, pitch=0.0, roll=0.0) -> Quaternion:
    """Converts Euler angles (yaw, pitch, roll) to a Quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

class EspControllerNode(Node):
    def __init__(self):
        super().__init__('esp_talker')

        # Declare parameters
        self.declare_parameter('esp_ip_address', '192.168.0.102')
        self.declare_parameter('state_fetch_interval_sec', 0.1) # Renamed and potentially faster for odom
        self.declare_parameter('request_timeout_sec', 0.5)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')

        # Get parameters
        self.esp_ip = self.get_parameter('esp_ip_address').get_parameter_value().string_value
        state_fetch_interval = self.get_parameter('state_fetch_interval_sec').get_parameter_value().double_value
        self.timeout = self.get_parameter('request_timeout_sec').get_parameter_value().double_value
        self.odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value


        self.get_logger().info(f"ESP32 IP Address: {self.esp_ip}")
        self.get_logger().info(f"State Fetch Interval: {state_fetch_interval}s")
        self.get_logger().info(f"Request Timeout: {self.timeout}s")
        self.get_logger().info(f"Odometry Frame ID: {self.odom_frame}")
        self.get_logger().info(f"Base Frame ID: {self.base_frame}")

        # Subscriber to /cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Publisher for battery voltage
        self.battery_publisher = self.create_publisher(
            Float32,
            '/battery_voltage',
            10)

        # Publisher for odometry
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10) # QoS can be adjusted if needed, 10 is default

        # Timer to fetch state (odometry and battery)
        self.state_timer = self.create_timer(
            state_fetch_interval,
            self.fetch_state_callback) # Renamed callback

        self.get_logger().info("ESP Controller Node started.")
        self.get_logger().info(f"Listening for /cmd_vel messages...")
        self.get_logger().info(f"Will publish to /battery_voltage and /odom periodically.")
        self.get_logger().info(f"Attempting to connect to ESP32 at http://{self.esp_ip}")


    def cmd_vel_callback(self, msg: Twist):
        v_esp = msg.linear.x * 1000.0  # m/s to mm/s
        w_esp = msg.angular.z          # rad/s

        url = f"http://{self.esp_ip}/setVW?v={v_esp:.2f}&w={w_esp:.3f}"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
        except requests.exceptions.Timeout:
            self.get_logger().warn(f"Timeout sending command to ESP32 at {url}")
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f"Error sending command to ESP32: {e}")

    def fetch_state_callback(self):
        url = f"http://{self.esp_ip}/state" # ESP endpoint for all state data
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            data = response.json()

            # --- Battery Voltage ---
            if 'battery_v' in data:
                voltage = float(data.get('battery_v', 0.0))
                battery_msg = Float32()
                battery_msg.data = voltage
                self.battery_publisher.publish(battery_msg)
            else:
                self.get_logger().warn("Battery voltage ('battery_v') not found in /state response.")

            # --- Odometry ---
            if 'odom' in data and isinstance(data['odom'], dict):
                odom_data = data['odom']
                # Expected keys: x, y, theta, v, w
                if all(k in odom_data for k in ('x', 'y', 'theta', 'v', 'w')):
                    current_time = self.get_clock().now().to_msg()

                    odom_msg = Odometry()
                    odom_msg.header.stamp = current_time
                    odom_msg.header.frame_id = self.odom_frame
                    odom_msg.child_frame_id = self.base_frame

                    # Position (ESP odom.x, odom.y are in meters)
                    odom_msg.pose.pose.position.x = float(odom_data['x'])
                    odom_msg.pose.pose.position.y = float(odom_data['y'])
                    odom_msg.pose.pose.position.z = 0.0 # Assuming 2D

                    # Orientation (ESP odom.theta is in radians)
                    odom_msg.pose.pose.orientation = euler_to_quaternion(yaw=float(odom_data['theta']))

                    # Twist (ESP odom.v is in m/s, odom.w is in rad/s)
                    odom_msg.twist.twist.linear.x = float(odom_data['v'])
                    odom_msg.twist.twist.linear.y = 0.0
                    odom_msg.twist.twist.linear.z = 0.0
                    odom_msg.twist.twist.angular.x = 0.0
                    odom_msg.twist.twist.angular.y = 0.0
                    odom_msg.twist.twist.angular.z = float(odom_data['w'])

                    # Covariances (optional, can be set to indicate uncertainty)
                    # For now, leaving them as default (zeros)
                    # Example: odom_msg.pose.covariance[0] = 0.1 # X variance
                    #          odom_msg.twist.covariance[0] = 0.1 # Vx variance

                    self.odom_publisher.publish(odom_msg)
                else:
                    self.get_logger().warn(f"Odometry data from ESP32 is missing some keys. Received: {odom_data}")
            else:
                self.get_logger().warn("Odometry data ('odom') not found or not a dict in /state response.")

        except requests.exceptions.Timeout:
            self.get_logger().warn(f"Timeout fetching state from ESP32 at {url}")
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f"Error fetching state: {e}")
        except ValueError: # Includes JSONDecodeError
            self.get_logger().warn(f"Error parsing state JSON from ESP32. Response: {response.text if 'response' in locals() else 'N/A'}")


def main(args=None):
    rclpy.init(args=args)
    esp_controller_node = None  # Initialize to None
    try:
        esp_controller_node = EspControllerNode()
        rclpy.spin(esp_controller_node)
    except KeyboardInterrupt:
        if esp_controller_node:
            esp_controller_node.get_logger().info("KeyboardInterrupt, shutting down ESP Controller Node...")
    except Exception as e:
        if esp_controller_node:
            esp_controller_node.get_logger().error(f"An exception occurred: {e}")
        else:
            print(f"An exception occurred before node initialization: {e}")
    finally:
        if esp_controller_node:
            esp_controller_node.destroy_node()
        # Only shutdown if context is still valid and rclpy is initialized
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()