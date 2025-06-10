import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import time
import math
import asyncio
import websockets # For WebSocket client
import threading
import json

def euler_to_quaternion(yaw, pitch=0.0, roll=0.0) -> Quaternion:
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
        super().__init__('esp_controller_ws_node') # Renamed node

        self.declare_parameter('esp_ip_address', '192.168.0.102') # Your ESP32 IP
        self.declare_parameter('request_timeout_sec', 0.5) # Timeout for sending commands
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')

        self.esp_ip = self.get_parameter('esp_ip_address').get_parameter_value().string_value
        self.timeout = self.get_parameter('request_timeout_sec').get_parameter_value().double_value
        self.odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value
        
        self.ws_uri = f"ws://{self.esp_ip}/ws"

        self.get_logger().info(f"ESP32 WebSocket URI: {self.ws_uri}")
        self.get_logger().info(f"Odometry Frame ID: {self.odom_frame}")
        self.get_logger().info(f"Base Frame ID: {self.base_frame}")

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.battery_publisher = self.create_publisher(Float32, '/battery_voltage', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        self.ws_client = None
        self.ws_loop = None
        self.ws_thread = threading.Thread(target=self._websocket_client_thread_target, daemon=True)
        self.shutdown_event = threading.Event()


        self.get_logger().info("ESP Controller Node (WebSocket) started.")
        self.get_logger().info(f"Listening for /cmd_vel messages...")
        self.get_logger().info(f"Will publish to /battery_voltage and /odom based on data from ESP32.")
        
        self.ws_thread.start()

    def _websocket_client_thread_target(self):
        self.ws_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.ws_loop)
        try:
            self.ws_loop.run_until_complete(self.manage_websocket_connection())
        except Exception as e:
            self.get_logger().error(f"WebSocket thread exception: {e}")
        finally:
            if self.ws_loop.is_running():
                self.ws_loop.stop()
            self.ws_loop.close()
            self.get_logger().info("WebSocket client thread finished.")

    async def manage_websocket_connection(self):
        while not self.shutdown_event.is_set() and rclpy.ok():
            try:
                self.get_logger().info(f"Attempting to connect to {self.ws_uri}...")
                async with websockets.connect(self.ws_uri, ping_interval=10, ping_timeout=5) as websocket:
                    self.ws_client = websocket
                    self.get_logger().info(f"Connected to ESP32 WebSocket at {self.ws_uri}")
                    
                    # Optional: Request initial data if needed
                    # await self.send_ws_message_async({"action": "getPID"})

                    while websocket.open and not self.shutdown_event.is_set() and rclpy.ok():
                        try:
                            message_str = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                            self.process_ws_message(message_str)
                        except asyncio.TimeoutError:
                            continue # Allows checking shutdown_event and rclpy.ok()
                        except websockets.exceptions.ConnectionClosed:
                            self.get_logger().warn("WebSocket connection closed by server.")
                            break # Break inner loop to trigger reconnection attempt
                        except Exception as e:
                            self.get_logger().error(f"Error processing WebSocket message: {e}")
                            # Decide if to break or continue based on error
                            
            except (websockets.exceptions.ConnectionClosedError, ConnectionRefusedError, OSError) as e:
                self.get_logger().warn(f"WebSocket connection failed: {e}. Retrying in 5s...")
            except Exception as e:
                self.get_logger().error(f"Unhandled exception in WebSocket manager: {e}. Retrying in 5s...")
            finally:
                self.ws_client = None
                if not self.shutdown_event.is_set() and rclpy.ok():
                    await asyncio.sleep(5) # Wait before retrying
                else:
                    break # Exit if shutting down

    def process_ws_message(self, message_str: str):
        try:
            data = json.loads(message_str)
            # self.get_logger().debug(f"WS RECV: {data}") # Verbose

            msg_type = data.get("type")

            if msg_type == "state":
                # --- Battery Voltage ---
                if 'battery_v' in data:
                    voltage = float(data.get('battery_v', 0.0))
                    battery_msg = Float32()
                    battery_msg.data = voltage
                    self.battery_publisher.publish(battery_msg)
                
                # --- Odometry ---
                if 'odom' in data and isinstance(data['odom'], dict):
                    odom_data = data['odom']
                    if all(k in odom_data for k in ('x', 'y', 'theta', 'v', 'w')):
                        current_time = self.get_clock().now().to_msg()
                        odom_msg = Odometry()
                        odom_msg.header.stamp = current_time
                        odom_msg.header.frame_id = self.odom_frame
                        odom_msg.child_frame_id = self.base_frame

                        odom_msg.pose.pose.position.x = float(odom_data['x'])
                        odom_msg.pose.pose.position.y = float(odom_data['y'])
                        odom_msg.pose.pose.position.z = 0.0
                        odom_msg.pose.pose.orientation = euler_to_quaternion(yaw=float(odom_data['theta']))

                        odom_msg.twist.twist.linear.x = float(odom_data['v'])
                        odom_msg.twist.twist.linear.y = 0.0
                        odom_msg.twist.twist.linear.z = 0.0
                        odom_msg.twist.twist.angular.x = 0.0
                        odom_msg.twist.twist.angular.y = 0.0
                        odom_msg.twist.twist.angular.z = float(odom_data['w'])
                        self.odom_publisher.publish(odom_msg)
                    else:
                        self.get_logger().warn(f"Odometry data in 'state' message is missing keys. Received: {odom_data}")
                else:
                    self.get_logger().warn("'odom' field not found or not a dict in 'state' message.")

            elif msg_type == "pid_coeffs":
                self.get_logger().info(f"Received PID Coeffs: kp={data.get('kp')}, ki={data.get('ki')}, kd={data.get('kd')}, kff={data.get('kff')}")
            elif msg_type == "battery":
                 self.get_logger().info(f"Received Battery Voltage: {data.get('voltage')}V")
            elif msg_type == "status":
                self.get_logger().info(f"ESP32 Status: {data.get('message', '')} for action '{data.get('action_acked', 'N/A')}' - Status: {data.get('status', 'N/A')}")
            else:
                self.get_logger().debug(f"Received unhandled WS message type '{msg_type}': {data}")

        except json.JSONDecodeError:
            self.get_logger().warn(f"Error parsing JSON from ESP32: {message_str}")
        except Exception as e:
            self.get_logger().error(f"Error in process_ws_message: {e}")


    async def send_ws_message_async(self, payload: dict):
        if self.ws_client and self.ws_client.open:
            try:
                # self.get_logger().debug(f"WS SEND: {payload}") # Verbose
                await asyncio.wait_for(self.ws_client.send(json.dumps(payload)), timeout=self.timeout)
            except asyncio.TimeoutError:
                self.get_logger().warn(f"Timeout sending WebSocket message: {payload}")
            except websockets.exceptions.ConnectionClosed:
                 self.get_logger().warn(f"WebSocket closed while trying to send: {payload}")
            except Exception as e:
                self.get_logger().error(f"Error sending WebSocket message {payload}: {e}")
        else:
            self.get_logger().warn(f"WebSocket not connected. Cannot send: {payload}")

    def cmd_vel_callback(self, msg: Twist):
        v_esp = msg.linear.x * 1000.0  # m/s to mm/s
        w_esp = msg.angular.z          # rad/s

        payload = {"action": "setVW", "v": round(v_esp, 2), "w": round(w_esp, 3)}
        
        if self.ws_loop and self.ws_loop.is_running() and not self.shutdown_event.is_set():
            asyncio.run_coroutine_threadsafe(self.send_ws_message_async(payload), self.ws_loop)
        else:
            self.get_logger().warn(f"WebSocket loop not ready. Cannot send cmd_vel: {payload}")

    def destroy_node(self):
        self.get_logger().info("Shutting down ESP Controller Node (WebSocket)...")
        self.shutdown_event.set() # Signal the WebSocket thread to shut down
        
        # Attempt to close WebSocket connection gracefully from its own loop
        if self.ws_client and self.ws_loop and self.ws_loop.is_running():
            future = asyncio.run_coroutine_threadsafe(self.ws_client.close(), self.ws_loop)
            try:
                future.result(timeout=2.0) # Wait for close to complete
                self.get_logger().info("WebSocket connection closed via threadsafe call.")
            except asyncio.TimeoutError:
                self.get_logger().warn("Timeout waiting for WebSocket to close via threadsafe call.")
            except Exception as e:
                self.get_logger().error(f"Exception during threadsafe WebSocket close: {e}")

        if self.ws_thread.is_alive():
            self.ws_thread.join(timeout=5.0) # Wait for the thread to exit
            if self.ws_thread.is_alive():
                 self.get_logger().warn("WebSocket thread did not terminate cleanly.")
        
        super().destroy_node()
        self.get_logger().info("Node destroyed.")


def main(args=None):
    rclpy.init(args=args)
    esp_controller_node = None
    try:
        esp_controller_node = EspControllerNode()
        rclpy.spin(esp_controller_node)
    except KeyboardInterrupt:
        if esp_controller_node:
            esp_controller_node.get_logger().info("KeyboardInterrupt, shutting down...")
    except Exception as e:
        if esp_controller_node:
            esp_controller_node.get_logger().error(f"An unhandled exception occurred in main: {e}", exc_info=True)
        else:
            print(f"An exception occurred before node initialization: {e}")
    finally:
        if esp_controller_node:
            esp_controller_node.destroy_node()
        if rclpy.ok(): # Only shutdown if context is still valid and rclpy is initialized
            rclpy.shutdown()
        print("ROS 2 shutdown complete.")


if __name__ == '__main__':
    main()