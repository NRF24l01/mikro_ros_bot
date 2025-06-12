import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool # <-- Импортируем Bool для статуса ESP
from flask import Flask, render_template_string, request
from flask_socketio import SocketIO, emit
import threading
import socket

# --- Flask Web Server with SocketIO ---
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret_ros_joystick_key!'
socketio = SocketIO(app, async_mode=None)

node_instance = None

HTML_TEMPLATE_WITH_JS = """
<!DOCTYPE html>
<html>
<head>
    <title>ROS2 WebSocket Joystick</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no, maximum-scale=1.0">
    <style>
        body { 
            display: flex; 
            flex-direction: column;
            justify-content: center; 
            align-items: center; 
            height: 100vh; 
            margin: 0; 
            background-color: #2c3e50; 
            font-family: Arial, sans-serif;
            color: #ecf0f1;
            touch-action: none; 
            overflow: hidden; 
        }
        #joystick_container { 
            width: 200px; 
            height: 200px; 
            background-color: #34495e; 
            border-radius: 50%; 
            position: relative; 
            border: 3px solid #7f8c8d; 
            box-shadow: 0px 0px 15px rgba(0,0,0,0.3);
            margin-bottom: 20px; /* Добавим отступ снизу */
        }
        #joystick_handle { 
            width: 70px; 
            height: 70px; 
            background-color: #95a5a6; 
            border-radius: 50%; 
            position: absolute; 
            top: calc(50% - 35px); 
            left: calc(50% - 35px); 
            cursor: grab; 
            border: 2px solid #bdc3c7;
            box-shadow: inset 0px 0px 10px rgba(0,0,0,0.2);
        }
        .status-container {
            display: flex;
            flex-direction: column; /* Статусы один под другим */
            align-items: center;
            margin-top: 10px;
        }
        .status { 
            margin-top: 8px; /* Уменьшим отступ между статусами */
            font-size: 1.0em;
            background-color: #34495e;
            padding: 8px 15px;
            border-radius: 5px;
            box-shadow: 0px 0px 10px rgba(0,0,0,0.2);
            min-width: 280px; 
            text-align: center;
        }
        .instructions {
            margin-bottom: 20px;
            font-size: 0.9em;
            color: #bdc3c7;
        }
        .esp-status-online {
            color: #2ecc71; /* Зеленый для онлайн */
        }
        .esp-status-offline {
            color: #e74c3c; /* Красный для оффлайн */
        }
    </style>
    <!-- Используйте локальный файл или CDN, как вы настроили -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.7.5/socket.io.min.js"></script>
</head>
<body>
    <div class="instructions">Управляйте джойстиком. Отпускание останавливает робота.</div>
    <div id="joystick_container">
        <div id="joystick_handle"></div>
    </div>
    <div class="status-container">
        <div class="status" id="velocity_status_display">Linear: 0.00 m/s, Angular: 0.00 rad/s</div>
        <div class="status" id="esp_status_display">ESP Status: Unknown</div>
    </div>

    <script>
        const joystickContainer = document.getElementById('joystick_container');
        const joystickHandle = document.getElementById('joystick_handle');
        const velocityStatusDisplay = document.getElementById('velocity_status_display');
        const espStatusDisplay = document.getElementById('esp_status_display');

        const socket = io();

        socket.on('connect', () => {
            console.log('Connected to WebSocket server');
            socket.emit('request_initial_esp_status');
        });

        socket.on('disconnect', () => {
            console.log('Disconnected from WebSocket server');
        });

        socket.on('velocity_update', function(data) {
            velocityStatusDisplay.textContent = `Linear: ${data.linear_x.toFixed(2)} m/s, Angular: ${data.angular_z.toFixed(2)} rad/s`;
        });

        socket.on('esp_status_update', function(data) {
            const isOnline = data.connected;
            espStatusDisplay.textContent = `ESP Status: ${isOnline ? 'Online' : 'Offline'}`;
            espStatusDisplay.className = 'status'; 
            if (isOnline) {
                espStatusDisplay.classList.add('esp-status-online');
            } else {
                espStatusDisplay.classList.add('esp-status-offline');
            }
        });

        let isDragging = false;
        let containerRect = joystickContainer.getBoundingClientRect();

        function updateContainerRect() {
            containerRect = joystickContainer.getBoundingClientRect();
        }
        window.addEventListener('load', updateContainerRect);
        window.addEventListener('resize', updateContainerRect);

        const centerX = joystickContainer.offsetWidth / 2;
        const centerY = joystickContainer.offsetHeight / 2;
        const handleRadius = joystickHandle.offsetWidth / 2;
        const maxDisplacement = centerX - handleRadius;

        let normalizedX = 0;
        let normalizedY = 0;
        
        let lastSent = 0;
        let sendInterval = 150; // ms
        let lastX = 0, lastY = 0;

        function getEventCoordinates(event) {
            if (event.touches && event.touches.length > 0) {
                return { clientX: event.touches[0].clientX, clientY: event.touches[0].clientY };
            }
            return { clientX: event.clientX, clientY: event.clientY };
        }

        function updateJoystickPosition(event) {
            if (!isDragging) return;

            const coords = getEventCoordinates(event);
            let eventX = coords.clientX - containerRect.left;
            let eventY = coords.clientY - containerRect.top;

            let dx = eventX - centerX;
            let dy = eventY - centerY;

            const distance = Math.sqrt(dx * dx + dy * dy);
            if (distance > maxDisplacement) {
                dx = (dx / distance) * maxDisplacement;
                dy = (dy / distance) * maxDisplacement;
            }

            joystickHandle.style.left = (centerX + dx - handleRadius) + 'px';
            joystickHandle.style.top = (centerY + dy - handleRadius) + 'px';

            normalizedY = parseFloat(-(dy / maxDisplacement).toFixed(2))*0.5; 
            normalizedX = parseFloat((dx / maxDisplacement).toFixed(2))*0.5;
        }

        function sendJoystickData(x, y) {
            socket.emit('joystick_command', { x: x, y: y });
            lastSent = Date.now();
        }

        function scheduleJoystickData() {
            setInterval(() => {
                sendJoystickData(normalizedX, normalizedY);
            }, sendInterval);
        }


        function startDrag(event) {
            isDragging = true;
            joystickHandle.style.cursor = 'grabbing';
            updateContainerRect(); 
            updateJoystickPosition(event);
            if (event.cancelable) event.preventDefault();
        }

        function drag(event) {
            updateJoystickPosition(event);
            if (isDragging && event.cancelable) event.preventDefault();
        }

        function endDrag() {
            if (isDragging) {
                isDragging = false;
                joystickHandle.style.cursor = 'grab';
                joystickHandle.style.left = (centerX - handleRadius) + 'px';
                joystickHandle.style.top = (centerY - handleRadius) + 'px';
                // Сбросить отложенную отправку и сразу отправить (0,0)
                lastX = 0;
                lastY = 0;
                normalizedY = 0; 
                normalizedX = 0;
                sendJoystickData(0, 0); 
                pending = false;
            }
        }

        joystickContainer.addEventListener('mousedown', startDrag);
        document.addEventListener('mousemove', drag);
        document.addEventListener('mouseup', endDrag);

        joystickContainer.addEventListener('touchstart', startDrag, { passive: false });
        document.addEventListener('touchmove', drag, { passive: false });
        document.addEventListener('touchend', endDrag);
        document.addEventListener('touchcancel', endDrag);
        
        scheduleJoystickData();
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE_WITH_JS)

@socketio.on('joystick_command')
def handle_joystick_command(data):
    global node_instance
    if node_instance:
        joystick_y = float(data.get('y', 0.0))
        joystick_x = float(data.get('x', 0.0))

        twist_msg = Twist()
        twist_msg.linear.x = joystick_y * node_instance.max_linear_speed
        twist_msg.angular.z = joystick_x * node_instance.max_angular_speed * (-1.0) * 3

        node_instance.publisher_cmd_vel.publish(twist_msg)
        
        client_sid = request.sid
        socketio.emit('velocity_update',
                      {'linear_x': twist_msg.linear.x, 'angular_z': twist_msg.angular.z},
                      room=client_sid)

@socketio.on('request_initial_esp_status')
def handle_request_initial_esp_status():
    global node_instance
    if node_instance:
        client_sid = request.sid
        socketio.emit('esp_status_update', {'connected': node_instance.esp_connected_status}, room=client_sid)
        node_instance.get_logger().info(f"Sent initial ESP status ({node_instance.esp_connected_status}) to client {client_sid}")


class WebJoystick(Node):
    def __init__(self):
        super().__init__('web_joystick_node')
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.esp_status_subscriber = self.create_subscription(
            Bool,
            '/esp_connection_status',
            self.esp_status_callback,
            10)
        self.esp_connected_status = False 

        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value

        self.get_logger().info('Web Joystick Node (WebSocket) запущен.')
        self.get_logger().info(f'Max linear speed: {self.max_linear_speed} m/s, Max angular speed: {self.max_angular_speed} rad/s')
        self.get_logger().info('Subscribed to /esp_connection_status')
        
        global node_instance
        node_instance = self

    def esp_status_callback(self, msg):
        new_status = msg.data
        if new_status != self.esp_connected_status:
            self.get_logger().info(f'ESP Connection Status changed: {new_status}')
            self.esp_connected_status = new_status
            # ИЗМЕНЕНИЕ: Удаляем аргумент broadcast=True
            # Отправка всем клиентам в пространстве имен по умолчанию
            socketio.emit('esp_status_update', {'connected': self.esp_connected_status})

    def get_ip_address(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.1)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            try:
                return socket.gethostbyname(socket.gethostname())
            except socket.gaierror:
                return "127.0.0.1"

def run_flask_socketio(node_logger):
    host_ip = node_instance.get_ip_address()
    port = 5000
    node_logger.info(f'Веб-сервер (SocketIO) доступен по адресу: http://{host_ip}:{port} и http://localhost:{port}')
    try:
        socketio.run(app, host='0.0.0.0', port=port, debug=False, use_reloader=False, allow_unsafe_werkzeug=True if not (socketio.async_mode == 'eventlet' or socketio.async_mode == 'gevent') else False)
    except OSError as e:
        node_logger.error(f"Не удалось запустить Flask-SocketIO сервер на {host_ip}:{port}. Ошибка: {e}")

def main(args=None):
    rclpy.init(args=args)
    web_joystick_node = WebJoystick()

    flask_thread = threading.Thread(target=run_flask_socketio, args=(web_joystick_node.get_logger(),))
    flask_thread.daemon = True
    flask_thread.start()

    try:
        rclpy.spin(web_joystick_node)
    except KeyboardInterrupt:
        web_joystick_node.get_logger().info('Прерывание с клавиатуры, выключение...')
    finally:
        web_joystick_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()