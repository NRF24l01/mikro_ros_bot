from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ESP Talker Node
        Node(
            package='esp_talker',
            executable='esp_talker',
            name='esp_talker',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'esp_ip_address': '192.168.0.103'}, # Using the IP from your log
                {'state_fetch_interval_sec': 0.5},   # SLOWED DOWN: Was 0.1. Let's try 2 Hz
                {'request_timeout_sec': 1.0},        # INCREASED: Was 0.5. More patient.
                {'odom_frame_id': 'odom'},
                {'base_frame_id': 'base_link'}
            ]
        ),
        # Static Transform Publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_base_link_laser',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),
        # Web Joystick
        Node(
            package='web_joystick',
            executable='web',
            name='web_joystick',
            output='screen'
        ),
        # Lidar Reader
        Node(
            package='lidar_node',
            executable='lidar_reader',
            name='lidar_reader',
            output='screen'
        )
    ])