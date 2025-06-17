from launch import LaunchDescription
from launch_ros.actions import Node
import math

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
                {'esp_ip_address': '192.168.0.110'}, # Using the IP from your log
                {'state_fetch_interval_sec': 0.5},   # SLOWED DOWN: Was 0.1. Let's try 2 Hz
                {'request_timeout_sec': 1.0},        # INCREASED: Was 0.5. More patient.
                {'odom_frame_id': 'sts_odom'},
                {'base_frame_id': 'sts_base_frame'}
            ]
        ),
        # Web Joystick
        Node(
            package='web_joystick',
            executable='web',
            name='web_joystick',
            output='screen'
        ),
        Node(
            package='lidar_joiner',
            executable='join',
            name='lidar_joiner',
            output='screen'
        ),
        # Lidar Reader
        Node(
            package='lidar_node',
            executable='lidar_reader',
            name='lidar_reader',
            output='screen',
            parameters=[
                {'host': "192.168.0.110"},
                {'port': 8888},
                {'frame_id': "sts_laser"},
                {'range_min': 0.05},
                {'range_max': 16.0},
                {'lidar_xyz': [0.0, 0.0, 0.1]},
                {'lidar_rpy_deg': [0.0, 0.0, -180.0]},
                {'base_frame': "sts_base_frame"},
            ]
        ),
    ])