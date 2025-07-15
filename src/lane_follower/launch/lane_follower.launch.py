from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # robotaksi_controller/config/twist_filter.yaml dosyasının tam yolu
    twist_filter_config = os.path.join(
        get_package_share_directory('robotaksi_controller'),
        'config',
        'twist_filter.yaml'
    )

    return LaunchDescription([
        # 1. Şerit tespiti
        Node(
            package='lane_follower',
            executable='lane_follower',
            name='lane_follower',
            output='screen'
        ),

        # 2. Offset'e göre /cmd_vel üret
        Node(
            package='lane_follower',
            executable='lane_driver',
            name='lane_driver',
            output='screen'
        ),

        # 3. /cmd_vel'i filtrele → /ackermann_steering_controller/reference
        Node(
            package='lane_follower',
            executable='twist_filter_node',
            name='twist_filter_node',
            parameters=[twist_filter_config],  # başka paketten geldi
            output='screen'
        ),
        
            Node(
        package='lane_follower',
        executable='lane_control_pid',
        name='lane_control_pid',
        output='screen'
        ),

    ])
