import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('dynamixel_rdk_ros2'),
        'config',
        'dynamixel.yaml'
    )

    dynamixel_rdk_node = Node(
        package='dynamixel_rdk_ros2',
        executable='dynamixel_rdk_ros2',
        name='dynamixel_rdk_ros2_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_dir]
    )

    return LaunchDescription([dynamixel_rdk_node])