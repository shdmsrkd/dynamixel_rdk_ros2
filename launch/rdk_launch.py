from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
         Node(
            package='dynamixel_rdk_ros2',
            executable='dynamixel_rdk_ros2',
            name='dynamixel_rdk_ros2',
            parameters=['/home/eungang/ros2_ws/src/dynamixel_rdk_ros2/config/dynamixel.yaml']
             )
    ])
