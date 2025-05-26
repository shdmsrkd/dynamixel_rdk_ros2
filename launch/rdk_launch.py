from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
         Node(
            package='dynamixel_rdk_ros2',
            executable='dynamixel_communication_node',
            name='dynamixel_communication_node',
            output='screen',
         ),

         Node(
            package='dynamixel_rdk_ros2',
            executable='motor_status_node',
            name='motor_status_node',
            output='screen',
         ),

         Node(
            package='dynamixel_rdk_ros2',
            executable='motor_control_node',
            name='motor_control_node',
            output='screen',
         )
    ])
