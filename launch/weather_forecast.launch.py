from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robosysasgmt2',
            executable='weather_forecast_node',
            name='weather_forecast_node',
            output='screen',
            parameters=[
                {'config_file': '/home/kei1ro/ros2_ws/src/robosysasgmt2/config/config.yaml'}
            ]
        )
    ])

