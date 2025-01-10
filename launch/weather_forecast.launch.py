import launch
import launch.actions
import launch.substitutions
import launch_ros.action

def generate_launch_description():
    weather_forecast_node = launch_ros.actions.Node(
            package='robosysasgmt2',
            executable='weather_forecast_node',
            output='screen',
        ),
    return launch.LaunchDescription([weather_forecast_node])

