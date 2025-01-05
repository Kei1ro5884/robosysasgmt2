# SPDX-FileCopyrightText: 2024 Keiichiro Kobayashi
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import yml
import os

class WeatherForecastNode(Node):
    def __init__(self):
        super().__init__('weather_forecast_node')
        
        self.config = self.load_config(
    os.path.join(self.get_parameter("config_file").get_parameter_value().string_value)
)

        self.api_key = self.config.get('api_key', None)
        self.city = self.config.get('city', 'Tokyo')

        if not self.api_key:
            self.get_logger().error("API key is missing in config.yaml. Please provide a valid API key.")
            rclpy.shutdown()
            return

        self.publisher_ = self.create_publisher(String, 'weather_forecast', 10)
        self.timer = self.create_timer(3600.0, self.publish_weather)  
        self.get_logger().info(f"Weather forecast node initialized for city: {self.city}")

    def load_config(self, file_path):
        try:
            with open(file_path, "r") as file:
                return yml.safe_load(file)
        except FileNotFoundError:
            self.get_logger().error(f"Configuration file {file_path} not found.")
            rclpy.shutdown()
            return {}
        except yml.YMLError as e:
            self.get_logger().error(f"Error parsing configuration file: {e}")
            rclpy.shutdown()
            return {}

    def fetch_weather(self):
        base_url = "http://api.openweathermap.org/data/2.5/weather"
        params = {
            'q': self.city,
            'appid': self.api_key,
            'units': 'metric',
            'lang': 'ja'
        }
        try:
            response = requests.get(base_url, params=params)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error fetching weather data: {e}")
            return None

    def publish_weather(self):
        weather_data = self.fetch_weather()
        if weather_data:
            description = weather_data['weather'][0]['description']
            temp = weather_data['main']['temp']
            humidity = weather_data['main']['humidity']
            wind_speed = weather_data['wind']['speed']
            
            weather_message = (
                f"現在の天気: {description}\n"
                f"気温: {temp}°C\n"
                f"湿度: {humidity}%\n"
                f"風速: {wind_speed}m/s\n"
            )
            self.get_logger().info(f"Publishing weather: \n{weather_message}")
            msg = String()
            msg.data = weather_message
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    weather_forecast_node = WeatherForecastNode()
    try:
        rclpy.spin(weather_forecast_node)
    except KeyboardInterrupt:
        weather_forecast_node.get_logger().info('Shutting down weather forecast node.')
    finally:
        weather_forecast_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

