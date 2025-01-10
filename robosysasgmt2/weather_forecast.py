# SPDX-FileCopyrightText: 2025 Keiichiro Kobayashi
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import yaml
import os


class WeatherForecastNode(Node):
    def __init__(self):
        super().__init__("weather_forecast")
        self.publisher_ = self.create_publisher(String, "weather_forecast", 10)
        self.timer = self.create_timer(60.0, self.publish_weather_forecast)
        
        config_file_path = os.path.join(
            os.getenv('AMENT_PREFIX_PATH', ''),
            'share', 'robosysasgmt2', 'config', 'config.yml'
        )
        
        with open(config_file_path, "r") as file:
            config = yaml.safe_load(file)
        
        self.api_key = config["weather"]["api_key"]
        self.city_name = config["weather"]["city_name"]
        
        self.get_logger().info("WeatherForecastNode has been started.")

    def get_weather_forecast(self):
        url = f"https://api.openweathermap.org/data/2.5/weather?q={self.city_name}&units=metric&appid={self.api_key}"
        try:
            response = requests.get(url)
            response.raise_for_status()
            data = response.json()
            weather_description = data["weather"][0]["description"]
            temperature = data["main"]["temp"]
            weather_info = f"City: {self.city_name}, Weather: {weather_description}, Temp: {temperature}°C"
            return weather_info
        except requests.RequestException as e:
            self.get_logger().error(f"Failed to fetch weather data: {e}")
            return "Error fetching weather data"

    def publish_weather_forecast(self):
        weather_info = self.get_weather_forecast()
        msg = String()
        msg.data = weather_info
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = WeatherForecastNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

