from setuptools import setup
import os
from glob import glob

package_name = 'robosysasgmt2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Keiichiro Kobayashi',
    maintainer_email='s23c1050eb@s.chibakoudai.jp',
    description='A ROS2 package for weather forecasting using OpenWeatherMap API',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'weather_forecast_node = robosysasgmt2.weather_forecast_node:main',
        ],
    },
)

