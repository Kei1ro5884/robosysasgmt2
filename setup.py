from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'robosysasgmt2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kei1ro',
    maintainer_email='kei1ro@example.com',
    description='Weather forecast ROS2 package',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'weather_forecast_node = robosysasgmt2.weather_forecast_node:main',
        ],
    },
)

