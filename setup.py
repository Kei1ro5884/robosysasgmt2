import os
from glob import glob
from setuptools import setup

package_name = 'robosysasgmt2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), ['config/config.yml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Keiichiro Kobayashi',
    maintainer_email='s23c1050eb@s.chibakoudai.jp',
    description='Weather forecast ROS2 package',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'weather_forecast_node = robosysasgmt2.weather_forecast_node:main',
        ],
    },
)

