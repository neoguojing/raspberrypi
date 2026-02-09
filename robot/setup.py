from setuptools import setup,find_packages
import os
from glob import glob

package_name = 'robot' 

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 这一行非常重要：确保 launch 文件能被找到
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros_user',
    description='Camera and Car Driver Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 格式： '可执行文件名 = 包名.文件名:main函数'
            'car_driver_node = robot.car_driver_node:main',
            'camera_publisher_node = robot.camera_publisher_node:main',
            'icm20948_spi_node = robot.icm20948_spi_node:main',
            'static_tf_pub_node = robot.static_tf_pub_node:main',
            'seg_to_scan_node = robot.seg_to_scan_node:main',
            'manual_nav_commander = robot.manual_nav_commander:main',
            'explore_node = robot.explore_node:main',
            'continuous_explorer = robot.continuous_explorer:main',

        ],
    },
)