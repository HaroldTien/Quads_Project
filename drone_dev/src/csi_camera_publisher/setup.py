from setuptools import setup
import os
from glob import glob




package_name = 'csi_camera_publisher'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'data'),
            glob('data/*.npy')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harold',
    maintainer_email='a142758369@gmail.com',
    description='ROS 2 node for OV9281 CSI camera on Jetson Orin Nano',
    license='MIT',
    entry_points={
        'console_scripts': [
            'csi_camera_node = csi_camera_publisher.csi_camera_node:main',
        ],
    },
)
