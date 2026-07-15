from setuptools import setup
import os
from glob import glob


package_name = 'landing_pad_detector'


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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vsor198',
    maintainer_email='vsor198@aucklanduni.ac.nz',
    description='Low-light ArUco landing-pad detection for OV9281 on Jetson Nano',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Standalone detector: subscribes to camera topics from any source.
            'detector_node = landing_pad_detector.detector_node:main',
            # Single-process co-location of camera + detector (see composed.py).
            'composed = landing_pad_detector.composed:main',
        ],
    },
)
