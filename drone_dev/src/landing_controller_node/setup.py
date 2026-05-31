from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'landing_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the yaml and launch file into the package share dir so
        # the launch file can find them at runtime.
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harold',
    maintainer_email='a142758369@gmail.com',
    description='Precision landing controller: ArUco pose -> MAVROS velocity setpoints',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # executable_name = module.path:function
            'landing_controller_node = landing_controller.landing_controller_node:main',
        ],
    },
)