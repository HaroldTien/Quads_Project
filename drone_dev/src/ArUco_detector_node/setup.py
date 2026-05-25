from setuptools import setup

package_name = "aruco_detector_node"
python_module_name = "ArUco_detector_node"

setup(
    name=package_name,
    version="0.1.0",
    packages=[python_module_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Harold",
    maintainer_email="a142758369@gmail.com",
    description="ROS 2 ArUco marker detector node with pose estimation",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aruco_detector_node = ArUco_detector_node.aruco_detector_node:main",
        ],
    },
)
