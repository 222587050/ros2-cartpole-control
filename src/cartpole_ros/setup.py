from setuptools import find_packages, setup

package_name = "cartpole_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/cartpole.launch.py"]),
        ("share/" + package_name + "/config", ["config/cartpole.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mehmet Onal",
    maintainer_email="TODO@example.com",
    description="Cart-pole simulation and LQR/PID control as ROS 2 nodes",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_node = cartpole_ros.sim_node:main",
            "controller_node = cartpole_ros.controller_node:main",
            "logger_node = cartpole_ros.logger_node:main",
        ],
    },
)
