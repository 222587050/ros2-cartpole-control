"""Üç düğümü birlikte başlatır.

    ros2 launch cartpole_ros cartpole.launch.py
    ros2 launch cartpole_ros cartpole.launch.py controller:=pid
    ros2 launch cartpole_ros cartpole.launch.py log_file:=/tmp/lqr_run.csv
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("cartpole_ros"), "config", "cartpole.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("controller", default_value="lqr", description="lqr veya pid"),
        DeclareLaunchArgument("log_file", default_value="/tmp/cartpole_log.csv"),
        Node(package="cartpole_ros", executable="sim_node", name="cartpole_sim",
             parameters=[config], output="screen"),
        Node(package="cartpole_ros", executable="controller_node", name="cartpole_controller",
             parameters=[config, {"controller_type": LaunchConfiguration("controller")}],
             output="screen"),
        Node(package="cartpole_ros", executable="logger_node", name="cartpole_logger",
             parameters=[{"output_file": LaunchConfiguration("log_file")}], output="screen"),
    ])
