"""Launch the ROB3 driver + robot_state_publisher.

Usage:
  # real robot
  ros2 launch rob3_driver rob3.launch.py device:=/dev/ttyUSB0
  # ucSim: point `device` at the pty ucSim is attached to (see docs/SIMULATION.md)
  ros2 launch rob3_driver rob3.launch.py device:=/dev/pts/7
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = "rob3_driver"

    args = [
        DeclareLaunchArgument("transport", default_value="serial",
                              description="link transport (only 'serial')"),
        DeclareLaunchArgument("device", default_value="/dev/ttyUSB0",
                      description="serial device (real robot or ucSim pty)"),
        DeclareLaunchArgument("baud", default_value="9600"),
        DeclareLaunchArgument("publish_rate", default_value="10.0"),
        DeclareLaunchArgument("joint_prefix", default_value=""),
        DeclareLaunchArgument("driver", default_value="true",
                      description="Start the ROB3 transport driver"),
        DeclareLaunchArgument("rviz", default_value="false",
                      description="Start RViz2"),
    ]

    urdf = Command([
        "xacro ",
        PathJoinSubstitution([FindPackageShare(pkg), "urdf", "rob3.urdf.xacro"]),
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": ParameterValue(urdf, value_type=str)}],
    )

    driver = Node(
        package=pkg,
        executable="rob3_driver_node",
        name="rob3_driver",
        output="screen",
        parameters=[{
            "transport": LaunchConfiguration("transport"),
            "device": LaunchConfiguration("device"),
            "baud": LaunchConfiguration("baud"),
            "publish_rate": LaunchConfiguration("publish_rate"),
            "joint_prefix": LaunchConfiguration("joint_prefix"),
        }],
        condition=IfCondition(LaunchConfiguration("driver")),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration("driver")),
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare(pkg), "config", "rob3.rviz"]
        )],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription(args + [
        robot_state_publisher, driver, joint_state_publisher, rviz,
    ])
