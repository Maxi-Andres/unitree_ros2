#!/usr/bin/env python3
"""
go2_full_visualization.launch.py

One command to visualize the real Unitree Go2 in RViz. It starts:

  * robot_state_publisher  — loads the go2_description URDF (with mesh paths already
    resolved to file://) and turns /joint_states into the TF tree of the robot body.
  * the LowState -> JointState bridge (go2_lowstate_to_joint_states_bridge.py), which
    also publishes the odom -> base transform from the lidar odometry.
  * rviz2 with a preloaded config that shows the robot model + lidar + odometry + TF.

Prerequisites: run inside the devcontainer with the Go2 connected, after
`source /workspace/setup.sh` (so ROS 2, cyclonedds and the unitree_go messages are on
the environment and the robot's topics are visible).

Usage:
    ros2 launch /workspace/go2_visualization/launch/go2_full_visualization.launch.py
Options:
    launch_rviz:=false                 # start everything except RViz
    rviz_config:=<absolute .rviz path> # use a different RViz layout
    publish_base_transform_from_odometry:=false  # show the model in place, no world pose
"""
import os

from ament_index_python.packages import get_package_share_directory  # noqa: F401
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# This launch file lives in <base>/launch/, so <base> is its parent's parent.
THIS_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
GO2_VISUALIZATION_BASE_DIRECTORY = os.path.dirname(THIS_DIRECTORY)

RESOLVED_URDF_PATH = os.path.join(
    GO2_VISUALIZATION_BASE_DIRECTORY,
    "models", "go2_description", "urdf", "go2_description_resolved_file_paths.urdf")
LOWSTATE_TO_JOINT_STATES_BRIDGE_SCRIPT = os.path.join(
    GO2_VISUALIZATION_BASE_DIRECTORY,
    "scripts", "go2_lowstate_to_joint_states_bridge.py")
DEFAULT_RVIZ_CONFIG_PATH = os.path.join(
    GO2_VISUALIZATION_BASE_DIRECTORY,
    "rviz", "go2_full_robot_model_and_sensors.rviz")


def generate_launch_description() -> LaunchDescription:
    with open(RESOLVED_URDF_PATH, "r") as urdf_file:
        robot_description_urdf_xml = urdf_file.read()

    launch_rviz_argument = DeclareLaunchArgument(
        "launch_rviz", default_value="true",
        description="Whether to also open RViz with the preloaded config.")
    rviz_config_argument = DeclareLaunchArgument(
        "rviz_config", default_value=DEFAULT_RVIZ_CONFIG_PATH,
        description="Absolute path to the RViz .rviz config to load.")
    publish_base_tf_argument = DeclareLaunchArgument(
        "publish_base_transform_from_odometry", default_value="true",
        description="Publish odom->base from lidar odometry so the model gets a world "
                    "pose. Set false to just show the model articulating in place.")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="go2_robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_urdf_xml,
            "use_sim_time": False,
        }],
    )

    lowstate_to_joint_states_bridge_process = ExecuteProcess(
        cmd=["python3", LOWSTATE_TO_JOINT_STATES_BRIDGE_SCRIPT,
             "--ros-args",
             "-p", ["publish_base_transform_from_odometry:=",
                    LaunchConfiguration("publish_base_transform_from_odometry")]],
        name="go2_lowstate_to_joint_states_bridge",
        output="screen",
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="go2_rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription([
        launch_rviz_argument,
        rviz_config_argument,
        publish_base_tf_argument,
        robot_state_publisher_node,
        lowstate_to_joint_states_bridge_process,
        rviz2_node,
    ])
