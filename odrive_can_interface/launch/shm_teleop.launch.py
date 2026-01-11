# Copyright 2020 ROS2-Control Development Team (2020)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    teleop_rate = DeclareLaunchArgument(
        "teleop_rate",
        default_value="200",
        description="Teleop update rate in Hz",
    )
    teleop_terminal = DeclareLaunchArgument(
        "teleop_terminal",
        default_value="xterm",
        description="Terminal executable to run teleop (e.g. xterm, gnome-terminal)",
    )

    package_description = "odrive_can_interface"
    package_directory = get_package_share_directory(package_description)

    urdf_file = "swerve_base_description.urdf.xacro"
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)

    if not os.path.exists(robot_desc_path):
        raise FileNotFoundError(f"URDF file not found: {robot_desc_path}")

    controller_config = os.path.join(
        get_package_share_directory("swerve_controller"),
        "config",
        "swerve_controller.yaml",
    )
    robot_description_content = Command([
        "xacro ",
        robot_desc_path,
        " controller_config:=",
        controller_config,
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controller_config],
        output={"stderr": "screen"},
        arguments=["--ros-args", "--log-level", "INFO"],
    )
    delayed_controller_manager = TimerAction(period=1.0, actions=[controller_manager])

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--ros-args", "--log-level", "INFO"],
        output="screen",
    )
    delayed_joint_broad_spawner = TimerAction(period=1.5, actions=[joint_broad_spawner])

    hsh_node = Node(
        package="hardware_system_handler",
        executable="hsh_node",
        name="hsh_node",
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
    )
    delayed_hsh_node = TimerAction(period=1.5, actions=[hsh_node])

    teleop_cmd = PythonExpression([
        "'ros2 run hardware_system_handler shm_key_teleop ' + '",
        LaunchConfiguration("teleop_rate"),
        "'"
    ])
    teleop = ExecuteProcess(
        cmd=[
            LaunchConfiguration("teleop_terminal"),
            "-e",
            "bash",
            "-lc",
            teleop_cmd,
        ],
        output="screen",
    )
    delayed_teleop = TimerAction(period=2.0, actions=[teleop])

    return LaunchDescription([
        teleop_rate,
        teleop_terminal,
        rsp,
        delayed_controller_manager,
        delayed_joint_broad_spawner,
        delayed_hsh_node,
        delayed_teleop,
    ])
