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
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    # Package configuration
    package_description = "odrive_can_interface"
    package_directory = get_package_share_directory(package_description)
    rviz_path= os.path.join(get_package_share_directory('swerve_base_description'),'rviz','display3.rviz')

    # URDF CONFIGURATION 
    urdf_file = "swerve_base_description.urdf.xacro"
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)

    if not os.path.exists(robot_desc_path):
        raise FileNotFoundError(f"URDF file not found: {robot_desc_path}")

    print(f"Loading URDF file from: {robot_desc_path}")
    
    # Get controller config path
    controller_config = os.path.join(
        get_package_share_directory('swerve_controller'),
        'config',
        'swerve_controller.yaml'
    )
    robot_description_content = Command([
        'xacro ', robot_desc_path, 
        ' controller_config:=', controller_config
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)
    
    rsp = Node(            
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output="screen",
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},controller_config],
        output={
        #   'stdout': 'screen',
          'stderr': 'screen',
          },
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path])
    
    delayed_rviz = TimerAction(period=3.0, actions=[rviz2])
    
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_state_broadcaster", '--ros-args', '--log-level', 'INFO'],
        output="screen",)
    
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    swerve_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["swerve_controller", "--controller-manager", "/controller_manager", '--ros-args', '--log-level', 'FATAL'],
        # output="screen",
    )
    delayed_swerve_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[swerve_controller_spawner],
        )
    )
    return LaunchDescription([
    rsp,
    delayed_controller_manager,
    delayed_swerve_controller_spawner,
    delayed_joint_broad_spawner,
    delayed_rviz
])