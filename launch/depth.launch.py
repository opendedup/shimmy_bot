# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node


def generate_launch_description():
    zed_sensors = ['zed', 'zed2', 'zed2i', 'zedm']
    zed_common_config_path = PathJoinSubstitution(
        [FindPackageShare('shimmy_bot'), 'config', 'zed_common.yaml']
    )

    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py']
            )),
            condition=IfCondition(PythonExpression(['"', LaunchConfiguration('sensor'), '" in "', str(zed_sensors), '"'])),
            launch_arguments={
                'camera_model': LaunchConfiguration('sensor'),
                'config_common_path': zed_common_config_path,
                'camera_name': '',
                'node_name': 'top',
                'publish_urdf': 'true',
                'base_frame': 'camera_link'
            }.items()   
        ),
    ])

