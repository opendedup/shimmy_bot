# Copyright 2020 ros2_control Development Team
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,OpaqueFunction,SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PythonExpression, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import os
import yaml

camera_name = 'zed'
camera_model = 'zed2'

def launch_setup(context, *args, **kwargs):
    # Declare arguments
    declared_arguments = []

    # Get URDF via xacro
    
    # sensors_launch_path = PathJoinSubstitution(
    #     [FindPackageShare('shimmy_bot'), 'launch', 'sensors.launch.py']
    # )
    use_zed_localization = LaunchConfiguration('use_zed_localization')
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("shimmy_bot"),"description", "robot.urdf.xacro"]
            ),
            " ",
            'camera_name:=', camera_name, ' ',
            'camera_model:=', camera_model, ' ',
            'use_zed_localization:=', use_zed_localization,
#            " ",
#            "use_mock_hardware:=",
#            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    pkg_share = FindPackageShare(package='shimmy_bot').find('shimmy_bot')

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("shimmy_bot"),
            "config",
            "shimmy.yaml",
        ]
    )
    zed_controllers = PathJoinSubstitution(
        [
            FindPackageShare("shimmy_bot"),
            "config",
            "zed_common.yaml",
        ]
    )
    
    rtabmap_params = get_package_share_directory('shimmy_bot') + '/config/rtabmap.yaml'
    print("##########################################################")
    with open(rtabmap_params, 'r') as file:
        rtabmap_config = yaml.load(file, Loader=yaml.BaseLoader)
    rtabmap_launch_arguments={
                'rtabmap_args': "--delete_db_on_start",
                'rgb_topic': "/zed/zed_node/rgb/image_rect_color",
                'depth_topic':'/zed/zed_node/depth/depth_registered',
                'camera_info_topic':'/zed/zed_node/rgb/camera_info',
                'frame_id':'zed_camera_link',
                'approx_sync':'true',
                'wait_imu_to_init':'true',
                'imu_topic':'/zed/zed_node/imu/data',
                'qos': '1',
                'odom_topic':'/zed/zed_node/odom',
                'publish_tf_odom':'false',
                'namespace':'/',
                'queue_size':'30',
                #'visual_odometry':'true'
            }
    rtabmap_config.update(rtabmap_launch_arguments)
    print(rtabmap_config)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            robot_controllers
        ],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/shimmy_bot/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    
    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        output="both",
        remappings=[
            ("/cmd_vel_in", "/cmd_vel"),
            ("/cmd_vel_out", "/shimmy_bot/cmd_vel"),
        ],
    )
    
    shimmy_move = Node(
        package="shimmy_move",
        executable="shimmy_move",
        output="both",
    )
    
    
    
    zed_wrapper_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py']
            )),
            launch_arguments={
                'camera_name': camera_name,
                'camera_model': camera_model,
                #'ros_params_override_path': zed_controllers
            }.items()
        )
    
    rtabmap_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py']
            )),
            launch_arguments=rtabmap_config.items()
        )
    
    shimmy_talk_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('shimmy_talk'), 'launch', 'shimmy_talk.launch.py']
            )),
            launch_arguments={
                'camera_name': camera_name,
                'camera_model': camera_model,
                'ros_params_override_path': zed_controllers
            }.items()
        )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["shimmy_bot", "--controller-manager", "/controller_manager"],
    )
    
    fgnode=Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
        )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    
    
    

    return [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        zed_wrapper_launch,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        fgnode,
        twist_stamper,
        rtabmap_launch,
        shimmy_move,
        #shimmy_talk_launch
    ]
    
    
def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
            DeclareLaunchArgument(
                'use_zed_localization',
                default_value='true',
                description='Creates a TF tree with `camera_link` as root frame if `true`, otherwise the root is `base_link`.',
                choices=['true', 'false']),
            OpaqueFunction(function=launch_setup)    
        ]
    )
    #return LaunchDescription(declared_arguments + nodes + launch_descriptions)