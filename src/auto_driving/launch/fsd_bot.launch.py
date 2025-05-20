#!/usr/bin/env python3
#
# Copyright 2020 ROBOTIS CO., LTD.
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
#
# Authors: Hye-jong KIM

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()
    launch_dir0 = os.path.join(
        get_package_share_directory(
            'turtlebot3_manipulation_bringup'), 'launch')
    launch_dir1 = os.path.join(
        get_package_share_directory(
            'aruco_yolo'), 'launch')
    launch_dir2 = os.path.join(
        get_package_share_directory(
            'turtlebot3_manipulation_moveit_config'), 'launch')


    # turtlebot hardware
    th_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir0, '/hardware.launch.py'])
    )
    ld.add_action(th_launch)

    # aruco
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir1, '/aruco_yolo_one.launch.py'])
    )
    ld.add_action(aruco_launch)

    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir2, '/moveit_rviz.launch.py'])
    )
    ld.add_action(rviz_launch)

    # move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir2, '/move_group.launch.py'])
    )
    ld.add_action(move_group_launch)

    arm_control_node = Node(
        package="turtlebot_moveit",
        executable="turtlebot_arm_controller"
    )
    ld.add_action(arm_control_node)

    return ld
