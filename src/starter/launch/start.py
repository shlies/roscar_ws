# coding=utf-8
'''
@file      start.py
@brief     主启动文件
@author    shlies (shlies@github.com)
@version   1.0.0
@date      2024-07-11

@copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.

@attention 

@par history
| Version | Date | Author | Description |
| :---: | :---: | :---: | :---: |
| 1.0.0 | 2024-MM-DD | shlies | description |
@par last editor  shlies (shlies@github.com)
@par last edit time  2024-07-11
'''
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    controller_launch_file = os.path.join(
        get_package_share_directory('controller'),
        'launch',
        'controller_launch.py'
    )
    return LaunchDescription([
        Node(
            package='camera',
            executable='publish',
            name='publish',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch_file)
        ),
        Node(
            package='stm32_comm',
            executable='uart_comm',
            name='uart_comm',
            output='screen',
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=Node(
                    package='stm32_comm',
                    executable='uart_comm',
                    name='uart_comm',
                    output='screen',
                ),
                on_exit=[Node(
                    package='stm32_comm',
                    executable='uart_comm',
                    name='uart_comm',
                    output='screen',
                )]
            )
        ),
    ])