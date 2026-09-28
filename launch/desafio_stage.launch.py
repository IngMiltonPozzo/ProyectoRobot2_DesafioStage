#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    challenge_share = get_package_share_directory('stage_challenge')

    world_file = os.path.join(challenge_share, 'worlds', 'desafio_stage.world')
    rviz_config = os.path.join(challenge_share, 'config', 'rviz', 'desafio_stage.rviz')

    stage_node = Node(
        package='stage_ros2',
        executable='stage_ros2',
        name='stage',
        arguments=['--ros-args', '--log-level', 'stage:=error'],
        parameters=[{
            'world_file': world_file,
            'use_stamped_velocity': False,
            'use_ackermann': False,
            'enforce_prefixes': False,
            'use_static_transformations': True,
            'one_tf_tree': False,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    map_markers_node = Node(
        package='stage_challenge',
        executable='map_markers',
        name='map_markers',
    )

    return LaunchDescription([
        stage_node,
        rviz_node,
        map_markers_node,
    ])
