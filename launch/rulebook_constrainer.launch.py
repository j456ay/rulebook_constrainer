#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 패키지 경로
    pkg_share = FindPackageShare('rulebook_constrainer')
    
    # Launch arguments
    rulebook_yaml_arg = DeclareLaunchArgument(
        'rulebook_yaml',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'example_rulebook.yaml']),
        description='Path to rulebook YAML file'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'rulebook_params.yaml']),
        description='Path to parameters file'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the node'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Node configuration
    rulebook_constrainer_node = Node(
        package='rulebook_constrainer',
        executable='rulebook_constrainer_node',
        name='rulebook_constrainer_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'rulebook_yaml': LaunchConfiguration('rulebook_yaml'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        output='screen',
        emulate_tty=True,
        remappings=[
            # 필요시 토픽 리맵핑 추가
        ]
    )
    
    return LaunchDescription([
        rulebook_yaml_arg,
        params_file_arg,
        namespace_arg,
        use_sim_time_arg,
        rulebook_constrainer_node
    ])
