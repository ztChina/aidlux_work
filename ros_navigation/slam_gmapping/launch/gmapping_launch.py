#!/usr/bin/env python3

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='False')
    
    yaml_path = os.path.join(
        get_package_share_directory("slam_gmapping"),
        "params",
        "slam_gmapping.yaml"
    )

    return LaunchDescription([

        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            parameters=[{
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'laser_frame': 'laser',
                'use_sim_time': use_sim_time,
                },
                yaml_path
                ],
            
            output='screen',
            
        ),
    ])
