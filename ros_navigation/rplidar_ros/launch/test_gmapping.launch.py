from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch

def generate_launch_description():
    # ÉùÃ÷¼¤¹âÀ×´ïÀàÐÍ²ÎÊý
    lidar_type_arg = DeclareLaunchArgument(
        name='lidar_type',
        default_value='a1',
        description='Ö¸¶¨¼¤¹âÀ×´ïÀàÐÍ'
    )

    # Æô¶¯¼¤¹âÀ×´ï½Úµã
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'lidar.launch.py'
            )
        ]),
        launch_arguments={'lidar_type': LaunchConfiguration('lidar_type')}.items()
    )
    
    # Æô¶¯gmappingÏà¹Ø½Úµã
    gmapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam_gmapping'),
                'launch',
                'gmapping_launch.py'
            )
        ]),
        launch_arguments={'lidar_type': LaunchConfiguration('lidar_type')}.items()
    )
    
    slam_gmapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam_gmapping'),
                'launch',
                'slam_gmapping.launch.py'
            )
        ]),
        launch_arguments={
            'lidar_type': LaunchConfiguration('lidar_type')}.items()
    )
    
    # µØÍ¼±£´æÏà¹ØÅäÖÃ
    map_save_pkg = get_package_share_directory('rplidar_ros')
    map_save_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            map_save_pkg,
            '/launch',
            '/map_auto_save.launch.py'
        ])
    )

    # Æô¶¯RViz2½Úµã
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # Èç¹ûÓÐ×Ô¶¨ÒåµÄRVizÅäÖÃÎÄ¼þ£¬¿ÉÒÔÌí¼ÓÏÂÃæÕâÐÐÖ¸¶¨
        # arguments=['-d', os.path.join(get_package_share_directory('ÄãµÄ°üÃû'), 'config', 'slam_config.rviz')],
    )

    # ×é×°²¢·µ»ØÆô¶¯ÃèÊö
    return LaunchDescription([
        lidar_type_arg,
        lidar_launch,
        gmapping_launch,
        slam_gmapping_launch,
        # map_save_launch,
        # rviz_node  # Ìí¼ÓRViz½Úµã
    ])
