import os
import launch
import launch_ros
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # »ñÈ¡°üÂ·¾¶
    fishbot_navigation2_dir = get_package_share_directory('daohang')
    urdf_model = get_package_share_directory('slam_gmapping')
    lidar_a1 = get_package_share_directory('rplidar_ros')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # ÅäÖÃ²ÎÊý - Ê¹ÓÃ¾ø¶ÔÂ·¾¶Ö¸¶¨µØÍ¼
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='false')
    # ÐÞ¸ÄÎª¾ø¶ÔÂ·¾¶
    # map_yaml_path = launch.substitutions.LaunchConfiguration(
    #     'map', default='/home/aidlux/shouji7/src/rplidar_ros/map/map.yaml')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(lidar_a1, 'map', 'map.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))
    


    return launch.LaunchDescription([
        # ÉùÃ÷Æô¶¯²ÎÊý
        launch.actions.DeclareLaunchArgument('use_sim_time', 
                                             default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', 
                                             default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', 
                                             default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        # Æô¶¯µ¼º½2
        # launch.actions.IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
        #     launch_arguments={
        #         'map': map_yaml_path,
        #         'use_sim_time': use_sim_time,
        #         'params_file': nav2_param_path
        #     }.items(),
        # ),
        # ÔÚ nav.launch.py ÖÐÐÞ¸Ä Nav2 Æô¶¯²¿·Ö£¬Ìí¼ÓÑÓ³Ù
        launch.actions.TimerAction(
            period=3.0,  # ÑÓ³Ù3Ãë£¬È·±£µØÍ¼¼ÓÔØÍê³É
            actions=[
                launch.actions.IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
                    launch_arguments={
                        'map': map_yaml_path,
                        'use_sim_time': use_sim_time,
                        'params_file': nav2_param_path
                    }.items(),
                )
            ]
        )
        
        # Æô¶¯URDF
        # urdf_launch,

        # # ÑÓ³ÙÆô¶¯À×´ï
        # lidar_delay,
    ])
