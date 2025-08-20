from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch

def generate_launch_description():
    # 1. 声明激光雷达类型参数
    lidar_type_arg = DeclareLaunchArgument(
        name='lidar_type',
        default_value='a1',
        description='激光雷达类型'
    )



    # 3. 原有激光雷达和SLAM启动配置
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
    
    # Æô¶¯RViz2½Úµã
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # Èç¹ûÓÐ×Ô¶¨ÒåµÄRVizÅäÖÃÎÄ¼þ£¬¿ÉÒÔÌí¼ÓÏÂÃæÕâÐÐÖ¸¶¨
        # arguments=['-d', os.path.join(get_package_share_directory('ÄãµÄ°üÃû'), 'config', 'slam_config.rviz')],
    )
 

    # 5. 组合所有组件（按依赖顺序）
    return LaunchDescription([
        lidar_type_arg,
        # 基础节点先启动
        lidar_launch,
        # SLAM相关节点后启动
        slam_gmapping_launch,
        # rviz_node
    ])    