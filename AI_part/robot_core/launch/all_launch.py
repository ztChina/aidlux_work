from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('voice_interaction'), 'launch', 'voice_interaction_launch.py')
        )
    )
    pkg2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('yolov5_check'), 'launch', 'vision.launch.py')
        )
    )

    return LaunchDescription([
        pkg1_launch,
        pkg2_launch,
    #     Node(
    #     package='yolov5_check',
    #     executable='yolov5_check',
    #     name='yolov5_check'
    # ),
    #     Node(
    #     package='camera_start_python',
    #     executable='cam_pub',
    #     name='cam_pub'
    # ),
    #     Node(
    #     package='fake_drive',
    #     executable='fake_drive_node',
    #     name='fake_drive_node'
    # ),
    #     Node(
    #     package='fake_nav',
    #     executable='fake_nav',
    #     name='fake_nav'
    # ),
    #     Node(
    #     package='robot_core',
    #     executable='robot_core_node',
    #     name='robot_core_node'
    # ),

    ])
