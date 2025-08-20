from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    yolov5_check = Node(
        package='yolov5_check',
        executable='yolov5_check',
        name='yolov5_check',
        output='screen',
    )

    cam_pub = Node(
        package='camera_start_python',
        executable='cam_pub',
        name='cam_pub',
        output='screen',
    )
    return LaunchDescription([
        cam_pub,
        yolov5_check
    ])
