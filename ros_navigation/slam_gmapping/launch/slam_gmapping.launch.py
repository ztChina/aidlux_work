from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
import launch.actions
import launch_ros.actions
from launch_ros.actions import Node
import os
import launch
from ament_index_python.packages import get_package_share_directory

from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command


def generate_launch_description():

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
    )
    

    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_path = os.path.join(get_package_share_directory("slam_gmapping"), 'urdf', 'robot.urdf.xacro')

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("slam_gmapping"),
            "urdf",
            "robot.urdf.xacro"
        ),
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim_time}  # 使用声明的参数
        ],

        output="screen",
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],  # 使用声明的参数
        arguments=[urdf_path],
        output="screen",
    )


    return LaunchDescription([
        declare_use_sim_time_cmd,
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,

        # Node(
        #     package='slam_gmapping',
        #     namespace='transform',
        #     executable='transform',
        #     parameters=[{
        #         'parents_frame': 'odom',
        #         'child_frame': 'base_link',
        #                 }],
        #     output='screen',
        # ),


        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     output='screen',
        #     parameters=[{"use_sim_time": use_sim_time}]  # 使用声明的参数
        # )
    ])
