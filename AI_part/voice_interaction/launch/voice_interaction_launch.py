from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    dialoguecoordinator_node = Node(
        package='voice_interaction',
        executable='dialoguecoordinator_node',
        name='dialoguecoordinator_node',
        output='screen',
    )

    vadrecorder_node = Node(
        package='voice_interaction',
        executable='vadrecorder_node',
        name='vadrecorder_node',
        output='screen'
    )

    asr_node = Node(
        package='voice_interaction',
        executable='asr_node',
        name='asr_node',
        output='screen'
    )

    llm_node = Node(
        package='voice_interaction',
        executable='llm_node',
        name='llm_node',
        output='screen'
    )

    tts_node = Node(
        package='voice_interaction',
        executable='tts_node',
        name='tts_node',
        output='screen'
    )

    return LaunchDescription([
        vadrecorder_node,
        dialoguecoordinator_node,
        asr_node,
        llm_node,
        tts_node
    ])
