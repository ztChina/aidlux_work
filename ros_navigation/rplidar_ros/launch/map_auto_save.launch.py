from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    
    # 地图保存根路径（确保文件夹存在）
    map_root_path = '/home/aidlux/shouji7/src/rplidar_ros/map'
    
    # 生成带序号的唯一文件名（如map_1、map_2...）
    def get_numbered_filename():
        filename = f"{map_root_path}/map"  # 拼接序号
        return filename
    
    # 创建新的保存进程（带唯一序号文件名）
    def create_save_map_process():
        filename = get_numbered_filename()
        return ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', filename],
            output='screen',
            name=f"map_saver"  # 进程名带序号，避免冲突
        ), filename
    
    # 首次保存进程
    first_save_process, first_filename = create_save_map_process()
    
    # 首次定时保存
    timer = TimerAction(
        period=30.0,
        actions=[
            first_save_process,
            LogInfo(msg=f"已保存地图：{first_filename}")
        ]
    )
    
    # 循环保存逻辑
    def restart_timer(event, context):
        new_save_process, new_filename = create_save_map_process()
        return [
            TimerAction(
                period=30.0,
                actions=[
                    new_save_process,
                    LogInfo(msg=f"已保存地图：{new_filename}")
                ]
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=new_save_process,
                    on_exit=restart_timer
                )
            )
        ]
    
    # 首次保存后启动循环
    first_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=first_save_process,
            on_exit=restart_timer
        )
    )
    
    return LaunchDescription([
        timer,
        first_event_handler
    ])