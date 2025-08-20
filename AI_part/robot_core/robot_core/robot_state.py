import mmap
import struct
import threading
import mmap
import struct
import rclpy
from std_srvs.srv import SetBool
import os
from rclpy.node import Node
from std_msgs.msg import Bool,String
import threading
import time
#####暂时不写

class bit_enum():
    """
    枚举出机器人状态int所有32位的代表状态

    State Bit Allocation (32-bit integer):
        Bit 0: 是否SLAM建完图，1为是
        Bit 1: 是否YOLO是否开启，1为是
        Bits 1-31: Reserved for future use.
    """
    SLAM_Over_bit   = 0
    Water_Loss_bit  = 1 #0为正常，1为缺水
    YOLO_Start_bit  = 2
    NAV_Start_bit   = 3
    AI_Start_bit    = 4 #先 start 才能 wake
    Close_to_Person_bit = 5 #这个close只表示寻人中找到的人的附近了。
    Serching_for_Person = 6 #找人中。没用
    Wait_for_Help_bit = 7


class MyRobotState:
    def __init__(self):
        self.state = 0  # 使用一个整数存储状态位

    def set_bit(self, bit_position: int, value: bool):
        """设置某一位为1或0"""
        if value:
            self.state |= (1 << bit_position)
        else:
            self.state &= ~(1 << bit_position)

    def get_bit(self, bit_position: int) -> bool:
        """获取某一位的布尔值"""
        return (self.state & (1 << bit_position)) != 0

    def toggle_bit(self, bit_position: int):
        """切换某一位的状态"""
        self.state ^= (1 << bit_position)

    def clear_all(self):
        """清空所有状态位"""
        self.state = 0

    def __repr__(self):
        return f"RobotState({bin(self.state)})"


class MyNode(Node):
    """ 
        事物优先级：
        导航（行走中）
        对话
        yolo检测
        缺水语音“缺水啦”
    """
    def __init__(self):
        super().__init__('robot_core_node')
        self.my_robot_state = MyRobotState()
        #状态发布
        self.state_publishers = {
            bit_enum.SLAM_Over_bit: self.create_publisher(Bool, '/robot/state/slam_over', 10),
            bit_enum.Water_Loss_bit: self.create_publisher(Bool, '/robot/state/water_loss', 10),
            bit_enum.YOLO_Start_bit: self.create_publisher(Bool, '/robot/state/yolo_start', 10),
            bit_enum.NAV_Start_bit: self.create_publisher(Bool, '/robot/state/nav_start', 10),
            bit_enum.AI_Start_bit: self.create_publisher(Bool, '/robot/state/ai_start', 10),
            bit_enum.Close_to_Person_bit: self.create_publisher(Bool, '/robot/state/close_to_person', 10),
            bit_enum.Wait_for_Help_bit: self.create_publisher(Bool, '/robot/state/wait_for_help', 10),
        }

        # 订阅缺水状态（来自 fake_drive）
        self.create_subscription(Bool, '/device/water_loss', self.water_callback, 10)
        # 订阅等待帮助状态（来自 ai）
        self.wait_control_sub = self.create_subscription(Bool, '/ai/wait_for_help', self.wait_for_help_control_callback, 10)
        # 订阅是否接近人（来自 fake_nav）
        self.create_subscription(Bool, '/close_to_person', self.close_to_person_callback, 10)

        # 订阅结束对话（来自 dialoguecoordinator）
        self.create_subscription(Bool, '/end_talk', self.end_talk_callback, 10)

        # 发布返回信号
        self.back_pub = self.create_publisher(String, '/back_to_start', 10)

        # yolo、nav、AI 控制服务客户端
        self.nav_client = self.create_client(SetBool, '/start_nav')
        self.yolo_client = self.create_client(SetBool, '/start_yolo')
        self.ai_client = self.create_client(SetBool, '/start_AI')

        #初始开启AI模块
        self.set_and_publish_bit(bit_enum.AI_Start_bit, True)
        self.call_service(self.ai_client, True, "AI启动")
        
        self.get_logger().info("ControlCoreNode 初始化完成")

    def water_callback(self, msg: Bool):
        water_loss = self.my_robot_state.get_bit(bit_enum.Water_Loss_bit)

        if msg.data and not water_loss:  # 从不缺水变成了缺水状态
            self.get_logger().info("缺水！关闭ai，开启YOLO,准备导航，开始找人")
            self.set_and_publish_bit(bit_enum.Water_Loss_bit, True)

            # if self.my_robot_state.get_bit(bit_enum.AI_Start_bit):
            self.set_and_publish_bit(bit_enum.AI_Start_bit, False)
            self.call_service(self.ai_client, False, "关闭AI")

            # 如果 NAV_Start_bit 尚未置为 True，则修改并调用服务
            # if not self.my_robot_state.get_bit(bit_enum.NAV_Start_bit):
            self.set_and_publish_bit(bit_enum.NAV_Start_bit, True)
            self.call_service(self.nav_client, True, "导航已启动")

            # 如果 YOLO_Start_bit 尚未置为 True，则修改并调用服务
            # if not self.my_robot_state.get_bit(bit_enum.YOLO_Start_bit):
            self.set_and_publish_bit(bit_enum.YOLO_Start_bit, True)
            self.call_service(self.yolo_client, True, "YOLO已启动")
        if not msg.data and water_loss: # 不再缺水,状态转换
            #  不缺水后开启ai节点。
            self.set_and_publish_bit(bit_enum.Water_Loss_bit, False)

            # if not self.my_robot_state.get_bit(bit_enum.AI_Start_bit):
            self.set_and_publish_bit(bit_enum.AI_Start_bit, True)
            self.call_service(self.ai_client, True, "AI启动")

                # 如果 NAV_Start_bit 尚未置为 True，则修改并调用服务
            # if  self.my_robot_state.get_bit(bit_enum.NAV_Start_bit):
            self.set_and_publish_bit(bit_enum.NAV_Start_bit, False)
            self.call_service(self.nav_client, False, "导航已关闭")

            # 如果 YOLO_Start_bit 尚未置为 True，则修改并调用服务
            # if  self.my_robot_state.get_bit(bit_enum.YOLO_Start_bit):
            self.set_and_publish_bit(bit_enum.YOLO_Start_bit, False)
            self.call_service(self.yolo_client, False, "YOLO已关闭")

    def wait_for_help_control_callback(self, msg: Bool):
        """中间控制回调：收到外部 wait_for_help 控制，来自ai"""
        self.set_and_publish_bit(bit_enum.Wait_for_Help_bit, True)

    def set_and_publish_bit(self, bit_enum_val: int, value: bool):
        """统一设置状态位并发布话题"""
        # 设置状态位
        self.my_robot_state.set_bit(bit_enum_val, value)

        # 发布对应的 Bool 消息
        if bit_enum_val in self.state_publishers:
            self.state_publishers[bit_enum_val].publish(Bool(data=value))
            self.get_logger().info(f"状态位 {bit_enum_val} 设置为 {value} 并已发布")
        else:
            self.get_logger().warn(f"状态位 {bit_enum_val} 无对应发布器，未发布话题")

    # ========== 寻人完成回调 ==========
    def close_to_person_callback(self, msg: Bool):
        if not msg.data:
            return

        self.get_logger().info("已接近人，关闭YOLO与导航，启动AI。")
        # if not self.my_robot_state.get_bit(bit_enum.Close_to_Person_bit):
        self.set_and_publish_bit(bit_enum.Close_to_Person_bit, True)

        # if  self.my_robot_state.get_bit(bit_enum.YOLO_Start_bit):
        self.set_and_publish_bit(bit_enum.YOLO_Start_bit, False)
        self.call_service(self.yolo_client, False, "YOLO已关闭")

        # if  self.my_robot_state.get_bit(bit_enum.NAV_Start_bit):
        self.set_and_publish_bit(bit_enum.NAV_Start_bit, False)
        self.call_service(self.nav_client, False, "导航已关闭")

        # if  not self.my_robot_state.get_bit(bit_enum.AI_Start_bit):
        self.set_and_publish_bit(bit_enum.AI_Start_bit, True)
        self.call_service(self.ai_client, True, "AI启动")
    
    # ========== 会话结束 ==========
    def end_talk_callback(self, msg: Bool):
        "只用于缺水状态下的状态判别"
        if not msg.data:
            return

        self.get_logger().info("对话结束，处理状态...")

        if self.my_robot_state.get_bit(bit_enum.Close_to_Person_bit): #这段逻辑只用于寻人中
            # self.my_robot_state.set_bit(bit_enum.Close_to_Person_bit, False)
            # 判断 Wait_for_Help_bit
            wait_for_help = self.my_robot_state.get_bit(bit_enum.Wait_for_Help_bit)
            if wait_for_help:
                self.get_logger().info("等待帮助中，暂停行动，等待水满...")

                def wait_for_refill():
                    while self.my_robot_state.get_bit(bit_enum.Water_Loss_bit):
                        self.get_logger().info("水仍未加满，继续等待...")
                        time.sleep(2.0)

                    self.get_logger().info("水已加满。")
                    # self.call_service(self.nav_client, True, "返回导航已启动")
                    # self.back_pub.publish(String(data="back"))
                threading.Thread(target=wait_for_refill, daemon=True).start()
                #线程阻塞等待中，结束后不再需要help,清除接近人
                self.my_robot_state.set_bit(bit_enum.Close_to_Person_bit, False)
                self.set_and_publish_bit(bit_enum.Wait_for_Help_bit, False)

            else:
                self.get_logger().info("无需等待，准备继续寻找人...")
                self.my_robot_state.set_bit(bit_enum.Close_to_Person_bit, False)

                def resume_search():
                    self.call_service(self.nav_client, True, "重新导航中")
                    self.call_service(self.ai_client, False, "AI关闭")
                    self.get_logger().info("延时1秒后启动YOLO")
                    time.sleep(1.0)
                    self.call_service(self.yolo_client, True, "YOLO已重启")


                threading.Thread(target=resume_search, daemon=True).start()
        # else:


    # ========== 服务调用统一封装 ==========
    def call_service(self, client, flag: bool, log_msg: str):
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"服务不可用: {client.srv_name}")
            return
        req = SetBool.Request()
        req.data = flag
        future = client.call_async(req)
        self.get_logger().info(log_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
