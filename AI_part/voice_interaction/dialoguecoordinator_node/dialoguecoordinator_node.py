import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
import time


class DialogueCoordinator(Node):
    def __init__(self):
        """
            所有语音模块的控制节点
        """
        super().__init__('dialogue_coordinator')
        self.in_conversation = False    # 是否处于对话模式
        self.ai_started = False
        self.close_to_person = False
        self.water_loss = False
        self.empty_asr_count = 0  # 连续空识别计数
        self.timeout_timer = None   # 超时计时器
        self.wait_for_help = False
        # 初始化开启服务
        self.srv = self.create_service(SetBool, '/start_AI', self.handle_start_AI_request)
        # self.srv = self.create_service(SetBool, '/start_water_voice', self.handle_start_water_request)
        #订阅状态
        self.close_sub = self.create_subscription(Bool, '/robot/state/close_to_person', self.close_callback, 10)
        self.water_sub = self.create_subscription(Bool, '/robot/state/water_loss', self.water_callback, 10)
        #发布状态
        self.core_wait_pub = self.create_publisher(Bool, '/ai/wait_for_help', 10)
        #发布状态
        self.core_conv_pub = self.create_publisher(Bool, '/ai/conversation', 10)
        # 触发录音
        self.vad_trigger_pub = self.create_publisher(Bool, 'vad/start', 10)
        #关键词唤醒
        self.wake_reset_pub = self.create_publisher(Bool, 'wake_word/reset', 10)
        #关闭vad节点音频流
        self.vad_stop_pub = self.create_publisher(Bool, 'vad/stop', 10)
        #向大模型发布内容
        self.llm_pub = self.create_publisher(String, 'coordinator/result', 10)
        # 订阅唤醒、TTS完成、ASR结果
        self.wake_sub = self.create_subscription(Bool, 'wake_word/detected', self.on_wake, 10)
        self.tts_done_sub = self.create_subscription(Bool, 'tts/done', self.on_tts_done, 10)
        self.asr_result_sub = self.create_subscription(String, 'asr/result', self.on_asr_result, 10)

        self.tts_prompt_pub = self.create_publisher(String, 'tts/prompt', 10)
        self.end_talk_pub = self.create_publisher(Bool, '/end_talk', 10)
        # 订阅是否接近人（来自 fake_nav）
        # self.create_subscription(Bool, '/close_to_person', self.close_to_person_callback, 10)
        self.get_logger().info('AI Moudle service is ready and waiting for requests.')
        

    def handle_start_AI_request(self, request, response):
        """
            服务回调函数，用于判断是否启动下面的步骤
        """
        self.requested_state = "START" if request.data else "STOP"
        self.get_logger().info(f'Received request to {self.requested_state} AI Moudle.')
        response.success = True
        response.message = f'AI_node  {self.requested_state} successfully.'
        #3、判断启动还是停止
        if request.data:
           self.ai_started = True
           self.wake_reset_pub.publish(Bool(data=True))
        else:
            self.ai_started = False
        
        #服务端会自动返回response对象
        return response

    def close_callback(self, msg: Bool):
        "触发该回调表示必定接近人了"
        # if not self.ai_started: #此时ai还没有启动。标志位的接收不应该和模块的启用关联
        #     return
        self.get_logger().info(f'ai收到人在旁边了！{msg.data}')
        self.close_to_person = msg.data

    def water_callback(self, msg: Bool):
        "触发该回调表示缺水状态更新"
        # if not self.ai_started:
        #     return
        self.get_logger().info('ai收到缺水状态更新了！')
        self.water_loss = msg.data

    def on_wake(self, msg):
        """
            关键字检测成功之后，进入对话模式
        """
        if not self.ai_started:
            self.get_logger().info("ai未开启,唤醒跳过")
            return
        if msg.data and not self.in_conversation:
            self.in_conversation = True
            self.core_conv_pub.publish(Bool(data=True)) 
            self.empty_asr_count = 0
            # close_to_person = self.my_robot_state.get_bit(robot_state.bit_enum.Close_to_Person_bit)
            # water_loss = self.my_robot_state.get_bit(robot_state.bit_enum.Water_Loss_bit)
            # self.vad_stop_pub.publish(Bool(data=True)) 
            if  self.close_to_person and self.water_loss and not self.wait_for_help:
                # 两个状态都为 True，执行相应逻辑
                self.get_logger().info("执行缺水唤醒")
                self.tts_prompt_pub.publish(String(data="小笨蛋缺水啦，你可以帮我加水吗"))
                # time.sleep(5)
            else:
                self.get_logger().info("唤醒成功，进入对话模式。")
                self.tts_prompt_pub.publish(String(data="我在"))
            # self.my_robot_state.set_bit(robot_state.bit_enum.AI_Waked_bit,True)

    def on_tts_done(self, msg):
        if not self.ai_started:
            return
        if self.in_conversation: #这里不需要判断是否在对话中，只要结束就开，ai模块是开的就行
            self.get_logger().info("TTS 播放完成，准备开始录音。")
            self.vad_trigger_pub.publish(Bool(data=True))
            self.restart_timeout()
        else :
            self.wake_reset_pub.publish(Bool(data=True))

    def on_asr_result(self, msg):
        text = msg.data.strip()
        self.cancel_timeout()
        if not text:
            self.empty_asr_count += 1
            self.get_logger().info(f"空识别结果次数：{self.empty_asr_count}")
            # self.tts_prompt_pub.publish(String(data="哎，我在"))
            if self.empty_asr_count >= 2:  # 连续两次空识别，结束对话
                self.get_logger().info("连续空识别，结束对话模式。")
                self.end_conversation()
                return
            else:
                self.restart_timeout()
                self.vad_trigger_pub.publish(Bool(data=True)) #空文本必须在此额外加录音唤醒
                return
        else:
            self.empty_asr_count = 0

        # 可根据需求检测结束关键词
        end_keywords1 = ["再见"]
        end_keywords2 = ["好","等一下","马上","好啊","行","可以","等下"]
        self.vad_stop_pub.publish(Bool(data=True)) 
        self.get_logger().info(f"识别结果：{text}")
        if any(word in text for word in end_keywords1):
            self.get_logger().info("检测到结束关键词，结束对话模式。")
            self.end_conversation()
            return
        elif  self.close_to_person and self.water_loss:
            # 两个状态都为 True，执行相应逻辑
            if any(word in text for word in end_keywords2):
                self.end_conversation2()
                self.get_logger().info("检测到结束关键词，结束对话模式。")
            else :#这是在原地等待过程中回复
                self.get_logger().info("已发布到llm")
                self.llm_pub.publish(msg)

        else :#这是一般的情况发布到大模型
            self.get_logger().info("已发布到llm")
            self.llm_pub.publish(msg)

        self.restart_timeout()

    def restart_timeout(self):
        """ 重置超时计时器 """
        self.cancel_timeout()
        # ROS定时器替代threading.Timer，更安全
        self.timeout_timer = self.create_timer(15.0, self.end_due_to_timeout)
        # self.vad_trigger_pub.publish(Bool(data=True))
        # self.wake_reset_pub.publish(Bool(data=True))
        # self.end_talk_pub.publish(Bool(data=True))

    def cancel_timeout(self):
        """ 取消超时计时器 """
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
            self.timeout_timer = None

    def end_due_to_timeout(self):
        if self.in_conversation:
            self.get_logger().info("10秒无用户输入，自动结束对话。")
            self.end_conversation()

    def end_conversation(self):
        """
            结束对话模式
        """
        if self.in_conversation:
            self.vad_stop_pub.publish(Bool(data=True))
            self.tts_prompt_pub.publish(String(data="再见"))
            self.in_conversation = False
            self.core_conv_pub.publish(Bool(data=False)) 
            self.empty_asr_count = 0
            self.cancel_timeout()

            # 发布 reset 信号，恢复唤醒检测器
            # if not self.close_to_person:#在原地的时候才重置
            self.get_logger().info("唤醒词重置")
            self.wake_reset_pub.publish(Bool(data=True))
            self.end_talk_pub.publish(Bool(data=True))
        else :#用于异常的恢复
            self.get_logger().info("唤醒词异常重置")
            self.wake_reset_pub.publish(Bool(data=True))

    def end_conversation2(self):
        """
            结束对话模式，获得帮助了
        """
        if self.in_conversation:
            self.vad_stop_pub.publish(Bool(data=True))
            self.tts_prompt_pub.publish(String(data="好呀，谢谢你，我就在这里等你"))
            
            self.in_conversation = False
            self.core_conv_pub.publish(Bool(data=False)) 
            self.empty_asr_count = 0
            self.cancel_timeout()
            # 发布 reset 信号，恢复唤醒检测器
            self.get_logger().info("唤醒词重置.结束对话")
            self.wake_reset_pub.publish(Bool(data=True))
            self.core_wait_pub.publish(Bool(data=True))  # 进入等待帮助状态
            self.end_talk_pub.publish(Bool(data=True))
            self.wait_for_help = True
            # self.my_robot_state.set_bit(robot_state.bit_enum.AI_Waked_bit,False)


def main(args=None):
    rclpy.init(args=args)
    node = DialogueCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
