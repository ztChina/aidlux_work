import re
import threading
import numpy as np
import librosa
import sounddevice as sd
from std_msgs.msg import String, Bool
import rclpy
from rclpy.node import Node
from voice_interaction.tts_cli.model import load_model
#根据情况导入内核器
# if __name__ == '__main__':
#     import sys
#     sys.path.append("/home/aidlux/my_ws/src/robot_core/robot_core")  # 添加 robot_core 的父目录
#     import robot_state  # 直接导入
#     from robot_state import my_robot_state 
# else:
#     from robot_core import robot_state
#     from robot_core.robot_state import my_robot_state    # 引入机器人状态实例


def replace_punctuation(s):
    data = re.sub(r'[^\w\s]', ' ', s)   # 将所有标点替换为空格
    return data.replace(" ", "")


class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        # self.my_robot_state = my_robot_state
        self.subscription = self.create_subscription(String, 'llm/reply', self.tts_callback, 10)
        self.prompt_sub = self.create_subscription(String, 'tts/prompt', self.tts_callback, 10)
        self.conv_sub = self.create_subscription(Bool, '/ai/conversation', self.conv_callback, 10)
        self.done_pub = self.create_publisher(Bool, 'tts/done', 10)
        self.tts_model = load_model()
        self.device_index = self.find_output_device_index()
        self.tts_done = True
        self.ai_start = False
        self.ai_conv = False
        if self.device_index is None:
            self.get_logger().error("没有找到有效的音频输出设备，TTS播放将无法进行。")
        else:
            self.get_logger().info(f"使用音频输出设备编号 {self.device_index} 进行播放。")
        self.get_logger().info("TTS节点启动，等待文本消息...")

    def find_output_device_index(self):
        devices = sd.query_devices()
        usb_candidates = []
        other_candidates = []

        for idx, dev in enumerate(devices):
            if dev['max_output_channels'] > 0:
                name = dev['name'].lower()
                if 'usb' in name:
                    usb_candidates.append((idx, dev))
                else:
                    other_candidates.append((idx, dev))

        if usb_candidates:
            return usb_candidates[0][0]  # 优先选择 USB 扬声器
        elif other_candidates:
            return other_candidates[0][0]  # 否则退而求其次
        else:
            return None

    def tts_callback(self, msg):
        text = msg.data
        if self.tts_done:
            self.tts_done = False
            if text and self.ai_conv:
                self.get_logger().info(f"收到回复文本，开始合成播放: {text}")
                # if "缺水" in text:
                #     self.get_logger().info("缺水语音播放中......")
                threading.Thread(target=self.play_speech, args=(text,)).start()
            else :
                self.done_pub.publish(Bool(data=True)) # 直接发布完成信息
                self.tts_done = True
    def conv_callback(self, msg: Bool):
        "回调显示会话状态"
        self.ai_conv = msg.data

    def play_speech(self, text):
        if self.device_index is None:
            self.get_logger().error("无有效音频输出设备，跳过播放。")
            return

        try:
            clean_text = replace_punctuation(text).strip()
            phones, audio, fronted_time, vits_time = self.tts_model.synthesis(clean_text)
            audio_float = audio.astype(np.float32) / 32767
            audio_48k = librosa.resample(audio_float, orig_sr=16000, target_sr=48000)
            audio_48k_int16 = (audio_48k * 32767).astype(np.int16)

            sd.play(audio_48k_int16, samplerate=48000, device=self.device_index)
            sd.wait()
            self.tts_done = True
            #更改标志位
            self.done_pub.publish(Bool(data=True))
            text = None
        except Exception as e:
            self.get_logger().error(f"TTS播放失败: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

