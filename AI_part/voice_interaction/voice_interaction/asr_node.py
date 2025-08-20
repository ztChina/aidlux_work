import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from ament_index_python.packages import get_package_share_directory
import numpy as np
import struct
import tempfile
if __name__ == '__main__':
    import sys
    sys.path.append("/home/aidlux/my_ws/src/voice_interaction/voice_interaction")  # 添加父目录
    from wenet import WeNetDecoder # 直接导入
else:
    from .wenet import WeNetDecoder
import os
import yaml
import wave

class ASRNode(Node):
    def __init__(self):
        super().__init__('asr_node')
        self.publisher_ = self.create_publisher(String, 'asr/result', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'audio/raw_wav',
            self.audio_callback,
            10
        )

        # 获取语音转文字模型路径并解析 config
        resource_dir = get_package_share_directory('voice_interaction')
        model_yaml_path = os.path.join(resource_dir, 'models/model_onnx.yaml')
        with open(model_yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        # 替换相对路径为绝对路径
        if 'dict' in config and not os.path.isabs(config['dict']):
            config['dict'] = os.path.join(resource_dir, config['dict'])
        if 'onnx_dir' in config and not os.path.isabs(config['onnx_dir']):
            config['onnx_dir'] = os.path.join(resource_dir, config['onnx_dir'])

        # 写入临时 config 文件
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as tmp:
            yaml.dump(config, tmp)
            self.tmp_path = tmp.name
        #提前加载模型
        self.get_logger().info("🧠 加载ASR模型...")
        self.weNetDecoder = WeNetDecoder(self.tmp_path)

        self.get_logger().info("✅ ASR节点已启动，等待音频输入...")

    def audio_callback(self, msg: Float32MultiArray):
        self.get_logger().info("📥 收到音频数据，开始ASR识别...")
        # self.get_logger().info(f"音频长度: {len(msg.data)}, 前5个数据: {msg.data[:5]}")

        tmp_wav_path = None
        try:
            # 数据转换：float32 → int16
            float32_data = np.array(msg.data, dtype=np.float32)
            int16_data = np.clip(float32_data, -32768, 32767).astype(np.int16)
            self.get_logger().info(f"音频最大最小值: {float32_data.min()} ~ {float32_data.max()}")
            

            # 写入临时 WAV 文件
            with tempfile.NamedTemporaryFile(delete=False, suffix='.wav') as tmp_wav:
                tmp_wav_path = tmp_wav.name
                with wave.open(tmp_wav_path, 'wb') as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(16000)
                    wf.writeframes(int16_data.tobytes())

            # 保存一份调试音频
            # debug_path = "/home/aidlux/aidcode/asr_saved_audio.wav"
            # with wave.open(debug_path, 'wb') as wf:
            #     wf.setnchannels(1)
            #     wf.setsampwidth(2)
            #     wf.setframerate(16000)
            #     wf.writeframes(int16_data.tobytes())
            # self.get_logger().info(f"📝 调试音频已保存到: {debug_path}")

            # 执行 ASR
            text = self.perform_asr(tmp_wav_path)
            msg_out = String()
            msg_out.data = text
            self.publisher_.publish(msg_out)
            self.get_logger().info(f"发布识别结果: {text}")

        except Exception as e:
            self.get_logger().error(f"音频处理失败: {str(e)}")

        finally:
            if tmp_wav_path and os.path.exists(tmp_wav_path):
                os.remove(tmp_wav_path)

    def perform_asr(self, input_wav_path):
        """
            语音转文字
        """
        try:
            with open(input_wav_path, 'rb') as audiostream:
                audiostream.read(44)  # 跳过 WAV 头
                self.get_logger().info("🎙️ 开始语音识别...")

                while True:
                    dataflow = audiostream.read(8000)
                    if len(dataflow) == 0:
                        break
                    sig = struct.unpack("%ih" % (len(dataflow) // 2), dataflow)
                    data = np.array(sig, dtype=np.float32)
                    # self.get_logger().info(f"ASR 输入帧前5值：{data[:5]}")
                    # 开始识别
                    self.weNetDecoder.detect(data)
                    _ = self.weNetDecoder.ctc_prefix_beam_search_purn_try()

                #asr_text, _ = self.weNetDecoder.decoder_rescoring()
                # self.weNetDecoder.resetAll()
                # return asr_text
            
                result = self.weNetDecoder.decoder_rescoring()
                if result is None:
                    self.get_logger().error("decoder_rescoring 返回 None，无法解包")
                    self.weNetDecoder.resetAll()
                    return ""
                asr_text, _ = result
                self.weNetDecoder.resetAll()
                return asr_text

        except Exception as e:
            self.get_logger().error(f"ASR识别失败: {str(e)}")
            return ""

def main(args=None):
    rclpy.init(args=args)
    node = ASRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()