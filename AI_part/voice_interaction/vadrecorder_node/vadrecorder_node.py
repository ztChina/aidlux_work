
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
import numpy as np
import sounddevice as sd
import webrtcvad
import threading
import queue
import time
import librosa
import noisereduce as nr
import os
from ament_index_python.packages import get_package_share_directory
import sherpa_onnx
from unidecode import unidecode
from Levenshtein import distance
from scipy.signal import resample_poly\

from scipy.io.wavfile import write as wav_write
import datetime

class AudioStreamManager(threading.Thread):
    """
        音频流管理器
    """
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.cmd_queue = queue.Queue()  # 队列用于存储命令
        self.stop_event = threading.Event() # 线程状态
        self.current_stream = None      #用在vad中
        self.stream_lock = threading.Lock() #进入临界区时加锁
        self.running = True #是否要启动线程

        # 状态
        self.mode = None  # 'wake' 或 'vad' 或 None，表示唤醒模式、语音模式，或者 None 表示未定义
        self.recognizer = None  # 关键字识别器
        self.stream_obj = None  # 关键字检测时的音频流对象

        # 唤醒相关
        # self.target_seq = ["xiao", "ai", "tong", "xue"]
        # self.all_pinyins = set(["a", "ai", "xiao", "xue", "de", "tong", "shi", "xue"])
        self.target_seq = ["xiao", "ben", "dan"]
        self.all_pinyins = set(["xiao", "ben", "dan", "fen", "de", "da", "pan"])
        package_dir = get_package_share_directory('voice_interaction')
        self.model_dir = os.path.join(package_dir, 'wake_model')
        self.recognizer = self.create_recognizer()

        # VAD相关
        self.vad = webrtcvad.Vad(2) # 创建VAD实例，用于检测是否有人声，2为灵敏度设置
        self.fs = 48000
        self.frame_duration = 30
        self.chunk_size = int(self.fs * self.frame_duration / 1000)
        self.max_silence_frames = int(1.5 * 1000 / self.frame_duration)
        self.tail_padding_frames = 15

    def create_recognizer(self):
        """
            创建语音关键字识别器
        """
        return sherpa_onnx.OnlineRecognizer.from_transducer(
            encoder=os.path.join(self.model_dir, 'encoder-epoch-12-avg-2-chunk-16-left-64.onnx'),
            decoder=os.path.join(self.model_dir, 'decoder-epoch-12-avg-2-chunk-16-left-64.onnx'),
            joiner=os.path.join(self.model_dir, 'joiner-epoch-12-avg-2-chunk-16-left-64.onnx'),
            tokens=os.path.join(self.model_dir, 'tokens.txt'),
            num_threads=1,
            sample_rate=16000,
            feature_dim=80,
            enable_endpoint_detection=True,
            rule1_min_trailing_silence=1.2,
            rule2_min_trailing_silence=0.6,
            rule3_min_utterance_length=50,
        )

    # def find_input_device(self):
    #     """
    #         查找输入设备，但存在bug，摄像头也有usb麦设备
    #     """
    #     devices = sd.query_devices()
    #     usb_candidates = [(i, d) for i, d in enumerate(devices) if d['max_input_channels'] > 0 and 'usb' in d['name'].lower()]
    #     fallback_candidates = [(i, d) for i, d in enumerate(devices) if d['max_input_channels'] > 0]
    #     return (usb_candidates or fallback_candidates or [(None, None)])[0][0]
    
    def find_input_device(self):
        """
        查找输入设备
        """
        devices = sd.query_devices()
        # 查找 USB 麦克风
        usb_candidates = [(i, d) for i, d in enumerate(devices) if d['max_input_channels'] > 0 and (('ab13x' in d['name'].lower()) or ('y11' in d['name'].lower()))]
        # 兜底：找任何能输入的设备
        fallback_candidates = [(i, d) for i, d in enumerate(devices) if d['max_input_channels'] > 0]
        # 最终选择的设备 (索引, 设备对象)
        selected_device = (usb_candidates or fallback_candidates or [(None, None)])[0]
        index, device = selected_device
        # 输出设备信息
        if device:
            self.node.get_logger().info(f"输入设备名称: {device['name']}")
        else:
            self.node.get_logger().warn("未找到输入设备")
        
        return index


    def split_pinyin(self, text):
        text = unidecode(text).replace(" ", "")
        result = []
        i = 0
        while i < len(text):
            for l in range(5, 0, -1):
                seg = text[i:i + l]
                if seg in self.all_pinyins:
                    result.append(seg)
                    i += l
                    break
            else:
                result.append(text[i])
                i += 1
        return result

    def run(self):
        """
            运行
        """
        self.node.get_logger().info("AudioStreamManager线程启动")
        while self.running:
            try:
                cmd = self.cmd_queue.get(timeout=0.1)
                if cmd == 'stop':
                    self._stop_stream()
                elif cmd == 'start_wake':
                    self._start_wake()
                elif cmd == 'start_vad':
                    self._start_vad()
                elif cmd == 'shutdown':
                    self._stop_stream()
                    self.running = False
            except queue.Empty:
                continue

    def _stop_stream(self):
        """
            关闭流和重置流
        """
        with self.stream_lock:  #打开临界区
            if self.current_stream:
                self.node.get_logger().info(f"关闭当前流 ({self.mode})")
                try:
                    self.current_stream.stop()
                    self.current_stream.close()
                except Exception as e:
                    self.node.get_logger().warn(f"关闭流异常: {e}")
                self.current_stream = None
                self.stream_obj = None
                # 等待设备彻底释放
                time.sleep(0.1)
            self.mode = None

    def _start_wake(self):
        self._stop_stream()
        device_index = self.find_input_device()
        if device_index is None:
            self.node.get_logger().error("找不到麦克风设备")
            return
        self.mode = 'wake'
        self.stream_obj = self.recognizer.create_stream()
        self.node.get_logger().info(f"启动唤醒监听 (设备 {device_index})")
        def callback(indata, frames, time_info, status):
            if status:
                self.node.get_logger().warn(str(status))
            samples = indata[:, 0]
            # samples_16k = librosa.resample(samples, orig_sr=self.fs, target_sr=16000)
            samples_16k = resample_poly(samples, up=1, down=3)  # 48000 → 16000

            self.stream_obj.accept_waveform(16000, samples_16k)
            while self.recognizer.is_ready(self.stream_obj):
                self.recognizer.decode_stream(self.stream_obj)

            if self.recognizer.is_endpoint(self.stream_obj):
                result = self.recognizer.get_result(self.stream_obj)
                self.node.get_logger().info(f"识别结果: {result}")
                pred_seq = self.split_pinyin(result)
                dist = distance(" ".join(pred_seq), " ".join(self.target_seq))
                self.node.get_logger().info(f"编辑距离: {dist}")
                if dist <= 4:
                    self.node.get_logger().info("唤醒成功，发布 detected 通知")
                    self.node.wake_publisher.publish(Bool(data=True))  # ← 添加这一行！
                    self.node.get_logger().info("停止唤醒监听，等待 vad/start")
                    self.cmd_queue.put('stop')  # ← 只停止当前 stream，不触发 VAD
                #重置识别器，识别新的语音，并传入新的stream对象
                self.recognizer.reset(self.stream_obj)

        self.current_stream = sd.InputStream(
            samplerate=self.fs,
            device=device_index,
            channels=1,
            dtype='float32',
            callback=callback,
            blocksize=4800
        )
        self.current_stream.start()

    def _start_vad(self):
        self._stop_stream()
        device_index = self.find_input_device()
        if device_index is None:
            self.node.get_logger().error("找不到麦克风设备")
            return
        self.mode = 'vad'
        self.node.get_logger().info(f"启动VAD录音 (设备 {device_index})")

        voiced_frames = []
        silence_count = 0
        stop_event = threading.Event()

        start_time = time.time()
        min_record_time = 1.0  # 最少录音 1 秒
        # logger = self.node.get_logger() 
        def callback(indata, frames, time_info, status):
            """
                音频回调，用于人声判断
            """
            nonlocal silence_count, voiced_frames

            current_time = time.time()
            
            # 1. 强制录音时间
            if current_time - start_time < min_record_time:
                voiced_frames.append(indata.copy())
                return

            is_speech = self.vad.is_speech(indata.tobytes(), sample_rate=self.fs)
            if is_speech:
                voiced_frames.append(indata.copy())
                silence_count = 0
            else:
                silence_count += 1
                if silence_count < self.max_silence_frames:
                    voiced_frames.append(indata.copy())
            # logger.info(f"⏱️ 当前录音时长: {current_time - start_time:.2f}s, 连续静音帧: {silence_count}/{self.max_silence_frames}")
            if silence_count >= self.max_silence_frames:
                tail_padding = voiced_frames[-self.tail_padding_frames:] if len(voiced_frames) >= self.tail_padding_frames else voiced_frames
                voiced_frames[:] = voiced_frames[:-self.tail_padding_frames] + tail_padding
                stop_event.set()    # 事件设置为true
                raise sd.CallbackStop()

        #录制音频流
        self.current_stream = sd.InputStream(
            samplerate=self.fs,
            device=device_index,
            channels=1,
            dtype='int16',
            blocksize=self.chunk_size,
            callback=callback
        )

        self.current_stream.start() #开始录制
        stop_event.wait(timeout=6)  # 等待结束或超时，结束条件为事件为true
        self.current_stream.stop()
        self.current_stream.close()
        self.node.get_logger().info(f"录音结束，已录制 {len(voiced_frames)} 帧")
        self.current_stream = None

        # 判断是否存在声音
        if voiced_frames:
            audio_int16 = np.concatenate(voiced_frames).flatten()   #拼接音频帧
            audio = audio_int16.astype(np.float32) / 32768.0    #转换为浮点数
            # audio = librosa.resample(audio, orig_sr=48000, target_sr=16000) #降采样16K
            audio = resample_poly(audio, up=1, down=3)
            # audio = np.append(audio[0], audio[1:] - 0.97 * audio[:-1])  # 预加重

            self.node.get_logger().info("开始降噪处理...")
            audio_denoised = nr.reduce_noise(y=audio, sr=16000, stationary=False)

            msg = Float32MultiArray(data=(audio_denoised * 32767).tolist())
            self.node.get_logger().info("发布音频数据")
            self.node.audio_publisher.publish(msg)

            # timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            # filename = f"audio_{timestamp}.wav"   # 路径可改
            # audio_save = (audio_denoised * 32767).astype(np.int16)
            # wav_write(filename, 16000, audio_save)
            # self.node.get_logger().info(f"音频已保存到: {filename}")
        else:
            self.node.get_logger().info("未检测到有效语音")

        # 录音结束后不自动恢复唤醒，由节点其他逻辑触发恢复

    def enqueue_cmd(self, cmd):
        self.cmd_queue.put(cmd)

    def shutdown(self):
        self.enqueue_cmd('shutdown')
        self.join() # 等待线程结束


class VADWakeNode(Node):
    """
        语音唤醒节点
    """
    def __init__(self):
        super().__init__('vad_wake_node')
        #发布麦克风数据topic，发布关键词确定信号
        self.audio_publisher = self.create_publisher(Float32MultiArray, 'audio/raw_wav', 10)
        self.wake_publisher = self.create_publisher(Bool, 'wake_word/detected', 10)
        # 音频流线程管理
        self.stream_manager = AudioStreamManager(self)
        self.stream_manager.start() # 启动线程

        #订阅唤醒、重置信号
        self.create_subscription(Bool, 'vad/start', self.on_vad_start, 10)
        self.create_subscription(Bool, 'wake_word/reset', self.on_wake_reset, 10)
        self.create_subscription(Bool, 'vad/stop', self.on_vad_stop, 10)

        self.get_logger().info("VAD + Wake 节点启动，等待唤醒监听")
        

    def on_vad_start(self, msg):
        """
            唤醒监听
        """
        if msg.data:
            self.get_logger().info("收到 /vad/start，切换到VAD录音")
            self.stream_manager.enqueue_cmd('start_vad')

    def on_wake_reset(self, msg):
        """
            重置唤醒
        """
        if msg.data:
            self.get_logger().info("收到 /wake_word/reset，开始唤醒监听")
            self.stream_manager.enqueue_cmd('start_wake')

    def on_vad_stop(self, msg):
        """
            停止当前音频流
        """
        if msg.data:
            self.get_logger().info("收到 /vad/stop，停止当前音频流")
            self.stream_manager.enqueue_cmd('stop')

    def destroy_node(self):
        self.get_logger().info("关闭节点，停止音频流管理线程")
        self.stream_manager.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VADWakeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

