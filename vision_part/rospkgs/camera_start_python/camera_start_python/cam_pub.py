import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import aidcv as cv2  # AidLux 特有
import numpy as np

FPS = 10
Width=1280
Height=720

class AidluxCameraPublisher(Node):
    def __init__(self, device_path='/dev/video2', device_type='usb', width=1280, height=720):
        super().__init__('aidlux_camera_publisher')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', FPS)
        self.bridge = CvBridge()

        self.get_logger().info(f"尝试打开摄像头：{device_path} 类型：{device_type}")
        self.cap = cv2.VideoCapture(device_path, device=device_type)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not self.cap.isOpened():
            self.get_logger().error(f" 无法打开摄像头：{device_path}")
            rclpy.shutdown()
            return

        self.get_logger().info(f'📷 摄像头打开成功，发布分辨率：{width}x{height}')
        self.timer = self.create_timer(1.0 / float(FPS), self.timer_callback)  # 10FPS

    def timer_callback(self):
        ret, frame = self.cap.read()

        #测试显示帧
        if __name__ == '__main__':        
            # 正常显示图像
            cv2.imshow('frame', frame)

        if not ret:
            self.get_logger().warn("⚠️ 摄像头帧读取失败，可能摄像头断开")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # 自动查找 USB 摄像头设备号（如/dev/video2）
    def find_usb():
        # TODO 可优化为lsusb设备号ID去锁死名字/dev/video*，然后映射为自己定义的名字
        ####先找到所有video&&usb设备
        devices = os.listdir("/sys/class/video4linux/")
        files = []
        for device in devices:
            if "video" in device:
                devices_link = os.readlink("/sys/class/video4linux/" + device)
                if "usb" in devices_link:
                    files.append(device)
        ####再找到其中索引最小的video设备
        index = 999
        for file in files:
            idx = int(file.split("video")[-1])
            if idx < index:
                index = idx
        return f"/dev/video{index}" if index != 999 else None

    device_path = find_usb()
    if not device_path:
        print(" 未找到 USB 摄像头设备")
        return

    node = AidluxCameraPublisher(device_path=device_path, device_type='usb', width=Width, height=Height)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()