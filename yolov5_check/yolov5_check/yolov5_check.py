from std_msgs.msg import String  # or Bool
from sensor_msgs.msg import Image
# from vision_msgs.msg import BoundingBox2D,Detection2D,Pose2D
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import numpy as np
import aidcv as cv2
from .qnn_yolov5_multi import yolo_detect
from ament_index_python.packages import get_package_share_directory

class YOLODetectorNode(Node):
    def __init__(self):
        """
            初始化节点
            """
        super().__init__('yolo_detector_node')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.control_sub = self.create_subscription(
            String, '/yolo_control', self.control_callback, 10)  # 或改为 Bool
        self.publisher = self.create_publisher(
            PointStamped, '/person_detection', 10)
        self.image_pub = self.create_publisher(
            Image, '/yolo_result_image', 10)
        self.bridge = CvBridge()
        self.active = False
        self.get_logger().info('YOLO Detector Node Initialized')
    
    def control_callback(self, msg):
        """
            缺水控制识别开关
            """
        if msg.data.lower() == "start":
            self.active = True
            self.get_logger().info("YOLO detection started.")
        elif msg.data.lower() == "stop":
            self.active = False
            self.get_logger().info("YOLO detection stopped.")

    def image_callback(self, msg):
        if not self.active:
            return  # 控制识别开关
        self.get_logger().info("YOLO detect_test.")
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        bbox = self.detect_person(frame)
        if bbox:
            x,y,w,h = bbox  #x,y是左上角坐标，w,h是宽高
            # //?输出中心点坐标，但是出现问题，center数据类型转换问题         
            center = PointStamped()
            center.header = msg.header  # 记得设置时间戳和坐标系
            center.point.x = float(x + w / 2)
            center.point.y = float(y + h / 2)
            center.point.z = 0.0
            self.publisher.publish(center)
            self.get_logger().info(f" 正在检测中，检测中心位置：({center.point.x:.1f}, {center.point.y:.1f})")

            # 可选：画出检测框并发布图像
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
            out_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out_img.header = msg.header
            self.image_pub.publish(out_img)
            #show
            cv2.imshow("detect_result", frame)
    def detect_person(self, frame):
        model_name = "cutoff_yolov5s_sigmoid_w8a8.qnn229.ctx.bin"       #//!模型选择
        resource_dir = get_package_share_directory('yolov5_check')

        # 模拟或调用 YOLO 推理模型
        return yolo_detect(model_name,resource_dir,frame)# 实际调用模型推理结果

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()