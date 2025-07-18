from std_msgs.msg import String  # or Bool
from sensor_msgs.msg import Image
# from vision_msgs.msg import BoundingBox2D,Detection2D,Pose2D
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import numpy as np
import aidcv as cv2
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import SetBool
import aidlite
import os

#根据情况导入内核器
if __name__ == '__main__':
    import sys
    sys.path.append("/home/aidlux/my_ws/src/robot_core/robot_core")  # 添加 robot_core 的父目录
    import robot_state  # 直接导入
    from robot_state import my_robot_state 
else:
    import robot_core.robot_core.robot_state    
    from robot_core.robot_core.robot_state import my_robot_state    # 引入机器人状态实例


class YOLODetectorNode(Node):
    """
        Attributes:
            self.my_robot_state:对int寄存器的引用
            self.requested_state:返回客户端的状态，START or STOP

        function:
            
    """
    def __init__(self):
        """
            初始化节点：
            订阅底盘驱动节点开启服务
        """
        super().__init__('yolo_detector_node')
        #1、初始化yolo检测模型
        # 模型基础参数
        self.coco_class = [
        "person",
        "bicycle",
        "car",
        "motorcycle",
        "airplane",
        "bus",
        "train",
        "truck",
        "boat",
        "traffic light",
        "fire hydrant",
        "stop sign",
        "parking meter",
        "bench",
        "bird",
        "cat",
        "dog",
        "horse",
        "sheep",
        "cow",
        "elephant",
        "bear",
        "zebra",
        "giraffe",
        "backpack",
        "umbrella",
        "handbag",
        "tie",
        "suitcase",
        "frisbee",
        "skis",
        "snowboard",
        "sports ball",
        "kite",
        "baseball bat",
        "baseball glove",
        "skateboard",
        "surfboard",
        "tennis racket",
        "bottle",
        "wine glass",
        "cup",
        "fork",
        "knife",
        "spoon",
        "bowl",
        "banana",
        "apple",
        "sandwich",
        "orange",
        "broccoli",
        "carrot",
        "hot dog",
        "pizza",
        "donut",
        "cake",
        "chair",
        "couch",
        "potted plant",
        "bed",
        "dining table",
        "toilet",
        "tv",
        "laptop",
        "mouse",
        "remote",
        "keyboard",
        "cell phone",
        "microwave",
        "oven",
        "toaster",
        "sink",
        "refrigerator",
        "book",
        "clock",
        "vase",
        "scissors",
        "teddy bear",
        "hair drier",
        "toothbrush",
    ]
        self.obj_class_num = 80
        self.nms_thresh = 0.45
        self.box_thresh = 0.5
        self.obj_numb_max_size = 64
        self.prop_box_size = 5 + self.obj_class_num
        self.anchors = [
            [10, 13, 16, 30, 33, 23],
            [30, 61, 62, 45, 59, 119],
            [116, 90, 156, 198, 373, 326],
        ]
        # acc_type 1 cpu 2 gpu 3 npu
        self.acctype =  3
        self.model_size = 640
        self.stride8_size = self.model_size // 8
        self.stride16_size = self.model_size // 16
        self.stride32_size = self.model_size // 32
        self.model_name = "cutoff_yolov5s_sigmoid_w8a8.qnn229.ctx.bin"
        self.model, self.interpreter, self.config = None, None, None
        if not self.init_model():
            self.get_logger().error("Failed to initialize YOLO model")
        self.get_logger().info('YOLO Detector Node Initialized')

        #2、初始化时只要开启服务
        self.srv = self.create_service(SetBool, '/start_yolo', self.handle_start_yolo_request)
        self.get_logger().info('Dummy YOLO service is ready and waiting for requests.')
        #3、寄存器读取
        self.my_robot_state = my_robot_state    #!使用引用
        # print(my_robot_state is self.my_robot_state )  # 输出 True，说明是同一个对象
        # print(id(my_robot_state) == id(self.my_robot_state ))  # 输出 True，内存地址相同

    def handle_start_yolo_request(self, request, response):
        """
            服务回调函数，用于判断是否启动下面的步骤
        """
        self.requested_state = "START" if request.data else "STOP"
        self.get_logger().info(f'Received request to {self.requested_state} YOLO.')
        response.success = True
        response.message = f'YOLO_node  {self.requested_state} successfully.'
        #3、判断启动还是停止
        if request.data:
            self.yolo_start()
        else:
            self.yolo_stop()
        
        #服务端会自动返回response对象
        return response
    
    def yolo_start(self):
        """
            启动yolo 节点
        """
        #启动订阅相机话题
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 5)
        #发布任务中心点话题、处理后图像话题
        self.publisher = self.create_publisher(PointStamped, '/person_box', 10)
        self.image_pub = self.create_publisher(Image, '/yolo_result_image', 5)
        self.bridge = CvBridge()
        self.get_logger().info('YOLO Detector Node started')
        #设置标志位1
        self.my_robot_state.set_bit(robot_state.bit_enum.YOLO_bit, True)
    
    def yolo_stop(self):
        """
            stop yolo 节点
        """
        #取消订阅相机话题
        self.destroy_subscription(self.subscription)
        #取消发布话题
        self.destroy_publisher(self.image_pub)
        self.destroy_publisher(self.publisher)
        self.get_logger().info('YOLO Detector Node stoped')
        #设置标志位0
        self.my_robot_state.set_bit(robot_state.bit_enum.YOLO_bit, False)

    def image_callback(self, msg):
        """
            订阅相机话题的回调函数
        """
        self.get_logger().info("YOLO detect_test.")
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #人物检测，输出人物框
        bbox = self.detect_person(frame)
        if bbox:
            #发布人物中心点
            x,y,w,h = bbox  #x,y是左上角坐标，w,h是宽高     
            center = PointStamped()
            center.header = msg.header  # 设置时间戳和坐标系
            center.point.x = float(x + w / 2)
            center.point.y = float(y + h / 2)
            center.point.z = 0.0
            self.publisher.publish(center)
            self.get_logger().info(f" 正在检测中，检测中心位置：({center.point.x:.1f}, {center.point.y:.1f})")

            # 发布人物检测框图像
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
            out_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out_img.header = msg.header
            self.image_pub.publish(out_img)
            self.get_logger().info("发送识别后图像")
            #显示
            # if __name__ == '__main__':
            #     cv2.imshow("detect_result", frame)
    def detect_person(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_input, scale = eqprocess(frame, self.model_size, self.model_size)
        img_input = img_input / 255
        img_input = img_input.astype(np.float32)
        # print("img_input.shape:", img_input.shape)
        
        input_tensor_data = img_input.data
        result = self.interpreter.set_input_tensor("images", input_tensor_data)
        result = self.interpreter.invoke()
        if result != 0:
            print("interpreter set_input_tensor() failed")
            return False
        if result != 0:
            print("interpreter set_input_tensor() failed")
            return False
        stride8 = self.interpreter.get_output_tensor("_326")
        if stride8 is None:
            print("sample : interpreter->get_output_tensor() 0 failed !")
            return False
        stride16 = self.interpreter.get_output_tensor("_364")
        if stride16 is None:
            print("sample : interpreter->get_output_tensor() 1 failed !")
            return False
        stride32 = self.interpreter.get_output_tensor("_402")
        if stride32 is None:
            print("sample : interpreter->get_output_tensor() 2 failed !")
            return False

        stride = [8, 16, 32]
        yolo_head = Detect(self.obj_class_num, self.anchors, stride, self.model_size)
        # 后处理部分reshape需要知道模型的output_shapes
        validCount0 = stride8.reshape(*self.output_shapes[2]).transpose(0, 3, 1, 2)
        validCount1 = stride16.reshape(*self.output_shapes[1]).transpose(0, 3, 1, 2)
        validCount2 = stride32.reshape(*self.output_shapes[0]).transpose(0, 3, 1, 2)
        pred = yolo_head([validCount0, validCount1, validCount2])
        det_pred = detect_postprocess(
            pred, frame.shape, [self.model_size, self.model_size, 3], conf_thres=0.5, iou_thres=0.45
        )
        # print("原始预测框数量：", len(det_pred))
        # 取出所有person
        person_det = det_pred[det_pred[:, 5] == 0]
        det_pred = person_det
        # 输出置信度最高的中心像素坐标（改为：面积最大的框）
        if len(person_det) > 0:
            # 先将 xywh 转换为 xyxy
            boxes_xyxy = xywh2xyxy(person_det[:, :4])
            # 提取 x1, y1, x2, y2
            x1 = boxes_xyxy[:, 0]
            y1 = boxes_xyxy[:, 1]
            x2 = boxes_xyxy[:, 2]
            y2 = boxes_xyxy[:, 3]        
            areas = (x2 - x1) * (y2 - y1)
            # 找到面积最大的框的索引
            best_index = np.argmax(areas)
            # 取出面积最大的person框
            best_person = person_det[[best_index]]  # 使用 [[index]] 保持二维数组结构
            det_pred = best_person
        else:
            self.get_logger().info("未检测到 person")
            return None  # 直接退出，防止后续访问 det_pred[0]
        
        det_pred[np.isnan(det_pred)] = 0.0
        det_pred[:, :4] = det_pred[:, :4] * scale
        # 提取坐标并计算中心点
        x, y, w, h = det_pred[0, :4].astype(int) 
        # res_img = draw_detect_res(self,frame, det_pred)

        # result = self.interpreter.destory()
        # if result != 0:
        #     print("interpreter set_input_tensor() failed")
        #     return False
        # frame_bgr = cv2.cvtColor(res_img, cv2.COLOR_RGB2BGR)
        # result_img_path = f"{os.path.splitext(os.path.abspath(__file__))[0]}.jpg"
        # result_img_path = f"/home/aidlux/aidcode/result.jpg"
        # cv2.imwrite(result_img_path, frame_bgr)
        # print(f"The result image has been saved to : {result_img_path}")
        return [x, y, w, h]  # 实际调用模型推理结果

    def init_model(self):
        resource_dir = get_package_share_directory('yolov5_check')
        # aidlite.set_log_level(aidlite.LogLevel.INFO)
        # 设置仅输出错误及以上级别的日志，屏蔽 INFO 和 DEBUG
        aidlite.set_log_level(aidlite.LogLevel.ERROR)
        aidlite.log_to_stderr()
        self.config = aidlite.Config.create_instance()
        if self.config is None:
            self.get_logger().error("Create config failed!")
            return False

        # 配置硬件加速类型
        if self.acctype == 1:
            self.config.accelerate_type = aidlite.AccelerateType.TYPE_CPU
        elif self.acctype == 2:
            self.config.accelerate_type = aidlite.AccelerateType.TYPE_GPU
            self.config.is_quantify_model = 0
        elif self.acctype == 3:
            self.config.accelerate_type = aidlite.AccelerateType.TYPE_DSP
            self.config.is_quantify_model = 1
        else:
            return False

        self.config.implement_type = aidlite.ImplementType.TYPE_LOCAL
        self.config.framework_type = aidlite.FrameworkType.TYPE_QNN229

        model_path = os.path.join(resource_dir, self.model_name)
        self.model = aidlite.Model.create_instance(model_path)
        if self.model is None:
            self.get_logger().error("Create model failed!")
            return False

        self.input_shapes = [[1, self.model_size, self.model_size, 3]]
        self.output_shapes = [[1, 20, 20, 255], [1, 40, 40, 255], [1, 80, 80, 255]]
        self.model.set_model_properties(
            self.input_shapes,
            aidlite.DataType.TYPE_FLOAT32,
            self.output_shapes,
            aidlite.DataType.TYPE_FLOAT32,
        )

        self.interpreter = aidlite.InterpreterBuilder.build_interpretper_from_model_and_config(
            self.model, self.config
        )
        if self.interpreter is None:
            self.get_logger().error("Build interpreter failed!")
            return False

        if self.interpreter.init() != 0:
            self.get_logger().error("Interpreter init failed!")
            return False

        if self.interpreter.load_model() != 0:
            self.get_logger().error("Interpreter load model failed!")
            return False

        return True

    def detect_person(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_input, scale = eqprocess(frame, self.model_size, self.model_size)
        img_input = img_input / 255
        img_input = img_input.astype(np.float32)
        # print("img_input.shape:", img_input.shape)
        
        input_tensor_data = img_input.data
        result = self.interpreter.set_input_tensor("images", input_tensor_data)
        result = self.interpreter.invoke()
        if result != 0:
            print("interpreter set_input_tensor() failed")
            return False
        if result != 0:
            print("interpreter set_input_tensor() failed")
            return False
        stride8 = self.interpreter.get_output_tensor("_326")
        if stride8 is None:
            print("sample : interpreter->get_output_tensor() 0 failed !")
            return False
        stride16 = self.interpreter.get_output_tensor("_364")
        if stride16 is None:
            print("sample : interpreter->get_output_tensor() 1 failed !")
            return False
        stride32 = self.interpreter.get_output_tensor("_402")
        if stride32 is None:
            print("sample : interpreter->get_output_tensor() 2 failed !")
            return False

        stride = [8, 16, 32]
        yolo_head = Detect(self.obj_class_num, self.anchors, stride, self.model_size)
        # 后处理部分reshape需要知道模型的output_shapes
        validCount0 = stride8.reshape(*self.output_shapes[2]).transpose(0, 3, 1, 2)
        validCount1 = stride16.reshape(*self.output_shapes[1]).transpose(0, 3, 1, 2)
        validCount2 = stride32.reshape(*self.output_shapes[0]).transpose(0, 3, 1, 2)
        pred = yolo_head([validCount0, validCount1, validCount2])
        det_pred = detect_postprocess(
            pred, frame.shape, [self.model_size, self.model_size, 3], conf_thres=0.5, iou_thres=0.45
        )
        # print("原始预测框数量：", len(det_pred))
        # 取出所有person
        person_det = det_pred[det_pred[:, 5] == 0]
        det_pred = person_det
        # 输出置信度最高的中心像素坐标（改为：面积最大的框）
        if len(person_det) > 0:
            # 先将 xywh 转换为 xyxy
            boxes_xyxy = xywh2xyxy(person_det[:, :4])
            # 提取 x1, y1, x2, y2
            x1 = boxes_xyxy[:, 0]
            y1 = boxes_xyxy[:, 1]
            x2 = boxes_xyxy[:, 2]
            y2 = boxes_xyxy[:, 3]        
            areas = (x2 - x1) * (y2 - y1)
            # 找到面积最大的框的索引
            best_index = np.argmax(areas)
            # 取出面积最大的person框
            best_person = person_det[[best_index]]  # 使用 [[index]] 保持二维数组结构
            det_pred = best_person
        else:
            self.get_logger().info("未检测到 person")
            return None  # 直接退出，防止后续访问 det_pred[0]
        
        det_pred[np.isnan(det_pred)] = 0.0
        det_pred[:, :4] = det_pred[:, :4] * scale
        # 提取坐标并计算中心点
        x, y, w, h = det_pred[0, :4].astype(int) 

        return [x, y, w, h]  # 实际调用模型推理结果
        
    def destroy_node(self):
        # 先销毁 interpreter
        if hasattr(self, "interpreter"):
            result = self.interpreter.destory()
            if result != 0:
                self.get_logger().warn("Failed to destroy interpreter.")
            else:
                self.get_logger().info("Interpreter destroyed.")
        # 再调用父类销毁
        super().destroy_node()

def eqprocess(image, size1, size2):
    h, w, _ = image.shape
    mask = np.zeros((size1, size2, 3), dtype=np.float32)
    scale1 = h / size1
    scale2 = w / size2
    if scale1 > scale2:
        scale = scale1
    else:
        scale = scale2
    img = cv2.resize(image, (int(w / scale), int(h / scale)))
    mask[: int(h / scale), : int(w / scale), :] = img
    return mask, scale

def xywh2xyxy(x):
    """
    Box (center x, center y, width, height) to (x1, y1, x2, y2)
    """
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def xyxy2xywh(box):
    """
    Box (left_top x, left_top y, right_bottom x, right_bottom y) to (left_top x, left_top y, width, height)
    """
    box[:, 2:] = box[:, 2:] - box[:, :2]
    return box


def NMS(dets, scores, thresh):
    """
    单类NMS算法
    dets.shape = (N, 5), (left_top x, left_top y, right_bottom x, right_bottom y, Scores)
    """
    x1 = dets[:, 0]
    y1 = dets[:, 1]
    x2 = dets[:, 2]
    y2 = dets[:, 3]
    areas = (y2 - y1 + 1) * (x2 - x1 + 1)
    keep = []
    index = scores.argsort()[::-1]
    while index.size > 0:
        i = index[0]  # every time the first is the biggst, and add it directly
        keep.append(i)
        x11 = np.maximum(x1[i], x1[index[1:]])  # calculate the points of overlap
        y11 = np.maximum(y1[i], y1[index[1:]])
        x22 = np.minimum(x2[i], x2[index[1:]])
        y22 = np.minimum(y2[i], y2[index[1:]])
        w = np.maximum(0, x22 - x11 + 1)  # the weights of overlap
        h = np.maximum(0, y22 - y11 + 1)  # the height of overlap
        overlaps = w * h
        ious = overlaps / (areas[i] + areas[index[1:]] - overlaps)
        idx = np.where(ious <= thresh)[0]
        index = index[idx + 1]  # because index start from 1

    return keep


def clip_coords(boxes, img_shape):
    # Clip bounding xyxy bounding boxes to image shape (height, width)
    boxes[:, 0].clip(0, img_shape[1], out=boxes[:, 0])  # x1
    boxes[:, 1].clip(0, img_shape[0], out=boxes[:, 1])  # y1
    boxes[:, 2].clip(0, img_shape[1], out=boxes[:, 2])  # x2
    boxes[:, 3].clip(0, img_shape[0], out=boxes[:, 3])  # y2


def detect_postprocess(
    prediction, img0shape, img1shape, conf_thres=0.25, iou_thres=0.45
):
    """
    检测输出后处理
    prediction: aidlite模型预测输出
    img0shape: 原始图片shape
    img1shape: 输入图片shape
    conf_thres: 置信度阈值
    iou_thres: IOU阈值
    return: list[np.ndarray(N, 5)], 对应类别的坐标框信息, xywh、conf
    """
    h, w, _ = img1shape
    valid_condidates = prediction[prediction[..., 4] > conf_thres]
    valid_condidates[:, 5:] *= valid_condidates[:, 4:5]
    valid_condidates[:, :4] = xywh2xyxy(valid_condidates[:, :4])

    max_det = 300
    max_wh = 7680
    max_nms = 30000
    valid_condidates[:, 4] = valid_condidates[:, 5:].max(1)
    valid_condidates[:, 5] = valid_condidates[:, 5:].argmax(1)
    sort_id = np.argsort(valid_condidates[:, 4])[::-1]
    valid_condidates = valid_condidates[sort_id[:max_nms]]
    boxes, scores = (
        valid_condidates[:, :4] + valid_condidates[:, 5:6] * max_wh,
        valid_condidates[:, 4],
    )
    index = NMS(boxes, scores, iou_thres)[:max_det]
    out_boxes = valid_condidates[index]
    clip_coords(out_boxes[:, :4], img0shape)
    out_boxes[:, :4] = xyxy2xywh(out_boxes[:, :4])
    # print("检测到{}个区域".format(len(out_boxes)))
    return out_boxes

def draw_detect_res(self,img, det_pred):
    """
    检测结果绘制
    """
    img = img.astype(np.uint8)
    color_step = int(255 / len(self.coco_class))
    for i in range(len(det_pred)):
        x1, y1, x2, y2 = [int(t) for t in det_pred[i][:4]]
        score = det_pred[i][4]
        cls_id = int(det_pred[i][5])

        print(i + 1, [x1, y1, x2 + x1, y2 + y1], score, self.coco_class[cls_id])

        cv2.putText(
            img,
            f"{self.coco_class[cls_id]}",
            (x1, y1 - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )
        cv2.rectangle(
            img,
            (x1, y1),
            (x2 + x1, y2 + y1),
            (0, int(cls_id * color_step), int(255 - cls_id * color_step)),
            thickness=2,
        )
    return img

class Detect:
    # YOLOv5 Detect head for detection models
    def __init__(self, nc=80, anchors=(), stride=[], image_size=640):  # detection layer
        super().__init__()
        self.nc = nc  # number of classes
        self.no = nc + 5  # number of outputs per anchor
        self.stride = stride
        self.nl = len(anchors)  # number of detection layers
        self.na = len(anchors[0]) // 2  # number of anchors
        self.grid, self.anchor_grid = [0] * self.nl, [0] * self.nl
        self.anchors = np.array(anchors, dtype=np.float32).reshape(self.nl, -1, 2)

        base_scale = image_size // 8
        for i in range(self.nl):
            self.grid[i], self.anchor_grid[i] = self._make_grid(
                base_scale // (2**i), base_scale // (2**i), i
            )

    def _make_grid(self, nx=20, ny=20, i=0):
        y, x = np.arange(ny, dtype=np.float32), np.arange(nx, dtype=np.float32)
        yv, xv = np.meshgrid(y, x)
        yv, xv = yv.T, xv.T
        # add grid offset, i.e. y = 2.0 * x - 0.5
        grid = np.stack((xv, yv), 2)
        grid = grid[np.newaxis, np.newaxis, ...]
        grid = np.repeat(grid, self.na, axis=1) - 0.5
        anchor_grid = self.anchors[i].reshape((1, self.na, 1, 1, 2))
        anchor_grid = np.repeat(anchor_grid, repeats=ny, axis=2)
        anchor_grid = np.repeat(anchor_grid, repeats=nx, axis=3)
        return grid, anchor_grid

    def sigmoid(self, arr):
        return 1 / (1 + np.exp(-arr))

    def __call__(self, x):
        z = []  # inference output
        for i in range(self.nl):
            bs, _, ny, nx = x[i].shape
            x[i] = x[i].reshape(bs, self.na, self.no, ny, nx).transpose(0, 1, 3, 4, 2)
            y = self.sigmoid(x[i])
            y[..., 0:2] = (y[..., 0:2] * 2.0 + self.grid[i]) * self.stride[i]  # xy
            y[..., 2:4] = (y[..., 2:4] * 2) ** 2 * self.anchor_grid[i]  # wh
            z.append(y.reshape(bs, self.na * nx * ny, self.no))

        return np.concatenate(z, 1)

def main(args=None):
    rclpy.init(args=args)
    # 创建节点
    node = YOLODetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()