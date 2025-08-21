### vision_part



- yolov5 :实现yolov5物体检测，包含任务检测
- rospkgs:视觉部分的使用 ros 包，一个相机驱动包，一个yolov5检测包
  - 其中yolov5_check包下qnn_yolov5_multi.py是对yolov5应用的封装和测试



#### 操作流程

##### 1、得到模型文件

直接访问https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt下载yolov5.pt模型文件

##### 2、模型格式转换

下载的pt格式一般是在英伟达环境加载，需要将其转为高通DSP硬件专属bin模型文件

(1) 更新pip版本为25.1.1

```shell
python3.10 -m pip install --upgrade pip

pip -V
```

(2) 安装ultralytics和onnx

```shell
pip install ultralytics onnx
```

(3) 设置yolo命令的环境变量

```shell
echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc

source ~/.bashrc  # 使修改立即生效

验证：执行 yolo --version，若输出版本号（如 8.3.152），则说明命令已生效。
```

(4) 将pt转换为onnx

新建python脚本，内容如下，并执行，可以在本文件夹生成onnx文件

```python
#!/usr/bin/env python

from ultralytics import YOLO

 

# 加载同级目录下的.pt模型文件

model = YOLO('./yolov5s.pt')  # 替换为实际模型文件名

 

# 导出ONNX配置参数

export_params = {

  'format': 'onnx',

  'opset': 12,      # 推荐算子集版本

  'simplify': True,   # 启用模型简化

  'dynamic': False,   # 固定输入尺寸

  'imgsz': 640,     # 标准输入尺寸

  'half': False     # 保持FP32精度

}

 

# 执行转换并保存到同级目录

model.export(**export_params)
```

 

(5) 进入[AI Model Optimizer](#/model-convert)，进行ONNX格式转换为QNN支持的格式

Step1：选择模型优化，模型格式选择onnx格式上传模型

![img](file:///C:\Users\86152\AppData\Local\Temp\ksohtml19516\wps1.jpg) 

Step2：选择芯片型号以及目标框架，这里我们选择QCS6490+Qnn2.31

![img](file:///C:\Users\86152\AppData\Local\Temp\ksohtml19516\wps2.jpg) 

Step3：点击查看模型，使用Netron查看模型结构，进行输入输出的填写，使用Netron工具查看onnx模型结构，选择剪枝位置

/model.24/m.0/Conv_output_0

/model.24/m.1/Conv_output_0

/model.24/m.2/Conv_output_0

![img](file:///C:\Users\86152\AppData\Local\Temp\ksohtml19516\wps3.jpg) 

参考上图中红色框部分填写，其他不变，注意开启自动量化功能，AIMO更多操作查看使用说明或参考[AIMO平台](https://docs.aidlux.com/guide/software/ai-platform-portal-aimo)

Step4：接下来进行提交即可，转换完成后将目标模型文件下载，解压缩后其中的.bin.aidem文件即为模型文件

![img](file:///C:\Users\86152\AppData\Local\Temp\ksohtml19516\wps4.jpg) 

QNN测试代码

该代码实现了基于aidlite库的 YOLOv5 目标检测流程，主要功能包括：

命令行参数解析（模型路径、图像路径、推理次数等）；

图像预处理（等比例缩放、填充、归一化）；

使用aidlite加载模型并执行推理（支持 QNN/SNPE 框架，DSP 加速）；

推理性能统计（耗时均值、最大值、最小值、方差）；

检测结果后处理（置信度筛选、非极大值抑制 NMS、坐标映射）；

可视化检测结果（绘制边界框和类别标签，并保存图像）。

```python
\#!/usr/bin/env python3

\# import sys

import os  #os模块提供了与操作系统进行交互的功能。

import sys #sys模块提供了与Python解释器进行交互的函数。

import time

 

import aidlite

import cv2

import numpy as np

 

OBJ_CLASS_NUM = 80 #COCO数据集的类别数

NMS_THRESH = 0.45  #非极大值抑制的阈值

BOX_THRESH = 0.5   #检测框的置信度阈值

MODEL_SIZE = 640   #模型输入尺寸

 

OBJ_NUMB_MAX_SIZE = 64 #最大目标检测数

PROP_BOX_SIZE = 5 + OBJ_CLASS_NUM #每个预测框的输出维度：5 表示 (x, y, w, h, confidence)，加上 80 个类别的概率

STRIDE8_SIZE = MODEL_SIZE / 8 #特征图下采样步长为 8 的特征图大小（即 640/8=80）

STRIDE16_SIZE = MODEL_SIZE / 16

STRIDE32_SIZE = MODEL_SIZE / 32

 

\#定义了三组不同尺度的锚框（anchor boxes），每组包含 3 个锚框。这些锚框用于生成候选框。

anchors = [

  [10, 13, 16, 30, 33, 23],

  [30, 61, 62, 45, 59, 119],

  [116, 90, 156, 198, 373, 326],

]

 

\#COCO 数据集中所有目标类别的名称列表，共 80 个类别。

coco_class = [

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

 

\#将边界框从中心坐标 (cx, cy, w, h) 转换为左上角和右下角坐标 (x1, y1, x2, y2)

def eqprocess(image, size1, size2):

  h, w, _ = image.shape

  mask = np.zeros((size1, size2, 3), dtype=np.float32)

  scale1 = h / size1

  scale2 = w / size2

  if scale1 > scale2:

​    scale = scale1

  else:

​    scale = scale2

  img = cv2.resize(image, (int(w / scale), int(h / scale)))

  mask[: int(h / scale), : int(w / scale), :] = img

  return mask, scale

 

\#同上，描述该函数的作用是将边界框格式转换。

def xywh2xyxy(x):

  """

  Box (center x, center y, width, height) to (x1, y1, x2, y2)

  """

  y = np.copy(x)

  y[:, 0] = x[:, 0] - x[:, 2] / 2 # top left x

  y[:, 1] = x[:, 1] - x[:, 3] / 2 # top left y

  y[:, 2] = x[:, 0] + x[:, 2] / 2 # bottom right x

  y[:, 3] = x[:, 1] + x[:, 3] / 2 # bottom right y

  return y

 

\#描述该函数将边界框从 (x1, y1, x2, y2) 格式转换为 (x1, y1, w, h) 格式

def xyxy2xywh(box):

  """

  Box (left_top x, left_top y, right_bottom x, right_bottom y) to (left_top x, left_top y, width, height)

  """

  box[:, 2:] = box[:, 2:] - box[:, :2]

  return box

 

\#描述这是一个针对单类的 NMS（非极大值抑制） 算法，用于去除冗余的边界框。

def NMS(dets, scores, thresh):

  """

  单类NMS算法

  dets.shape = (N, 5), (left_top x, left_top y, right_bottom x, right_bottom y, Scores)

  """

  x1 = dets[:, 0]

  y1 = dets[:, 1]

  x2 = dets[:, 2]

  y2 = dets[:, 3]

  areas = (y2 - y1 + 1) * (x2 - x1 + 1)#计算面积

  keep = []

  index = scores.argsort()[::-1]  # 按置信度从高到低排序

  while index.size > 0:

​    i = index[0]  # every time the first is the biggst, and add it directly

​    keep.append(i)

​    x11 = np.maximum(x1[i], x1[index[1:]])  # calculate the points of overlap

​    y11 = np.maximum(y1[i], y1[index[1:]])

​    x22 = np.minimum(x2[i], x2[index[1:]])

​    y22 = np.minimum(y2[i], y2[index[1:]])

​    w = np.maximum(0, x22 - x11 + 1)  # the weights of overlap

​    h = np.maximum(0, y22 - y11 + 1)  # the height of overlap

​    overlaps = w * h

​    ious = overlaps / (areas[i] + areas[index[1:]] - overlaps)

​    idx = np.where(ious <= thresh)[0]

​    index = index[idx + 1]  # because index start from 1

 

  return keep

 

\#描述该函数将边界框裁剪到图像范围内，防止越界。

def clip_coords(boxes, img_shape):

  \# Clip bounding xyxy bounding boxes to image shape (height, width)

  boxes[:, 0].clip(0, img_shape[1], out=boxes[:, 0])  # x1

  boxes[:, 1].clip(0, img_shape[0], out=boxes[:, 1])  # y1

  boxes[:, 2].clip(0, img_shape[1], out=boxes[:, 2])  # x2

  boxes[:, 3].clip(0, img_shape[0], out=boxes[:, 3])  # y2

 

\#描述该函数对模型输出进行后处理，包括筛选高置信度框、NMS 去重等操作。

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

​    valid_condidates[:, :4] + valid_condidates[:, 5:6] * max_wh,

​    valid_condidates[:, 4],

  )

  index = NMS(boxes, scores, iou_thres)[:max_det]

  out_boxes = valid_condidates[index]

  clip_coords(out_boxes[:, :4], img0shape)

  out_boxes[:, :4] = xyxy2xywh(out_boxes[:, :4])

  print("检测到{}个区域".format(len(out_boxes)))

  return out_boxes

 

\#描述该函数用于在图像上绘制检测结果，包括边界框和类别标签。

def draw_detect_res(img, det_pred):

  """

  检测结果绘制

  """

  img = img.astype(np.uint8)

  color_step = int(255 / len(coco_class))

  for i in range(len(det_pred)):

​    x1, y1, x2, y2 = [int(t) for t in det_pred[i][:4]]

​    score = det_pred[i][4]

​    cls_id = int(det_pred[i][5])

 

​    print(i + 1, [x1, y1, x2 + x1, y2 + y1], score, coco_class[cls_id])

 

​    cv2.putText(

​      img,

​      f"{coco_class[cls_id]}",

​      (x1, y1 - 6),

​      cv2.FONT_HERSHEY_SIMPLEX,

​      0.5,

​      (255, 255, 255),

​      1,

​    )

​    cv2.rectangle(

​      img,

​      (x1, y1),

​      (x2 + x1, y2 + y1),

​      (0, int(cls_id * color_step), int(255 - cls_id * color_step)),

​      thickness=10,

​    )

 

  return img

 

\#描述该类是 YOLOv5 的检测头，用于处理模型输出并生成最终的检测结果。

class Detect:

  \# YOLOv5 Detect head for detection models

  def __init__(self, nc=80, anchors=(), stride=[], image_size=640):  # detection layer

​    super().__init__()

​    self.nc = nc # number of classes

​    self.no = nc + 5 # number of outputs per anchor

​    self.stride = stride

​    self.nl = len(anchors)  # number of detection layers

​    self.na = len(anchors[0]) // 2 # number of anchors

​    self.grid, self.anchor_grid = [0] * self.nl, [0] * self.nl

​    self.anchors = np.array(anchors, dtype=np.float32).reshape(self.nl, -1, 2)

 

​    base_scale = image_size // 8

​    for i in range(self.nl):

​      self.grid[i], self.anchor_grid[i] = self._make_grid(

​        base_scale // (2**i), base_scale // (2**i), i

​      )

 

  def _make_grid(self, nx=20, ny=20, i=0):

​    y, x = np.arange(ny, dtype=np.float32), np.arange(nx, dtype=np.float32)

​    yv, xv = np.meshgrid(y, x)

​    yv, xv = yv.T, xv.T

​    \# add grid offset, i.e. y = 2.0 * x - 0.5

​    grid = np.stack((xv, yv), 2)

​    grid = grid[np.newaxis, np.newaxis, ...]

​    grid = np.repeat(grid, self.na, axis=1) - 0.5

​    anchor_grid = self.anchors[i].reshape((1, self.na, 1, 1, 2))

​    anchor_grid = np.repeat(anchor_grid, repeats=ny, axis=2)

​    anchor_grid = np.repeat(anchor_grid, repeats=nx, axis=3)

​    return grid, anchor_grid

 

  def sigmoid(self, arr):

​    return 1 / (1 + np.exp(-arr))

 

  def __call__(self, x):

​    z = []  # inference output

​    for i in range(self.nl):

​      bs, _, ny, nx = x[i].shape

​      x[i] = x[i].reshape(bs, self.na, self.no, ny, nx).transpose(0, 1, 3, 4, 2)

​      y = self.sigmoid(x[i])

​      y[..., 0:2] = (y[..., 0:2] * 2.0 + self.grid[i]) * self.stride[i]  # xy

​      y[..., 2:4] = (y[..., 2:4] * 2) ** 2 * self.anchor_grid[i]  # wh

​      z.append(y.reshape(bs, self.na * nx * ny, self.no))

 

​    return np.concatenate(z, 1)

 

def show_help_prompt():

  print("python3 ./qnn_yolov5_multi.py 3 Run on the DSP")

  return False

 

def main():

  argvs = len(sys.argv)

  if argvs < 2:

​    return show_help_prompt()

  acc_type = int(sys.argv[1])

 

  aidlite.set_log_level(aidlite.LogLevel.INFO)

  aidlite.log_to_stderr()

  print(f"Aidlite library version : {aidlite.get_library_version()}")

  print(f"Aidlite python library version : {aidlite.get_py_library_version()}")

 

  config = aidlite.Config.create_instance()

  if config is None:

​    print("Create config failed !")

​    return False

 

  model_name = "cutoff_yolov5s_sigmoid_w8a8.qnn229.ctx.bin"

  \#model_name = "cutoff_yolov8x_qcs6490_w8a8.qnn231.ctx.bin"    #//!模型选择

  if acc_type == 1:

​    config.accelerate_type = aidlite.AccelerateType.TYPE_CPU

  elif acc_type == 2:

​    config.accelerate_type = aidlite.AccelerateType.TYPE_GPU

​    config.is_quantify_model = 0

  elif acc_type == 3:

​    config.accelerate_type = aidlite.AccelerateType.TYPE_DSP   #//!选择DSP方式

​    config.is_quantify_model = 1

  else:

​    return show_help_prompt()

 

  config.implement_type = aidlite.ImplementType.TYPE_LOCAL

  config.framework_type = aidlite.FrameworkType.TYPE_QNN229    #//!选择QNN229方式

 

  \# model_path = os.path.join(

  \#   resource_dir, "cutoff_yolov5s_640_sigmoid_int8.serialized.bin"

  \# )

  resource_dir = '/home/aidlux/yolov5/data/qnn_yolov5_multi/'

  model_path = os.path.join(resource_dir, model_name) #加载模型

  model = aidlite.Model.create_instance(model_path)

  if model is None:

​    print("Create model failed !")

​    return False

  \#输入张量，[每次推理图像张数,图像高，图像宽，图像通道数]

  input_shapes = [[1, MODEL_SIZE, MODEL_SIZE, 3]]

  \#三个输出层（feature map），分别对应不同尺度的目标检测[每次推理图像张数,特征图高，特征图宽，输出纬度]

  \#每个锚框预测 (4 + 1 + 80) = 85 个值（xywh + confidence + class probs），每层有 3 个 anchor box → 3 × 85 = 255

  \#三种输出，分别是检测小目标（深层特征）、中目标（中等深度特征）、大目标（浅层特征）

  output_shapes = [[1, 20, 20, 255], [1, 40, 40, 255], [1, 80, 80, 255]]

  model.set_model_properties(

​    input_shapes,

​    aidlite.DataType.TYPE_FLOAT32,

​    output_shapes,

​    aidlite.DataType.TYPE_FLOAT32,

  )

 

  interpreter = aidlite.InterpreterBuilder.build_interpretper_from_model_and_config(

​    model, config

  )

  if interpreter is None:

​    print("build_interpretper_from_model_and_config failed !")

​    return None

  result = interpreter.init()

  if result != 0:

​    print("interpreter init failed !")

​    return False

  result = interpreter.load_model()

  if result != 0:

​    print("interpreter load model failed !")

​    return False

  print("detect model load success!")

 

  input_tensor_info = interpreter.get_input_tensor_info()

  if len(input_tensor_info) == 0 :

​    printf("interpreter get_input_tensor_info() failed !\n")

​    return False

  for gi, graph_tensor_info in enumerate(input_tensor_info):

​    for ti, tensor_info in enumerate(graph_tensor_info):

​      print(  f"Input  tensor : Graph[{gi}]-Tensor[{ti}]-name[{tensor_info.name}]"

​          f"-element_count[{tensor_info.element_count}]-element_type[{tensor_info.element_type}]"

​          f"-dimensions[{tensor_info.dimensions}]-shape{tensor_info.shape}")

 

  output_tensor_info = interpreter.get_output_tensor_info()

  if len(output_tensor_info) == 0 :

​    printf("interpreter get_output_tensor_info() failed !\n")

​    return False

  for gi, graph_tensor_info in enumerate(output_tensor_info):

​    for ti, tensor_info in enumerate(graph_tensor_info):

​      print(  f"Output tensor : Graph[{gi}]-Tensor[{ti}]-name[{tensor_info.name}]"

​          f"-element_count[{tensor_info.element_count}]-element_type[{tensor_info.element_type}]"

​          f"-dimensions[{tensor_info.dimensions}]-shape{tensor_info.shape}")

 

  stride8 = stride16 = stride32 = None

 

  image_path = os.path.join(resource_dir, "image.jpg")

  frame = cv2.imread(image_path)

  frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

  img_input, scale = eqprocess(frame, MODEL_SIZE, MODEL_SIZE)

  img_input = img_input / 255

  img_input = img_input.astype(np.float32)

 

  sum_time_0 = 0.0

  sum_time_1 = 0.0

  sum_time_2 = 0.0

  _counter = 10

  for idx in range(_counter):

​    st0 = time.time()

​    input_tensor_data = img_input.data

​    \# result = interpreter.set_input_tensor(0, input_tensor_data)

​    result = interpreter.set_input_tensor("images", input_tensor_data)

​    if result != 0:

​      print("interpreter set_input_tensor() failed")

​      return False

​    et0 = time.time()

​    dur0 = et0 - st0

​    sum_time_0 += dur0

​    print(f"current [{idx}] set_input_tensor cost time :{dur0} ms")

 

​    st1 = time.time()

​    result = interpreter.invoke()  #执行推理

​    if result != 0:

​      print("interpreter set_input_tensor() failed")

​      return False

​    et1 = time.time()

​    dur1 = et1 - st1

​    sum_time_1 += dur1

​    print(f"current [{idx}] invoke cost time :{dur1} ms")

 

​    st2 = time.time()

 

​    \# stride8 = interpreter.get_output_tensor(0)

​    stride8 = interpreter.get_output_tensor("_326") #获取不同层级的输出张量（如 stride8、stride16、stride32）

​    if stride8 is None:

​      print("sample : interpreter->get_output_tensor() 0 failed !")

​      return False

​    print(f"len(stride8 {len(stride8)}")

 

​    \# stride16 = interpreter.get_output_tensor(1)

​    stride16 = interpreter.get_output_tensor("_364")

​    if stride16 is None:

​      print("sample : interpreter->get_output_tensor() 1 failed !")

​      return False

​    print(f"len(stride16 {len(stride16)}")

 

​    \# stride32 = interpreter.get_output_tensor(2)

​    stride32 = interpreter.get_output_tensor("_402")

​    if stride32 is None:

​      print("sample : interpreter->get_output_tensor() 2 failed !")

​      return False

​    print(f"len(stride32 {len(stride32)}")

​    et2 = time.time()

​    dur2 = et2 - st2

​    sum_time_2 += dur2

​    print(f"current [{idx}] get_output_tensor cost time :{dur2} ms")

 

  print(

​    f"repeat [{_counter}] times , input[{sum_time_0 * 1000}]ms --- invoke[{sum_time_1 * 1000}]ms --- output[{sum_time_2 * 1000}]ms --- sum[{(sum_time_0 + sum_time_1 + sum_time_2) * 1000}]ms"

  )

 

  stride = [8, 16, 32]

  yolo_head = Detect(OBJ_CLASS_NUM, anchors, stride, MODEL_SIZE)

  \# 后处理部分reshape需要知道模型的output_shapes

  validCount0 = stride8.reshape(*output_shapes[2]).transpose(0, 3, 1, 2)

  validCount1 = stride16.reshape(*output_shapes[1]).transpose(0, 3, 1, 2)

  validCount2 = stride32.reshape(*output_shapes[0]).transpose(0, 3, 1, 2)

  \#使用 Detect 类对模型输出进行解码，得到最终的检测结果。

  pred = yolo_head([validCount0, validCount1, validCount2])

  \#对模型输出进行后处理，得到可用的边界框和类别信息。

  det_pred = detect_postprocess(

​    pred, frame.shape, [MODEL_SIZE, MODEL_SIZE, 3], conf_thres=0.5, iou_thres=0.45

  )

 

  \#//?根据置信度最大取出best_person

  \# 取出所有person

  person_det = det_pred[det_pred[:, 5] == 0]

  det_pred = person_det

  \# 输出置信度最高的中心像素坐标（改为：面积最大的框）

  if len(person_det) > 0:

​     \# 先将 xywh 转换为 xyxy

​    boxes_xyxy = xywh2xyxy(person_det[:, :4])

​     \# 提取 x1, y1, x2, y2

​    x1 = boxes_xyxy[:, 0]

​    y1 = boxes_xyxy[:, 1]

​    x2 = boxes_xyxy[:, 2]

​    y2 = boxes_xyxy[:, 3]     

​    areas = (x2 - x1) * (y2 - y1)

​    \# 找到面积最大的框的索引

​    best_index = np.argmax(areas)

​    \# 取出面积最大的person框

​    best_person = person_det[[best_index]]  # 使用 [[index]] 保持二维数组结构

​    det_pred = best_person

  else:

​    print("未检测到 person")

  

 

  det_pred[np.isnan(det_pred)] = 0.0

  det_pred[:, :4] = det_pred[:, :4] * scale

  \# 提取坐标并计算中心点

  x, y, w, h = det_pred[0, :4].astype(int)   #x,y左上角点，w,h宽高

  center_x = x + w// 2

  center_y = y + h // 2

  print(f"Person 中心坐标: ({center_x}, {center_y})")

  print(f"x, y, w, h: {x, y, w, h}\n\n\n\n\n\n\n")

  \#在原始图像上绘制检测结果

  res_img = draw_detect_res(frame, det_pred)

  \#画出中心点画一个点

  cv2.circle(res_img, (center_x, center_y), radius=5, color=(0, 0, 255), thickness=20)

 

  result = interpreter.destory()

  if result != 0:

​    print("interpreter set_input_tensor() failed")

​    return False

  frame_bgr = cv2.cvtColor(res_img, cv2.COLOR_RGB2BGR)

  result_img_path = f"{os.path.splitext(os.path.abspath(__file__))[0]}.jpg"

  cv2.imwrite(result_img_path, frame_bgr)

  print(f"The result image has been saved to : {result_img_path}")

  return True

 

if __name__ == "__main__":

  main()
```

 

执行python your_code.py 3

输出图像：

![img](file:///C:\Users\86152\AppData\Local\Temp\ksohtml19516\wps5.jpg) 

