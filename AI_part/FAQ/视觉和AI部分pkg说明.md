## pkg 说明

##### <span style="color:red">数据可能没有同步，但是参数类型是这些，详细数据源码</span>

### 1、camera_start_python：相机驱动包

##### 	cam_pub.py : 打开相机，并发布原始图像话题 

```python
        -node名称:aidlux_camera_publisher
        -/camera/image_raw : topic:
            -pub
            -FPS：10
            -图像宽Width=1280
            -图像高Height=720
```

##### 方法使用：

```sh
1、切换到root用户
2、ros2 run camera_start_python cam_pub运行节点
```

### 2、yolov5_check:yolov5功能部分

##### 	yolov5_check.py：检测图片人物

```python
        -node名称:yolo_detector_node 
        -/camera/image_raw:topic:  
            -sub
            -FPS:：10
        -/person_detection:topic:
            -pub
            -type:PointStamped(坐标点x,y,z类型)
            -FPS:10
        -/yolo_result_image:topic:
            -pub
            -type:Image
            -FPS:10
        -/start_yolo:server:
            -service
            -type:std_srvs/SetBool(bool处理服务)
```

##### 	qnn_yolov5_multi.py：yolo检测模块

```python
   -YOLO_Detector : Class ：yolo检测类
   		-__init__(self,model_name,resource_dir):创建对象输入两个参数
     	-Detect ： 嵌入Class ：yolo检测头，处理模型输出并生成检测结果
  		-yolo_detect(self,input_frame)：function : 对图像输入处理，yolo检测，输出目标框box
```

##### 方法使用：

        1、切换到root用户
        2、执行 python src/yolov5_check/yolov5_check/yolov5_check.py 启动节点
    	3、使用 ros2 service call /start_yolo std_srvs/srv/SetBool "{data: true}" 向节点服务端发送请求，开启yolo检测节点，并改变寄存器 bit 标志位为1
    	4、使用 ros2 run camera_start_python cam_pub 开启相机节点
    	5、使用 ros2 service call /start_yolo std_srvs/srv/SetBool "{data: false}" 向节点服务端发送请求，关闭yolo检测节点，并改变寄存器 bit 标志位为0

### 3、robot_core:机器人公共包，仅仅只是提供操作的class，不实现任何实际功能

#####     -robot_core.py：机器人功能模块，将机器人整体状态放在一个int中

```python
        -bit_enum : Class : 机器人状态bit枚举类
    	"""	（按照优先级排序，数字越小，优先级越高） """
            -SLAM_bit = 0 :第0位，SLAM建图是否完成
        -my_robot_state : Object : 机器人状态实例对象
            -get_int() : function : 获取机器人状态int
            -set_int(int) : function : 设置机器人状态int
            -set_bit(bit_pos : bit_enum, value : bool) : function : 设置机器人状态bit
            -get_bit(bit_pos : bit_enum) : function : 获取机器人状态bit
```

#####     robot_core使用方法：

```python
        1、robot_core是一个软件包将其放在src/下
        2、在需要使用的软件包的package.xml中，加上依赖robot_core包，如：
            <depend>robot_core</depend>
        3、引入机器人状态实例
            from robot_core.robot_state import my_robot_state    
        4、直接对 my_robot_state 操作，或者将其引用到你的类中 self.my_robot_state = my_robot_state
```

​      

### 4、voice_interaction：AI中关键字唤醒、ASR、LLM、STT模块

##### -vadrecorder_node.py:关键字唤醒和音频读取功能

```python
	-AudioStreamManager： Class : 音频流控制，实现音频对象的处理
    -node名称:vad_wake_node 
        -vad/start:topic: 关键字唤醒之后，向调节节点发布vad开始 
            -sub
            -type:Bool
            -FPS:：10
        -wake_word/reset:topic: 来自调节节点，重置关键字唤醒
            -sub
            -type:Bool
            -FPS:：10
        -audio/raw_wav:topic:向asr发送音频信息
            -pub
            -type:Float32MultiArray(音频流数据类型)
            -FPS:10
        -wake_word/detected:topic:关键字唤醒确认
            -pub
            -type:Bool
            -FPS:10
```

##### -等其它文件包





#### 命令行备份

1、ros2 launch voice_interaction voice_interaction_launch.py 

2、ros2 service call /start_AI std_srvs/srv/SetBool "{data: "True"}"

3、htop

4、ros2 topic pub --once /asr/result std_msgs/msg/String "{data: 'Hello，为什么植物会变黄'}"



### 知识库下载安装

1、先按照AI安装流程安装

2、启动模型api

2、 安装 UI 服务

​	aidllm install ui

3、注册和登录aidlux 账号

4、安装rag

​	aidllm install rag

5、查看可⽤向量知识库

​	aidllm remote-list rag

6、下载指定数据库

​	aidllm pull rag tesla

​	列出本地数据库

​	aidllm list rag

7、开启三个服务

​	aidllm start api -m <model_name> # 指定模型启动

​	aidllm start ui

​	aidllm start rag -n tesla # 启动指定知识库

8、[localhost:51104](http://192.168.110.61:51104/)是AI的ui接口

![image-20250723095522408](C:\Users\86152\AppData\Roaming\Typora\typora-user-images\image-20250723095522408.png)

​	

**http://<**本机**IP>:18111/docs**是知识库接口的docs请求回应说明

![image-20250723123917651](C:\Users\86152\AppData\Roaming\Typora\typora-user-images\image-20250723123917651.png)

​	开启rag服务之后，使用python申请<本机IP>：18111的请求可以得到response

##### 9、所有的 api \ rag \ ui 配置文件都在 /opt/aidlux/app 下面


