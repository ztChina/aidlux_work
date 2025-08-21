# 目录
* [基本功能](#基本功能)
* [调试文档](#wukong语音交互调试文档)

# 基本功能
## 1.与`ROS`通信
```
pip install pyzmq
```
- `data_publisher.py`
- `ros_listener.py`



## 2.与`stm32`通信
```
pip install pyserial
```
- `get_stm32_msg.py`<br>
监听串口，一旦满足条件，则会发布到指定信息到socket本地6000端口。

## 3.功能：
### 1. `JustRun.py`：  <br>
接收跑路指令，并把消息发送给`ROS`，自动导航到指定地点。 <br>
### 2. `Water.py`： <br>
接收用户询问，返回湿度情况。
###  3. `get_stm32_msg.py`：<br>
接收`stm`串口通信，并在后台检测湿度变化，满足条件后唤醒，并输出语音，然后发消息给`ros`。
##### 3.1 `LifeCycleHandler.py`：


```python
# ...existing code...

    def onHumidityWakeup(self, text="嘴巴干了，猎杀时刻，启动！"):
        """
        湿度唤醒：只提示，不进入录音
        """
        logger.info("onHumidityWakeup")
        self._beep_hi()
        if config.get("/LED/enable", False):
            LED.wakeup()
        self._unihiker and self._unihiker.record(1, text)
        self._unihiker and self._unihiker.wakeup()
        # 直接语音播报
        self._conversation.say(text, True)

# ...existing code...
```
##### 3.2 `wukong.py`：

```python
import sys
import socket
import threading
sys.path.append("/root/.wukong/custom")
from data_publisher import DataPublisher

# ...existing code...    
    def socket_listener(self, host='127.0.0.1', port=6000):
        """
        监听本地 socket，接收唤醒信号
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((host, port))
        s.listen(1)
        print(f"湿度唤醒监听已启动: {host}:{port}")
        while True:
            conn, addr = s.accept()
            data = conn.recv(1024).decode()
            if data == "WAKEUP":
                print("收到湿度唤醒信号，唤醒机器人")
                self.lifeCycleHandler.onHumidityWakeup()
                # time.sleep(1.5)  # 等待唤醒
                publisher = get_publisher()
                while True:
                    message = "status 找人通知：口干干，喝水水"
                    if publisher.send_message(message):
                        break
            conn.close()
# ...existing code...
```

```python
 def run(self):
        self.init()
# ...existing code...
        # 启动 socket 监听线程
        threading.Thread(target=self.socket_listener, daemon=True).start()
# ...existing code...
```


### 4. `move.py`：  <br>
接收移动指令，通过百度`UNIT`识别`方向`和`距离`，并把消息发送给`ROS`。 <br>



# wukong语音交互调试文档

官方教程：[链接](https://wukong.hahack.com/#/README)

### 1.安装

板子系统默认3.10版本，不要`python3.10`版本，有bug，安装3.9版本

#### 1.1 下载安装激活`conda`虚拟环境

```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
```

**激活虚拟环境**

```bash
source ~/miniconda3/bin/activate
#conda init --all
conda create -n py39 python=3.9
conda activate py39
```

#### 1.2 手动安装wukong

[教程](https://wukong.hahack.com/#/install?id=%e6%96%b9%e5%bc%8f%e4%ba%8c%ef%bc%9a%e6%89%8b%e5%8a%a8%e5%ae%89%e8%a3%85)

```
 git clone https://ghfast.top/github.com/wzpan/wukong-robot.git
```

```
git clone  https://ghfast.top/wzpan/wukong-contrib.git contrib
```

### 2.音频配置

#### 2.1 音箱Y11配置

```
alsamixer -c 1 # 或者是1
使用方向键调节音量
```

![image-20250714094431230](https://raw.githubusercontent.com/i37532/PicGo_PictureHome/main/Windows/image-20250714094431230.png)

```
sudo -i
```

查看卡号

```
arecord -l
```

[配置 .asoundrc](https://wukong.hahack.com/#/run?id=配置-asoundrc)

首先创建 `~/.asoundrc` ：

```
vim ~/.asoundrc
```



```
pcm.!default {
        type plug slave {
                pcm "hw:0,0"
        }
}

ctl.!default {
        type hw
        card 0
}
```

测试：

```
arecord test.wav
aplay test.wav
```

### 3.配置文件

第一次运行之后，创建配置文件，打开文件

```
vim ~/.wukong/config.yml
```

```
:set fileencoding=utf-8
:w
```

#### 3.1 配置唤醒

使用`snowboy`

环境嘈杂度：35

录音：5

#### 3.2 配置ASR

使用本地模型：`fun-asr`

模型下载：[链接](https://modelscope.cn/models/iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-pytorch/files)

```
funasr-export ++model=paraformer ++quantize=true ++device=cpu
```

```
from funasr import AutoModel

model = AutoModel(model="paraformer", device="cpu")

res = model.export(quantize=True)
```

```
/root/.cache/modelscope/hub/models/iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-pytorch
```

#### 3.2 配置NLU

使用`open-ai`

GPT免费api-key

https://github.com/chatanywhere/GPT_API_free?tab=readme-ov-file#%E5%A6%82%E4%BD%95%E4%BD%BF%E7%94%A8

api_key:

```
xxxxxxxxxxxxxxxxxxxxxxxxxxxx
```

api_base:

```
https://api.chatanywhere.tech/v1/chat/
```

### 运行

```
sudo -i
conda activate py39
vim /root/.wukong/custom/data_publisher.py
python /opt/wukong-robot/wukong.py
```

### 关闭

```
Ctrl+\
```

![image-20250722111458077](https://raw.githubusercontent.com/i37532/PicGo_PictureHome/main/Windows/image-20250722111458077.png)

```
sudo netstat -tulnp | grep LISTEN
sudo kill -9 <PID>  # 端口5001 对应的
```






### 5.`AI.py`,`car_control.py`: <br>
提示词，ai根据模板生成对应代码。
