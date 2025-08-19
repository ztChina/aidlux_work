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


### 5.`AI.py`,`car_control.py`: <br>
提示词，ai根据模板生成对应代码。
