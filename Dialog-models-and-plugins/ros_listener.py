#!/usr/bin/env python3

import zmq
import threading

# 配置变量
PUBLISHER_IP = "192.168.110.203"
SUBSCRIBER_IP = "192.168.111.119"
PUB_PORT = "5556"
REP_PORT = "5557"

class ROSListener:
    def __init__(self, publisher_ip=PUBLISHER_IP, pub_port=PUB_PORT, rep_port=REP_PORT, topic="status"):
        self.context = zmq.Context()
        address = f"tcp://{publisher_ip}:{pub_port}"
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        print(f"ROS监听器已连接到 {address}，订阅主题 '{topic}'")

        # REP套接字绑定到订阅者IP
        rep_bind_address = f"tcp://{SUBSCRIBER_IP}:{rep_port}"
        self.rep_socket = self.context.socket(zmq.REP)
        self.rep_socket.bind(rep_bind_address)
        print(f"ROS监听器应答端口绑定到 {rep_bind_address}")

    def listen(self):
        def sub_thread():
            print("订阅线程已启动")
            while True:
                message = self.socket.recv_string()
                print("收到数据：", message)

        def rep_thread():
            print("应答线程已启动")
            while True:
                req = self.rep_socket.recv_string()
                print("收到请求：", req)
                self.rep_socket.send_string("已收到你的消息！")
                
        threading.Thread(target=sub_thread, daemon=True).start()
        threading.Thread(target=rep_thread, daemon=True).start()
        while True:
            pass  # 保持主线程运行

if __name__ == "__main__":
    listener = ROSListener()
    listener.listen()
