#!/usr/bin/env python3

import zmq
import time
import sys

# 配置变量
PUBLISHER_IP = "192.168.110.203"
SUBSCRIBER_IP = "192.168.111.119"
PUB_PORT = "5556"
REP_PORT = "5557"

class DataPublisher:
    def __init__(self, pub_ip=PUBLISHER_IP, pub_port=PUB_PORT, rep_ip=SUBSCRIBER_IP, rep_port=REP_PORT):
        self.context = zmq.Context()
        pub_address = f"tcp://{pub_ip}:{pub_port}"
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(pub_address)
        print(f"数据发布器启动中，绑定到 {pub_address}")

        # REQ套接字连接到订阅者的REP端口
        rep_address = f"tcp://{rep_ip}:{rep_port}"
        self.req_socket = self.context.socket(zmq.REQ)
        self.req_socket.connect(rep_address)
        print(f"数据发布器连接到应答端口 {rep_address}")

    def send_message(self, message):
        time.sleep(1.5)  # 等待订阅者连接
        self.socket.send_string(message)
        print("已发送：", message)

        self.req_socket.send_string("你好，订阅者！")
        reply = self.req_socket.recv_string()
        print("收到订阅者回复：", reply)
        if reply:
            print("订阅者回复成功！停止发送")
            self.close()  # 只关闭 socket
            return True
            
    def close(self):
        self.socket.close()
        self.req_socket.close()
        self.context.term()
        print("ZeroMQ 资源已释放，发布器已关闭。")
        
if __name__ == "__main__":
    publisher = DataPublisher()
    while True:
        message = "status 跑路通知:我不干了"
        if publisher.send_message(message):
            break
