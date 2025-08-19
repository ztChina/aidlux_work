#!/usr/bin/env python3
import socket

def send_wakeup_signal():
    """向本地端口 6000 发送唤醒信号"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('127.0.0.1', 6000))
        s.send(b"WAKEUP")
        s.close()
        print("[INFO] 已发送 WAKEUP 信号")
    except ConnectionRefusedError:
        print("[ERROR] 无法连接到 127.0.0.1:6000，请确保监听服务正在运行。")

print("请输入信息（如输入 HUM:85 或 WAKEUP），Ctrl+C 退出。")

try:
    while True:
        data = input("[输入] > ").strip()
        if not data:
            continue

        print(f"[用户] 收到: {data}")

        if data.startswith("HUM:"):
            try:
                hum = float(data.split(":")[1])
                if hum > 80:
                    send_wakeup_signal()
            except ValueError:
                print("[WARN] 无效的湿度值格式，应为 HUM:xx.xx")
        elif "WAKEUP" in data:
            send_wakeup_signal()

except KeyboardInterrupt:
    print("\n用户中断，退出程序。")

