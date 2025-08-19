#!/usr/bin/env python3
import serial
import socket

# 打开串口
ser = serial.Serial(
    port='/dev/ttyUSB0',    # 串口名称，根据实际情况可能是 ttyUSB0, ttyS0 等
    baudrate=9600,        # STM32 端的波特率，与你代码中的配置保持一致
    timeout=1               # 超时时间，单位为秒
)

print("开始监听串口 /dev/ttyUSB0...")

def send_wakeup_signal():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(('127.0.0.1', 6000))
    s.send(b"WAKEUP")
    s.close()

try:
    while True:
        if ser.in_waiting:
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"[STM32] 收到: {data}")
            # 假设湿度数据格式为 "HUM:xx.xx"
            if data.startswith("HUM:"):
                hum = float(data.split(":")[1])
                if hum > 80:  # 这里以湿度大于80为例
                    send_wakeup_signal()
            elif "Hello" in data:
                send_wakeup_signal()
                # 这里可以添加唤醒后的处理逻辑
except KeyboardInterrupt:
    print("用户中断，退出程序。")
finally:
    ser.close()
