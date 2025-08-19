#!/usr/bin/env python3

import serial

# 打开串口
ser = serial.Serial(
    port='/dev/ttyUSB1',    # 串口名称，根据实际情况可能是 ttyUSB0, ttyS0 等
    baudrate=115200,        # STM32 端的波特率，与你代码中的配置保持一致
    timeout=1               # 超时时间，单位为秒
)

print("开始监听串口 /dev/ttyUSB0...")

try:
    while True:
        if ser.in_waiting:
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"[STM32] 收到: {data}")
except KeyboardInterrupt:
    print("用户中断，退出程序。")
finally:
    ser.close()

