import serial
import struct
import time
import random

def send_sensor_data():
    ser = serial.Serial('/dev/pts/5', 115200)

    func_code = 0x01

    while True:
        # 模拟动态数据（不要每次都发一样的帧）
        left = random.uniform(0.9, 1.1)
        right = random.uniform(1.1, 1.3)
        temp = random.uniform(20.0, 26.0)
        humi = random.uniform(35.0, 45.0)
        water = random.uniform(0.5, 1.0)
        data = [left, right, temp, humi, water]

        frame = build_frame(func_code, data)
        ser.write(frame)

        time.sleep(0.05)  # 建议0.5秒或更慢，避免堆积

def build_frame(func_code, data):
    frame = bytearray()
    frame.append(0xAA)
    frame.append(0x55)
    length = 1 + 4 * len(data) + 1
    frame.append(length)
    frame.append(func_code)
    for val in data:
        frame.extend(struct.pack('<f', val))
    checksum = 0
    for b in frame[2:]:
        checksum ^= b
    frame.append(checksum)
    return frame

if __name__ == '__main__':
    send_sensor_data()