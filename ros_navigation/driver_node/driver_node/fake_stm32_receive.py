import serial
import time
import struct

def parse_data_frame(frame):
    if len(frame) < 5:
        return None, []

    length = frame[2]
    func_code = frame[3]
    data_len = length - 2  # 去掉func_code(1) + checksum(1)
    float_count = data_len // 4

    floats = []
    for i in range(float_count):
        start = 4 + i * 4
        float_bytes = frame[start:start+4]
        value = struct.unpack('<f', float_bytes)[0]
        floats.append(value)

    return func_code, floats

def parse_frames(data_buffer):
    frames = []
    i = 0
    while i + 4 < len(data_buffer):
        if data_buffer[i] == 0xAA and data_buffer[i+1] == 0x55:
            length = data_buffer[i+2]
            frame_len = 3 + length
            if i + frame_len <= len(data_buffer):
                frame = data_buffer[i:i+frame_len]
                # 可在此做校验，比如校验和
                frames.append(frame)
                i += frame_len
            else:
                break
        else:
            i += 1
    return frames, data_buffer[i:]

def main():
    ser = serial.Serial('/dev/pts/21', 115200, timeout=0.1)
    buffer = bytearray()

    try:
        while True:
            data = ser.read(ser.in_waiting or 1)
            if data:
                buffer.extend(data)

                frames, buffer = parse_frames(buffer)
                if frames:
                    latest_frame = frames[-1]
                    # 这里处理最新一帧，比如打印
                    print(f"最新一帧({len(latest_frame)}字节): {latest_frame.hex()}")
                    func_code, float_values = parse_data_frame(latest_frame)
                    print(f"功能码: 0x{func_code:02X}")
                    for i, val in enumerate(float_values):
                        print(f"Float[{i}]: {val:.4f}")
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

if __name__ == '__main__':
    main()
