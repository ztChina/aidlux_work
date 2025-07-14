#!/usr/bin/env python3

# Real-time speech recognition from a microphone with sherpa-onnx Python API
# with endpoint detection.
#
# Please refer to
# https://k2-fsa.github.io/sherpa/onnx/pretrained_models/index.html
# to download pre-trained models

import argparse
import sys
from pathlib import Path
from Levenshtein import distance
from unidecode import unidecode
try:
    import sounddevice as sd
except ImportError:
    print("Please install sounddevice first. You can use")
    print()
    print("  pip install sounddevice")
    print()
    print("to install it")
    sys.exit(-1)

import sherpa_onnx

#####################################参数部分
encoder_path = "../model/encoder-epoch-12-avg-2-chunk-16-left-64.onnx"
decoder_path = "../model/decoder-epoch-12-avg-2-chunk-16-left-64.onnx"
joiner_path  = "../model/joiner-epoch-12-avg-2-chunk-16-left-64.onnx"
tokens_path = "../model/tokens.txt"

# all_pinyins = set([
#     "a", "ai", "an", "ang", "ao",
#     "ba", "bai", "ban", "bang", "bao", "bei", "ben", "beng", "bi", "bian", "biao", "bie", "bin", "bing", "bo", "bu",
#     "ca", "cai", "can", "cang", "cao", "ce", "cen", "ceng", "cha", "chai", "chan", "chang", "chao", "che", "chen", "cheng", "chi", "chong", "chou", "chu", "chua", "chuai", "chuan", "chuang", "chui", "chun", "chuo",
#     # …… 所有拼音都加进去，这里只是示例，完整需要 400+ 个
#     "xiao", "xue", "de", "tong", "shi", "xian", "xun", "xue", "xiong"
# ])      #所有可能拼音，用于拼音分解
all_pinyins = set([
    "a", "ai",
    "xiao", "xue", "de", "tong", "shi", "xue"
])      #简化拼音
target_seq = ["xiao", "ai", "tong", "xue"] #目标拼音：小爱同学
#######################################函数部分
def create_recognizer():
    """
    创建一个识别器
    """
    recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
        encoder=encoder_path,
        decoder=decoder_path,
        joiner=joiner_path,
        tokens=tokens_path,
        num_threads=1,
        sample_rate=16000,
        feature_dim=80,
        enable_endpoint_detection=True, # 开启端点检测，用来识别语音结束
        rule1_min_trailing_silence=2.4,
        rule2_min_trailing_silence=1.2,
        rule3_min_utterance_length=300,  # it essentially disables this rule
    )
    return recognizer

def split_pinyin(text, pinyin_list):
    """
    #!使用已有拼音模块匹配分割拼音
    """
    # 用 unidecode 去掉拼音的声调
    text = unidecode(text).replace(" ", "")

    result = []
    i = 0
    while i < len(text):
        found = False
        # 最长音节最长 5 个字母 (e.g., zhuang)
        for l in range(5, 0, -1):
            if i + l <= len(text):
                seg = text[i:i + l]
                if seg in pinyin_list:
                    result.append(seg)
                    i += l
                    found = True
                    break
        if not found:
            # 没匹配到，说明错误，可选择丢弃或记录未知
            result.append(text[i])
            i += 1
    return result
#######################################主函数部分
def main():
    #1、确定设备
    devices = sd.query_devices()
    if len(devices) == 0:
        print("No microphone devices found")
        sys.exit(0)
    # 这里写你想用的名字
    target_device_name = "AB13X"

    # 用字符串匹配找出 index
    target_index = None
    for idx, device in enumerate(devices):
        if target_device_name in device["name"]:
            target_index = idx
            break

    if target_index is None:
        print(f"Cannot find device with name containing '{target_device_name}'")
        sys.exit(1)

    print(f"Use device: {devices[target_index]['name']} (index: {target_index})")

    # 修改默认设备
    sd.default.device = (target_index, target_index)

    #2、创建识别器
    recognizer = create_recognizer()
    print("Started! Please speak")

    #3、获取音频流
    sample_rate = 48000
    samples_per_read = int(0.1 * sample_rate)  # 0.1 second = 100 ms

    stream = recognizer.create_stream()

    with sd.InputStream(channels=1, dtype="float32", samplerate=sample_rate) as s:
        while True:
            samples, _ = s.read(samples_per_read)  # a blocking read
            samples = samples.reshape(-1)
            stream.accept_waveform(sample_rate, samples)
            while recognizer.is_ready(stream):
                recognizer.decode_stream(stream)

            is_endpoint = recognizer.is_endpoint(stream)

            result = recognizer.get_result(stream)
            if is_endpoint:
                if result:
                    print(result)
                    #4、检测是否为小爱同学
                    #预测识别与拼音解析
                    pred_seq = split_pinyin(result, all_pinyins)
                    print(pred_seq)
                    #计算与正确拼音的距离
                    dist = distance(" ".join(pred_seq), " ".join(target_seq))
                    print(f"编辑距离: {dist}")
                    if dist <= 3:
                        print("是小爱同学")
                    else:
                        print("不是")
                recognizer.reset(stream)


if __name__ == "__main__":

    try:
        main()
    except KeyboardInterrupt:
        print("\nCaught Ctrl + C. Exiting")