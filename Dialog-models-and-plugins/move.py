#!/usr/bin/env python3
# -*- coding: utf-8-*-

import re
from robot.sdk import unit
from robot.sdk.AbstractPlugin import AbstractPlugin
from data_publisher import get_publisher

class Plugin(AbstractPlugin):

    def handle(self, text, parsed):
        slots = unit.getSlots(parsed, 'DIRECTION_DISTANCE')  # 取出所有词槽
        # 遍历词槽，找出 user_person 对应的值
        for slot in slots:
            if slot['name'] == 'user_move':
                for slot2 in slots:
                    if slot2['name'] == 'user_move_distance':
                            self.say('好的，向{}移动{}！'.format(slot['normalized_word'],slot2['normalized_word']))
                            # 处理方向和距离
                            direction = slot['normalized_word']  # 方向
                            distance = slot2['normalized_word']  # 距离
                            # 1. 方向转换为大写英文
                            if '前' in direction:
                                direction = 'F'
                            elif '后' in direction:
                                direction = 'B'
                            elif '左' in direction:
                                direction = 'L'
                            elif '右' in direction:
                                direction = 'R'
                            # 2. 提取数字并转为int
                            match = re.match(r'([0-9.]+)', slot2['normalized_word'])
                            if match:
                                distance = int(float(match.group(1)))
                            else:
                                distance = 0
                            # 3. 发送消息
                            publisher = get_publisher()
                            while True:
                                message = f"status 移动通知:方向：{direction} 距离：{distance}"
                                print(f"发送消息: {message}")
                                if publisher.send_message(message):
                                    break
                            return
        # 如果没命中词槽，说 hello world
        self.say('不好依稀，没听懂哦', cache=True)

    def isValid(self, text, parsed):
        # 判断是否包含 HELLO_WORLD 意图
        return unit.hasIntent(parsed, 'DIRECTION_DISTANCE')

