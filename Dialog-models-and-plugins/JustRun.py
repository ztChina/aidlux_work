# -*- coding: utf-8-*-
#!/usr/bin/env python3

from robot.sdk.AbstractPlugin import AbstractPlugin
from data_publisher import get_publisher

class Plugin(AbstractPlugin):

    def handle(self, text, parsed):
        self.say('不干了，不干啦，我要跑路啦，世界那么大，我想去看看', cache=True)
        publisher = get_publisher()
        while True:
            message = "status 跑路通知:我不干了"
            if publisher.send_message(message):
                break

    def isValid(self, text, parsed):
        return "跑路" in text
