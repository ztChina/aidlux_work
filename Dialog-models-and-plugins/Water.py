# -*- coding: utf-8-*-
from robot.sdk.AbstractPlugin import AbstractPlugin

class Plugin(AbstractPlugin):

    def handle(self, text, parsed):
        self.say('Water,I need water,you asshole, come on, hurry up please!', cache=True)

    def isValid(self, text, parsed):
        return "how are you" in text

