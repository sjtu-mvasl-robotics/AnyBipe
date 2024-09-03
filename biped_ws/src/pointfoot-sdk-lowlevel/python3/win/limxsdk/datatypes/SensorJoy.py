import sys

class SensorJoy(object):
    __slots__ = ['stamp','axes','buttons']
    def __init__(self):
        self.stamp = 0
        self.axes = []
        self.buttons = []