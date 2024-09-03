import sys

class ImuData(object):
    __slots__ = ['stamp','acc','gyro','quat']
    def __init__(self):
        self.stamp = 0
        self.acc = [0. for x in range(0, 3)]
        self.gyro = [0. for x in range(0, 3)]
        self.quat = [0. for x in range(0, 4)]