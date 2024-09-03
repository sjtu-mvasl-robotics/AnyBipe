import sys

class DiagnosticValue(object):
    __slots__ = ['stamp','level','name','code','message']
    def __init__(self):
        self.stamp = 0
        self.level = 0
        self.name = ''
        self.code = 0
        self.message = ''
