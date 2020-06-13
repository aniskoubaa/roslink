__package__ = 'messages'

import json

class Takeoff (object):
    def __init__ (self, header, altitude):
        self.header = header
        self. altitude = altitude