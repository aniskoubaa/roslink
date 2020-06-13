__package__ = 'messages'

import json

class SetMode (object):
    def __init__ (self, header, flightMode):
        self.header = header
        self. flightMode = flightMode