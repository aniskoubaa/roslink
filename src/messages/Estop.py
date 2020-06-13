__package__ = 'messages'

import json

class Estop(object):
    def __init__ (self, header):
        self.header = header
        
    def from_json(self, message):
        self.__dict__ = json.loads(message)

        

            
