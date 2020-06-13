__package__ = 'messages'

import json

class HeartBeat(object):
    def __init__ (self, header=None, robot_type=None, name=None, system_status=None, owner_id=None, mode=None):
        self.header = header
        self.type = robot_type
        self.name = name
        self.system_status = system_status
        self.owner_id = owner_id
        self.mode = mode
        
    def from_json(self, message):
        self.__dict__ = json.loads(message)

        
    def printMessage(self):
        print 'type: ', self.type
        print 'name: ' , self.name
        print 'system_status: ' , self.system_status
        print 'owner_id: ' , self.owner_id
        print 'mode: ' , self.mode
            
