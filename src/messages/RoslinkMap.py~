__package__ = 'messages'

import json

class ROSLinkImage(object):
    def __init__ (self, header=None,owner_id=None, robot_key=None, data=None, height=None,  width=None):
        self.header = header
        self.owner_id = owner_id
        self.key = robot_key
        self.data = data
        self.height = height
        self.width = width
        
    def from_json(self, message):
        self.__dict__ = json.loads(message)

        
    def printMessage(self):
        print 'header: ', self.header
        print 'owner_id: ', self.owner_id
        print 'key: ' , self.key
        print 'height: ' , self.height
        print 'width: ' , self.width
        
            
