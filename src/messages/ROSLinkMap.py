__package__ = 'messages'

import json

class ROSLinkMap(object):
    def __init__ (self, header=None,owner_id=None, robot_key=None, data=None, height=None,  width=None, xoffset=None, yoffset=None, occupied_thresh=None, free_thresh=None, resolution=None ):
        self.header = header
        self.owner_id = owner_id
        self.key = robot_key
        self.data = data
        self.height = height
        self.width = width
        self.xoffset = xoffset
        self.yoffset = yoffset
        self.occupied_thresh= occupied_thresh
        self.free_thresh= free_thresh
        self.resolution= resolution
        
    def from_json(self, message):
        self.__dict__ = json.loads(message)

        
    def printMessage(self):
        print 'header: ', self.header
        print 'owner_id: ', self.owner_id
        print 'key: ' , self.key
        print 'height: ' , self.height
        print 'width: ' , self.width
        print "xoffset: ", self.xoffset
        print "yoffset: ", self.yoffset
        print "occupied_thresh: ", self.occupied_thresh
        print "resolution: ", self.resolution
            
