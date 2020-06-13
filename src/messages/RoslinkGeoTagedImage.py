__package__ = 'messages'

import json

class RoslinkGeoTagedImage(object):
    def __init__ (self, header=None,owner_id=None, robot_key=None, data=None, height=None,  width=None, lat=None, lon=None, alt=None, camera_pitch=None, camera_roll=None, camera_yaw=None):
        self.header = header
        self.owner_id = owner_id
        self.key = robot_key
        self.data = data
        self.height = height
        self.width = width
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.camera_pitch = camera_pitch
        self.camera_roll = camera_roll
        self.camera_yaw = camera_yaw
        
    def from_json(self, message):
        self.__dict__ = json.loads(message)

        
    def printMessage(self):
        print 'header: ', self.header
        print 'owner_id: ', self.owner_id
        print 'key: ' , self.key
        print 'height: ' , self.height
        print 'width: ' , self.width
        print 'lat: ', self.lat
        print 'lon: ', self.lon
        print 'alt: ', self.alt
        print 'camera_pitch: ', self.camera_pitch 
        print 'camera_roll: ', self.camera_roll 
        print 'camera_yaw: ', self.camera_yaw
        

        
            
