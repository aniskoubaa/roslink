__package__ = 'messages'

import json

class GlobalMotion(object):
    def __init__(self, header=None, time_boot_ms=None, x=None, y=None, z=None, vx=None, vy=None, vz=None,  wx=None, wy=None, wz=None, pitch=None, roll=None, heading=None):
        self.header = header 
        self.time_boot_ms = time_boot_ms
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.wx = wx
        self.wy = wy
        self.wz = wz
        self.pitch = pitch
        self.roll = roll
        self.heading = heading
	

    def from_json(self, message):
        self.__dict__ = json.loads(message)
        
    def printMessage(self):
        print ('time_boot_ms: ', self.time_boot_ms)
        print ('x: ', self.x)
        print ('y: ', self.y)
        print ('z: ', self.z)
        print ('vx: ', self.vx)
        print ('vy: ', self.vy)
        print ('vz: ', self.vz)
        print ('wx: ', self.wx)
        print ('wy: ', self.wy)
        print ('wz: ', self.wz)
        print ('pitch: ', self.pitch)
        print ('roll: ', self.roll)
        print ('heading: ', self.heading)
