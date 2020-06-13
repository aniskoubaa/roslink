__package__ = 'messages'

import json

class RangeFinderData(object):
    def __init__(self, header=None, time_usec=None, angle_min=None, angle_max=None, angle_increment=None, time_increment=None, scan_time=None, range_min=None, range_max=None, ranges=None, intensities=None):
        self.header = header
        self.time_usec = time_usec
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        self.time_increment = time_increment
        self.scan_time = scan_time
        self.range_min = range_min
        self.range_max = range_max
        self.ranges = ranges
        self.intensities = intensities

    def from_json(self, message):
        self.__dict__ = json.loads(message)
        
    def printMessage(self):
        print 'time_usec: ' , self.time_usec
        print 'angle_min: ' , self.angle_min
        print 'angle_max: ' , self.angle_max
        print 'angle_increment: ' , self.angle_increment 
        print 'time_increment: ' , self.time_increment 
        print 'scan_time: ' , self.scan_time 
        print 'range_min: ' , self.range_min 
        print 'range_max: ' , self.range_max 
        print 'ranges: ' , self.ranges 
        print 'intensities: ' , self.intensities 