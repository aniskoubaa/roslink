__package__ = 'messages'

import json

class GPSRawInfo(object):
    def __init__(self, header=None, time_usec=None, fix_type=None, lat=None, lon=None, alt=None, eph=None, epv=None, vel=None, cog=None, satellites_visible = None ):
        self.header = header 
        self.time_usec = time_usec
        self.fix_type = fix_type
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.eph = eph
        self.epv = epv
        self.vel = vel
        self.cog = cog
        self.satellites_visible = satellites_visible

    def from_json(self, message):
        self.__dict__ = json.loads(message)
        
    def printMessage(self):
        print 'time_usec: ', self.time_usec
        print 'fix_type: ', self.fix_type
        print 'lat: ', self.lat
        print 'lon: ', self.lon
        print 'alt: ', self.alt
        print 'eph: ', self.eph
        print 'epv: ', self.epv
        print 'vel: ', self.vel
        print 'cog: ', self.cog
        print 'satellites_visible: ', self.satellites_visible

    