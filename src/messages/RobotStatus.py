__package__ = 'messages'

import json

class RobotStatus(object):
    def __init__(self, header=None, state=None, onboard_control_sensors_present=None, onboard_control_sensors_enabled=None, voltage_battery=None, current_battery=None, battery_remaining=None):
        self.header = header
        self.state = state
        self.onboard_control_sensors_present = onboard_control_sensors_present # [] boolean
        self.onboard_control_sensors_enabled = onboard_control_sensors_enabled # [] boolean
        self.voltage_battery = voltage_battery
        self.current_battery = current_battery
        self.battery_remaining = battery_remaining
        
    def from_json(self, message):
        self.__dict__ = json.loads(message)
        
    def printMessage(self):
        print 'onboard_control_sensors_present: ', self.onboard_control_sensors_present
        print 'onboard_control_sensors_enabled: ' , self.onboard_control_sensors_enabled
        print 'voltage_battery: ' , self.voltage_battery
        print 'current_battery: ' , self.current_battery
        print 'battery_remaining: ' , self.battery_remaining
        
        
