#!/usr/bin/env python
'''
Created on Jun 24, 2016

@author: maram
'''
import json
import socket
from enums.MESSAGE_TYPE import MESSAGE_TYPE
from messages.ROSLinkHeader import ROSLinkHeader 
from messages.Takeoff import Takeoff
from messages.Land import Land
#from messages.Move import Move
import time




robot_list = ['drone', 'turtlebot']
drone_commands_list = ['Take off', 'Land', 'Emergency/Reset' , 'Move', 'Set mode', 'Arm']
turtlebot_commands_list = ['Move', 'Go To waypoint']


server_address = ("192.168.100.17", 25500)
user_command = {} # to store user command
#CMD = ['message_id', 'name'] # ROSLink command data structure
TwistCMD =['vx', 'vy', 'vz', 'wx', 'wy', 'wz']
GO_TO_WAYPOINT_COMMAND = ['x', 'y', 'z', 'frame_type']
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

seq_num = 0

def cunstructTwistCOMMAND(COMMAND):
    
    for i in xrange(len(TwistCMD)):
        user_command[TwistCMD[i]]= COMMAND[i]

def cunstructGoToWaypointCOMMAND(COMMAND):
    
    for i in xrange(len(GO_TO_WAYPOINT_COMMAND)):
        user_command[GO_TO_WAYPOINT_COMMAND[i]]= COMMAND[i]

# switch functionality
class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration
    
    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args: 
            self.fall = True
            return True
        else:
            return False
        
##########################        
def cunstructMessageHeader(message_id):
    global seq_num
    header = ROSLinkHeader(1, 8, 0, message_id, seq_num, "1243-0000-0000-FGFG" )
    seq_num += 1
    return header.__dict__        
##########################

           
        
#############################
        
#if __name__ == '__main__':
def control_station():    
    print 'choose your robot:'
    counter = 0
        #print the available commands for the user
    for robot in robot_list:
        print " %d == %s " % (counter , robot )
        counter+=1
        
    userRobot = input()
    
    if (userRobot == 0 ):
        print 'The available commands:', drone_commands_list
        while True:
            counter = 0
            #print the available commands for the user
            for command in drone_commands_list:
                print " %d == Command: %s " % (counter , command )
                counter+=1
                
                
            print "Enter command number: "
                
            userInput = input()
            
            for case in switch(userInput):
                if case(0):
                    header = cunstructMessageHeader(MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_TAKEOFF)
                    #message_id = 102
                    altitude = 2
                    CMD_TAKEOFF = Takeoff(header, altitude)
                    #return CMD_TAKEOFF.__dict__
                    #COMMAND = [0, 'TAKE_OFF']
                    #cunstructCOMMAND(COMMAND)
                    s.sendto(json.dumps(CMD_TAKEOFF.__dict__), server_address)
                    break
                if case(1):
                    header = cunstructMessageHeader(MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_LAND)
                    #COMMAND = [0,'LAND']
                    #cunstructCOMMAND(COMMAND)
                    CMD_LAND = Land(header)
                    s.sendto(json.dumps(CMD_LAND.__dict__), server_address)
                    break
                if case(2):
                    header = cunstructMessageHeader(MESSAGE_TYPE.ROSLINK_MESSAGE_MOVE)
                    vx = input('Enter linear velocity in x-axis ')
                    vy = input('Enter linear velocity in y-axis ')
                    vz = input('Enter linear velocity in z-axis ')
                    wx = input('Enter Angular velocity in x-axis ')
                    wy = input('Enter Angular velocity in y-axis ')
                    wz = input('Enter Angular velocity in z-axis ')
                    T_CMD=[vx, vy, vz, wx, wy, wz]
                    cunstructTwistCOMMAND(T_CMD)
                    print json.dumps(user_command)
                    s.sendto(json.dumps(user_command), server_address)
                    break
                print "Please enter valid command!"  
                break 
        
    elif (userRobot == 1 ):
        
        print 'The available commands:', turtlebot_commands_list
        while True:
            counter = 0
            #print the available commands for the user
            for command in turtlebot_commands_list:
                print " %d == Command: %s " % (counter , command )
                counter+=1
                
                
            print "Enter command number: "
                
            userInput = input()
            
            for case in switch(userInput):
                if case(0):
                    header = cunstructMessageHeader(MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_TWIST)	
                    user_command['header']=header	  
                    vx = input('Enter linear x ')
                    vz = input('Enter angular z ')
                    T_CMD=[vx, 0, vz, 0, 0, 0]
                    cunstructTwistCOMMAND(T_CMD)
                    print json.dumps(user_command)
                    count = 10
                    while (count > 0):	
                    	#print count	
                    	s.sendto(json.dumps(user_command), server_address)
                    	time.sleep (1.0)
                    	count = count - 1
                    break
                if case(1):
                    header = cunstructMessageHeader(MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_GO_TO_WAYPOINT)	
                    user_command['header']=header	  
                    x = input('Enter coordinate x ')
                    y = input('Enter coordinate y ')
                    z = input('Enter coordinate z ')
                    f = input('Enter 0 for global world frame, 1 for robot frame ')
                    if f == 0:
                      Go_CMD=[x, y, z, 'false']
                    if f == 1:
                      Go_CMD=[x, y, z, 'true']
                    cunstructGoToWaypointCOMMAND(Go_CMD)
                    print json.dumps(user_command)
                    s.sendto(json.dumps(user_command), server_address)
                    break
                print "Please enter valid command!"  
                break 
        
                
                
if __name__ == '__main__':
    control_station() 
