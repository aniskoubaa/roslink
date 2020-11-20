#!/usr/bin/env python

import websocket
from websocket import create_connection , WebSocket # pip install websocket-client
import threading
import datetime
import socket
import threading
import sys
import time
import json
import rospy
import tf
import rospkg
# from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import Image, CameraInfo, CompressedImage

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped

from std_msgs.msg import String
import os
import cv2
import numpy as np
import base64
from cv_bridge import CvBridge, CvBridgeError


# import ROSLink constants 
from enums.ROSLINK_VERSION import ROSLINK_VERSION
from enums.ROS_VERSION import ROS_VERSION
from enums.ROBOT_TYPE import ROBOT_TYPE
from enums.ROBOT_STATE import ROBOT_STATE
from enums.MESSAGE_TYPE import MESSAGE_TYPE
from enums.ROBOT_MODE import ROBOT_MODE


# import ROSLink messages
from messages.ROSLinkHeader import ROSLinkHeader 
from messages.HeartBeat import HeartBeat
from messages.RobotStatus import RobotStatus
from messages.GlobalMotion import GlobalMotion
from messages.GPSRawInfo import GPSRawInfo
from messages.RangeFinderData import RangeFinderData
from messages.ROSLinkImage import ROSLinkImage
from messages.ROSLinkMap import ROSLinkMap


# import configuration parameters
from configuration.CONFIG import *
from configuration.ROSLINK_MESSAGE_PERIOD import *

# import state variables 
from states.ROSLinkStateVariables import ROSLinkStateVariables

debug=False
ENCODING = 'utf-8'
COMPRESSION_RATE=35 #100 is full quality 
POSE_TOPIC="/amcl_pose"
CAMERA_TOPIC="/usb_cam/image_raw/compressed"
#CAMERA_TOPIC="/camera/rgb/image_raw/compressed"
CMD_VEL_TOPIC="/cmd_vel"
    
class ROSLinkBridgeTB3:
        
    '''''''''''''''''''''''''''''''''''''''''''''
    Represents ROSLink Bridge for AR Drone
    '''''''''''''''''''''''''''''''''''''''''''''
    def __init__(self):
        '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        Constructor: read and initialize robot and roslink bridge parameters
        '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        
        # init the parameters from launch file
        ROSLinkBridgeTB3.init_params()
        
        if (ROSLinkStateVariables.key == "NOKEY"):
            print("ROBOT KEY HAS NOT BEEN SET. PROGRAM WILL EXIT. SET YOUR ROBOT KEY BEFORE STARTING THE ROSLINK BRIDGE")
            sys.exit()
        if (ROSLinkStateVariables.owner_id == -1):
            print("OWNER ID NOT BEEN SET. PROGRAM WILL EXIT. SET YOUR OWNER ID BEFORE STARTING THE ROSLINK BRIDGE")
            sys.exit()
        # initialize ROS node for this client
        rospy.init_node('roslink_bridge_ardrone_node'+ROSLinkStateVariables.key, anonymous=True) 
        
        #start ROS publishers
        ROSLinkBridgeTB3.start_ros_publishers()
        
        #start ROS publishers
        ROSLinkBridgeTB3.start_ros_subscribers()
        
        #start UDP server socket
        ROSLinkBridgeTB3.init_servers() 
        
        #create threads for sending roslink messages
        ROSLinkBridgeTB3.create_roslink_message_threads()
        
        #create threads for processing received roslink command messages
        ROSLinkBridgeTB3.create_roslink_command_processing_thread()
    
        #sping ROS forever
        rospy.spin()
    
    @staticmethod
    def init_servers():
        ROSLinkBridgeTB3.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #create server socket
        ROSLinkBridgeTB3.client_socket.settimeout(2) #set a timeout on a blocking socket operation
        #ROSLinkBridgeTB3.gcs_server_ip = rospy.get_param("/ground_station_ip", "127.0.0.1")
        #ROSLinkBridgeTB3.gcs_server_ip = rospy.get_param("/ground_station_ip", "192.168.100.17")
        #ROSLinkBridgeTB3.gcs_server_port =rospy.get_param("/ground_station_port", 10000)
        ROSLinkBridgeTB3.gcs_server_ip = rospy.get_param("/ground_station_ip", "127.0.0.1")
        ROSLinkBridgeTB3.gcs_server_port =rospy.get_param("/ground_station_port", 25500)
        ROSLinkBridgeTB3.server_address  = ( ROSLinkBridgeTB3.gcs_server_ip, ROSLinkBridgeTB3.gcs_server_port)
        print (ROSLinkBridgeTB3.gcs_server_ip)
        print (ROSLinkBridgeTB3.gcs_server_port)
        
        ROSLinkStateVariables.ws_timer = 0
        ROSLinkBridgeTB3.WSconnected=False
        ROSLinkBridgeTB3.ws = None;

        
        t = threading.Thread(target=ROSLinkBridgeTB3.websocketThread)
        t.setName("websocket thread")
        t.start()
       
    
    


    @staticmethod
    def websocketThread():
        if(not ROSLinkBridgeTB3.WSconnected):
            if (ROSLinkBridgeTB3.gcs_server_ip=='localhost'):
                #for a local network
                WS_url = "ws://"+ ROSLinkBridgeTB3.gcs_server_ip+":9090/";
            else:
                # this for public network
                WS_url = "ws://"+ ROSLinkBridgeTB3.gcs_server_ip+":9090/websockets/roslink/user/"+str(ROSLinkStateVariables.owner_id) +"/robot/"+ROSLinkStateVariables.key+"/robot" 
            WS_url = "ws://"+ ROSLinkBridgeTB3.gcs_server_ip+":9090/websockets/roslink/user/"+str(ROSLinkStateVariables.owner_id) +"/robot/"+ROSLinkStateVariables.key+"/robot" 
            ROSLinkBridgeTB3.ws = websocket.WebSocketApp(WS_url,
                                on_message = ROSLinkBridgeTB3.on_message,
                                on_error = ROSLinkBridgeTB3.on_error,
                                on_close = ROSLinkBridgeTB3.on_close)
            ROSLinkBridgeTB3.ws.on_open = ROSLinkBridgeTB3.on_open
            # ROSLinkBridgeTB3.ws.settimeout(10)
            
            # ROSLinkBridgeTB3.WSconnected=True;
            ROSLinkBridgeTB3.ws.run_forever(ping_interval=13, ping_timeout=10)


    @staticmethod
    def handle_websocket_closing():
        print("***************reconnecting to WS server*****************")
        t = threading.Thread(target=ROSLinkBridgeTB3.websocketThread)
        t.setName("websocket thread")
        t.start()



    

    @staticmethod
    def on_close(ws):   
        ROSLinkStateVariables.WSconnected =False;
        print("[ROSLink Bridge:TB3] websocket connection closed");
        ROSLinkStateVariables.ws_timer+=1;
        print("[ROSLink Bridge:TB3] try to reconnect after",ROSLinkStateVariables.ws_timer)
        time.sleep(ROSLinkStateVariables.ws_timer)
        ROSLinkBridgeTB3.handle_websocket_closing();
        for f in ROSLinkBridgeTB3.logfiles:
            f.close()


    @staticmethod
    def on_open(ws):
        ROSLinkStateVariables.ws_timer=0;
        print("websocket connection opened");
        time.sleep(1) # to make sure no thread is sending now
        ROSLinkStateVariables.WSconnected =True;


    @staticmethod
    def on_message(ws, message):
        ROSLinkBridgeTB3.process_roslink_command_message(message)
        # print("ws command")



    @staticmethod
    def on_error(ws, error):   
        print("[ROSLink Bridge:TB3] websocket error ", error)



    @staticmethod
    def init_params():   
        rospy.loginfo('[ROSLink Bridge:TB3] reading initialization parameters')
        # get roslink version
        ROSLinkStateVariables.roslink_version = rospy.get_param("/roslink_version", ROSLINK_VERSION.ABUBAKR)  
        # get ROS version  
        ROSLinkStateVariables.ros_version = rospy.get_param("/ros_version", ROS_VERSION.INDIGO)    
        # get system id
        ROSLinkStateVariables.system_id = rospy.get_param("/system_id", 12)
        # get robot name
        ROSLinkStateVariables.robot_name = rospy.get_param("/robot_name", "RIA_E100")
        # get robot type
        ROSLinkStateVariables.type = rospy.get_param("/type", ROBOT_TYPE.ROBOT_TYPE_RIA_E100)
        # get owner id
        ROSLinkStateVariables.owner_id = rospy.get_param("/owner_id", "26")
        # get key
        ROSLinkStateVariables.key = str(rospy.get_param("/key", "ABCD"))
        #get map data
        ROSLinkStateVariables.map_location = rospy.get_param("/map_location", "ABCD")
        ROSLinkStateVariables.map_xoffset = rospy.get_param("/map_xoffset", 0)
        ROSLinkStateVariables.map_yoffset = rospy.get_param("/map_yoffset", 0)
        ROSLinkStateVariables.map_height = rospy.get_param("/map_height", 0)
        ROSLinkStateVariables.map_width = rospy.get_param("/map_width", 0)



        ROSLinkStateVariables.WSconnected= False;
        
        # define periods of updates
        ROSLinkBridgeTB3.heartbeat_msg_rate = rospy.get_param("/heartbeat_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_HEARTBEAT_MESSAGE_RATE)
        ROSLinkBridgeTB3.robot_status_msg_rate = rospy.get_param("/robot_status_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_ROBOT_STATUS_MESSAGE_RATE)
        ROSLinkBridgeTB3.global_motion_msg_rate = rospy.get_param("/global_motion_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_GLOBAL_MOTION_MESSAGE_RATE)
        ROSLinkBridgeTB3.gps_raw_info_msg_rate = rospy.get_param("/gps_raw_info_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_GPS_RAW_INFO_MESSAGE_RATE)
        ROSLinkBridgeTB3.range_finder_data_msg_rate = rospy.get_param("/range_finder_data_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_RANGE_FINDER_DATA_MESSAGE_RATE)
        ROSLinkBridgeTB3.ROSLing_Image_msg_rate = rospy.get_param("/image_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_IMAGE_MESSAGE_RATE)
    
        ROSLinkBridgeTB3.TwistCommand = Twist()
        ROSLinkBridgeTB3.sendingTwistFlag=True
        ROSLinkBridgeTB3.bridge = CvBridge()
        ROSLinkBridgeTB3.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        ROSLinkBridgeTB3.lastimagetime =0

        rospack = rospkg.RosPack()
        rospackpath= rospack.get_path('roslink') + "/logs/"
        print(rospackpath)
        ROSLinkBridgeTB3.logfiles={}
        ts=str(datetime.datetime.now())
        #print(ts)
        f_ROSLINK_MESSAGE_HEARTBEAT = open("ROSLINK_MESSAGE_HEARTBEAT_"+ts+".csv","w+")
        f_ROSLINK_MESSAGE_ROBOT_STATUS = open("ROSLINK_MESSAGE_ROBOT_STATUS_"+ts+".csv","w+")
        f_ROSLINK_MESSAGE_GLOBAL_MOTION = open("ROSLINK_MESSAGE_GLOBAL_MOTION_"+ts+".csv","w+")
        f_ROSLINK_MESSAGE_GPS_RAW_INFO = open("ROSLINK_MESSAGE_GPS_RAW_INFO_"+ts+".csv","w+")
        f_ROSLINK_MESSAGE_RANGE_FINDER_DATA = open("ROSLINK_MESSAGE_RANGE_FINDER_DATA_"+ts+".csv","w+")
        f_ROSLINK_MESSAGE_ROSLINK_IMAGE = open("ROSLINK_MESSAGE_ROSLINK_IMAGE_"+ts+".csv","w+")
        f_ROSLINK_MESSAGE_ROSLINK_MAP   = open("ROSLINK_MESSAGE_ROSLINK_MAP_"+ts+".csv","w+")
        f_ROSLINK_MESSAGE_ROSLINK_GEO_TAGED_IMAGE = open("ROSLINK_MESSAGE_ROSLINK_GEO_TAGED_IMAGE_"+ts+".csv","w+")

        ROSLinkBridgeTB3.logfiles.update( {ROSLinkStateVariables.ROSLINK_MESSAGE_HEARTBEAT:f_ROSLINK_MESSAGE_HEARTBEAT} )
        ROSLinkBridgeTB3.logfiles.update( {ROSLinkStateVariables.ROSLINK_MESSAGE_ROBOT_STATUS:f_ROSLINK_MESSAGE_ROBOT_STATUS} )
        ROSLinkBridgeTB3.logfiles.update( {ROSLinkStateVariables.ROSLINK_MESSAGE_GLOBAL_MOTION:f_ROSLINK_MESSAGE_GLOBAL_MOTION} )
        ROSLinkBridgeTB3.logfiles.update( {ROSLinkStateVariables.ROSLINK_MESSAGE_GPS_RAW_INFO:f_ROSLINK_MESSAGE_GPS_RAW_INFO} )
        ROSLinkBridgeTB3.logfiles.update( {ROSLinkStateVariables.ROSLINK_MESSAGE_RANGE_FINDER_DATA:f_ROSLINK_MESSAGE_RANGE_FINDER_DATA} )
        ROSLinkBridgeTB3.logfiles.update( {ROSLinkStateVariables.ROSLINK_MESSAGE_ROSLINK_IMAGE:f_ROSLINK_MESSAGE_ROSLINK_IMAGE} )
        ROSLinkBridgeTB3.logfiles.update( {ROSLinkStateVariables.ROSLINK_MESSAGE_ROSLINK_MAP:f_ROSLINK_MESSAGE_ROSLINK_MAP} )
        ROSLinkBridgeTB3.logfiles.update( {ROSLinkStateVariables.ROSLINK_MESSAGE_ROSLINK_GEO_TAGED_IMAGE:f_ROSLINK_MESSAGE_ROSLINK_GEO_TAGED_IMAGE} )        


    #start ROS publishers 
    @staticmethod
    def start_ros_publishers():
        # ROS publishers: for executing ROSLink commands
        ROSLinkBridgeTB3.move_publisher = rospy.Publisher(CMD_VEL_TOPIC,Twist, queue_size=10)    

    #start ROS subscribers
    @staticmethod
    def start_ros_subscribers():
        # rospy.Subscriber("/odom", Odometry, ROSLinkBridgeTB3.odometryCallback)
        rospy.Subscriber(POSE_TOPIC, PoseWithCovarianceStamped, ROSLinkBridgeTB3.amclCallback)
        # rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, ROSLinkBridgeTB3.compressedImageCallback)
        rospy.Subscriber(CAMERA_TOPIC, CompressedImage, ROSLinkBridgeTB3.compressedImageCallback)

    
    @staticmethod
    def frontImageCallback(data):
        data = str(data)
        if debug:
            print('----------------------',data[0:5])
        if debug:
            print(len(data))
        if(len(data)<65000):
            ROSLinkStateVariables.ROSLinkImageData=data[7:len(data)-1]
            return;
        else:
            ROSLinkStateVariables.ROSLinkImageData=data[:65000]
            print('\n\n\n image too long it will be cut \n\n\n')

        if debug:
            print(len(ROSLinkStateVariables.ROSLinkImageData))
        # cv2.imshow("ARDrone Front Image Viewer", cv_image)
        # cv2.waitKey(3)

    @staticmethod   
    def compressedImageCallback(data):
        try:
            
          #cv_image = ROSLinkBridgeARDrone.bridge.imgmsg_to_cv2(data, "bgr8")
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            frame = cv_image
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), COMPRESSION_RATE]
            result, encoded_img = cv2.imencode('.jpg', frame, encode_param)
            # encoded = cv2.imencode('.png',frame)[1]
            # cv2.imshow("df",frame)
            #print(encimg.shape)
            #data = base64.encodestring(encimg)
            data = base64.b64encode(encoded_img)
            data = data.decode(ENCODING)
            #print(data)
            #data = base64.encodebytes(encoded_img)
            # print("len(data)")
        except CvBridgeError as e:
          print(e)
        # data =  u''+base64.encodestring(np_arr)
        #if(debug):
        #print(len(data))
        ROSLinkStateVariables.ROSLinkImageData=data
       
        
    @staticmethod   
    def amclCallback(msg):
        #position 
        ROSLinkStateVariables.x= msg.pose.pose.position.x
        ROSLinkStateVariables.y= msg.pose.pose.position.y
        ROSLinkStateVariables.z= msg.pose.pose.position.z

        ROSLinkStateVariables.time_boot_ms=time.time()
        
        # print(ROSLinkStateVariables.x,ROSLinkStateVariables.y,"x,y")
   

    @staticmethod   
    def odometryCallback(msg):
        #position 
        ROSLinkStateVariables.time_boot_ms=time.time()
        ROSLinkStateVariables.x= msg.pose.pose.position.x
        ROSLinkStateVariables.y= msg.pose.pose.position.y
        ROSLinkStateVariables.z= msg.pose.pose.position.z
        # print(ROSLinkStateVariables.x,ROSLinkStateVariables.y,"x,y")
        #orientation
        quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        ROSLinkStateVariables.roll = euler[0]
        ROSLinkStateVariables.pitch = euler[1]
        ROSLinkStateVariables.yaw = euler[2]
        #twist: linear
        ROSLinkStateVariables.vx = msg.twist.twist.linear.x
        ROSLinkStateVariables.vy = msg.twist.twist.linear.y
        ROSLinkStateVariables.vz = msg.twist.twist.linear.z
        #twist: angular
        ROSLinkBridgeTB3.wx = msg.twist.twist.angular.x
        ROSLinkStateVariables.wy = msg.twist.twist.angular.y
        ROSLinkStateVariables.wz = msg.twist.twist.angular.z
             
    
    #this method create threads for sending roslink message at specific rate
    @staticmethod
    def create_roslink_message_threads(): 
        ROSLinkBridgeTB3.heatrbeat_message_thread = ROSLinkMessageThread(ROSLinkBridgeTB3.client_socket, ROSLinkBridgeTB3.ws, ROSLinkBridgeTB3.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT, "heatrbeat_message_thread", ROSLinkBridgeTB3.heartbeat_msg_rate)
        ROSLinkBridgeTB3.robot_status_thread = ROSLinkMessageThread(ROSLinkBridgeTB3.client_socket, ROSLinkBridgeTB3.ws, ROSLinkBridgeTB3.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS, "robot_status_thread", ROSLinkBridgeTB3.robot_status_msg_rate)
        ROSLinkBridgeTB3.global_motion_thread = ROSLinkMessageThread(ROSLinkBridgeTB3.client_socket, ROSLinkBridgeTB3.ws, ROSLinkBridgeTB3.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION, "global_motion_thread", ROSLinkBridgeTB3.global_motion_msg_rate)
        ROSLinkBridgeTB3.ROSLinkImage_message_thread = ROSLinkMessageThread(ROSLinkBridgeTB3.client_socket, ROSLinkBridgeTB3.ws,  ROSLinkBridgeTB3.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_IMAGE, "ROSLink_image_thread", ROSLinkBridgeTB3.ROSLing_Image_msg_rate)
        ROSLinkBridgeTB3.TwistCommandThread = TwistCommandThread()



# ROSLinkBridgeTB3.gps_raw_info_thread = ROSLinkMessageThread(ROSLinkBridgeTB3.client_socket,  ROSLinkBridgeTB3.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO, "gps_raw_info_thread", ROSLinkBridgeTB3.gps_raw_info_msg_rate)
        #ROSLinkBridgeTB3.range_finder_data_thread = ROSLinkMessageThread(ROSLinkBridgeTB3.client_socket, ROSLinkBridgeTB3.server_address, "range_finder_data_thread", 0.333)
        
    @staticmethod
    def create_roslink_command_processing_thread(): 
        ROSLinkBridgeTB3.command_processing_thread =  ROSLinkCommandProcessingThread(ROSLinkBridgeTB3.client_socket, 'ROSLink Command Processing Thread')
    
    @staticmethod
    def static_build_roslink_header_message(message_id):
        header = ROSLinkHeader(ROSLinkStateVariables.roslink_version, ROSLinkStateVariables.ros_version, ROSLinkStateVariables.system_id,ROSLinkStateVariables.owner_id, message_id, ROSLinkStateVariables.sequence_numbers[message_id],ROSLinkStateVariables.key)
        ROSLinkStateVariables.sequence_numbers[message_id] = ROSLinkStateVariables.sequence_numbers[message_id] + 1
        ROSLinkStateVariables.messages_time_recorder[message_id].update( {ROSLinkStateVariables.sequence_numbers[message_id]:time.time()} )

        if (ROSLinkStateVariables.sequence_numbers[message_id]>ROSLinkStateVariables.max_sequence):
            ROSLinkStateVariables.sequence_numbers[message_id]=1

        return header.__dict__
    
    @staticmethod 
    def static_build_heartbeat_message():
        
        message_header = ROSLinkBridgeTB3.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT)
        heartbeat_message = HeartBeat(message_header, ROBOT_TYPE.ROBOT_TYPE_RIA_E100, ROSLinkStateVariables.robot_name, ROBOT_STATE.ROBOT_STATE_ACTIVE, ROSLinkStateVariables.owner_id ,ROBOT_MODE.ROBOT_STATE_UNKNOWN)
        return heartbeat_message.__dict__
    
    @staticmethod 
    def static_build_robot_status_message():
        message_header = ROSLinkBridgeTB3.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS)
        robot_status_message = HeartBeat(message_header, 0, ROSLinkStateVariables.robot_name, 0, 0 ,0)
        return robot_status_message.__dict__
    
    @staticmethod
    def static_build_global_motion_message():
        message_header = ROSLinkBridgeTB3.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION) 
        global_motion_message = GlobalMotion(message_header, ROSLinkStateVariables.time_boot_ms , ROSLinkStateVariables.x, ROSLinkStateVariables.y, ROSLinkStateVariables.yaw, ROSLinkStateVariables.vx, ROSLinkStateVariables.vy, ROSLinkStateVariables.vz, ROSLinkStateVariables.wx, ROSLinkStateVariables.wy, ROSLinkStateVariables.wz, ROSLinkStateVariables.pitch, ROSLinkStateVariables.roll, ROSLinkStateVariables.yaw)
        return global_motion_message.__dict__  
    
    @staticmethod
    def static_build_gps_raw_info_message():
        message_header = ROSLinkBridgeTB3.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO) 
        global_motion_message = GPSRawInfo(message_header, ROSLinkStateVariables.time_boot_ms , ROSLinkStateVariables.fix_type, ROSLinkStateVariables.lat, ROSLinkStateVariables.lon, ROSLinkStateVariables.alt, ROSLinkStateVariables.eph, ROSLinkStateVariables.epv, ROSLinkStateVariables.vel, ROSLinkStateVariables.cog, ROSLinkStateVariables.satellites_visible)
        return global_motion_message.__dict__  

    @staticmethod
    def static_build_ROSLink_image_message():
        # print("fps",1/(time.time()-ROSLinkBridgeTB3.lastimagetime))
        # ROSLinkBridgeTB3.lastimagetime = time.time()
        message_header = ROSLinkBridgeTB3.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_IMAGE) 
        ROSLink_image_message = ROSLinkImage(message_header, ROSLinkStateVariables.owner_id ,ROSLinkStateVariables.key,ROSLinkStateVariables.ROSLinkImageData,ROSLinkStateVariables.ROSLinkImageHeight,ROSLinkStateVariables.ROSLinkImageWidth)
        return ROSLink_image_message.__dict__ 
    
    @staticmethod
    def move_to_goal(xGoal,yGoal):
        ROSLinkBridgeTB3.sendingTwistFlag=False;
        #define a client for to send goal requests to the move_base server through a SimpleActionClient
        # ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #wait for the action server to come up
        while(not ROSLinkBridgeTB3.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
               rospy.loginfo("Waiting for the move_base action server to come up")

        goal = MoveBaseGoal()


        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal*/

        goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal location ...")
        ROSLinkBridgeTB3.ac.send_goal(goal)

        ROSLinkBridgeTB3.ac.wait_for_result(rospy.Duration(60))

        if(ROSLinkBridgeTB3.ac.get_state() ==  GoalStatus.SUCCEEDED):
               rospy.loginfo("You have reached the destination")
               ROSLinkBridgeTB3.sendingTwistFlag = True

        else:
               rospy.loginfo("The robot failed to reach the destination")
               ROSLinkBridgeTB3.sendingTwistFlag = True
        ROSLinkBridgeTB3.sendingTwistFlag = True
        


    @staticmethod
    def process_roslink_command_message(msg):
        # print 'msg is ', msg 
        
        command = json.loads(msg)
        if(debug):
            print ('ROSLink command received ..')
        # print msg
        # if command['header']['key'] != ROSLinkStateVariables.key:
        #     return
        if command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_TWIST:  
            # print 'Twist command received'
            # print(msg)
            
            #print('Move robot: vx=',command['vx'],'wz=',command['wz'])
            ROSLinkBridgeTB3.TwistCommand.linear.x = command['vx']
            ROSLinkBridgeTB3.TwistCommand.linear.y = command['vy'] 
            ROSLinkBridgeTB3.TwistCommand.linear.z = command['vz'] 
            ROSLinkBridgeTB3.TwistCommand.angular.x = command['wx']
            ROSLinkBridgeTB3.TwistCommand.angular.y = command['wy'] 
            ROSLinkBridgeTB3.TwistCommand.angular.z = command['wz']             
            # print TwistCommand
            # ROSLinkBridgeTB3.move_publisher.publish (ROSLinkBridgeTB3.TwistCommand)
            
        elif command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_GET_MAP:
            print ('get map command received --------------------------')
            # print ROSLinkBridgeTB3.sendingTwistFlag
            # TwistCommandThread()
            try:
                Map = cv2.imread(ROSLinkStateVariables.map_location,0);

                frame = Map
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
                result, encimg = cv2.imencode('.jpg', frame, encode_param)

                # encoded = cv2.imencode('.png',Map)[1]
                # file = open(ROSLinkStateVariables.map_location, 'rb')
                # data = base64.b64encode(file.read())
                
                #data = base64.encodestring(encimg)
                data = base64.b64encode(encimg)
                data = data.decode(ENCODING)

                # data = u''+base64.encodestring(encoded);
                print("map size: ",len(data),"path: ", ROSLinkStateVariables.map_location)
                
                message_header = ROSLinkBridgeTB3.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_MAP)
                map_message = ROSLinkMap(message_header, ROSLinkStateVariables.owner_id ,  ROSLinkStateVariables.key, data,ROSLinkStateVariables.map_height,ROSLinkStateVariables.map_width,ROSLinkStateVariables.map_xoffset,ROSLinkStateVariables.map_yoffset,None,None,None)
                ROSLinkMap_Object = json.dumps(map_message.__dict__)

                # ROSLinkBridgeTB3.client_socket.sendto(ROSLinkMap,ROSLinkBridgeTB3.server_address)
                ROSLinkBridgeTB3.ws.send(ROSLinkMap_Object)
                return;
            except Exception as e:
                print(e)
                pass


        elif command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_GO_TO_WAYPOINT:
            print ('GO TO WAYPOINT command received --------------------------')
            print(msg)
            print(command)
            print(command['x'])
            print(command["y"])
            
            # os.system('rosrun roslink gotogaolnode.py')
            # ROSLinkBridgeTB3.move_to_goal(command['x'],command['y'])
            move_to_goalThread(command['x'],command['y'])
            print("goal sent to thread")
            return

        elif command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_ESTOP:
            ROSLinkBridgeTB3.sendingTwistFlag=True
            print("Estop")
            ROSLinkBridgeTB3.TwistCommand.linear.x = 0
            ROSLinkBridgeTB3.TwistCommand.linear.y = 0 
            ROSLinkBridgeTB3.TwistCommand.linear.z = 0 
            ROSLinkBridgeTB3.TwistCommand.angular.x = 0
            ROSLinkBridgeTB3.TwistCommand.angular.y = 0 
            ROSLinkBridgeTB3.TwistCommand.angular.z = 0
            # print(ROSLinkBridgeTB3.sendingTwistFlag)   
            ROSLinkBridgeTB3.ac.cancelAllGoals()
        elif command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_ACKNOWLEDGMENT:
            # print( str(command["message_id"])+",")
            #s = "\n"+str(command["message_id"])+","+str(command["sequence_number"])+","+str((time.time()-ROSLinkStateVariables.messages_time_recorder[command["message_id"]][command["sequence_number"]])/2)
            #print(s)
            ROSLinkBridgeTB3.logfiles[command["message_id"]].write(s)
            # print("acknowledgemnt received")
            # print("time needed to send "+str(command["message_id"])+" message #"+str(command["sequence_number"])+" is:"+str((time.time()-ROSLinkStateVariables.messages_time_recorder[command["message_id"]][command["sequence_number"]])/2))
            

class move_to_goalThread():
    """docstring for command"""
    def __init__(self,xGoal,yGoal):
        self.xGoal = xGoal
        self.yGoal = yGoal
        t = threading.Thread(target=self.run)
        t.setName("move_to_goalThread")
        t.start()

    def run ( self ):
        ROSLinkBridgeTB3.sendingTwistFlag=False;
        #define a client for to send goal requests to the move_base server through a SimpleActionClient
        # ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #wait for the action server to come up
        while(not ROSLinkBridgeTB3.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
               rospy.loginfo("Waiting for the move_base action server to come up")

        goal = MoveBaseGoal()


        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal*/

        goal.target_pose.pose.position =  Point(self.xGoal,self.yGoal,0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal location ...")
        ROSLinkBridgeTB3.ac.send_goal(goal)

        ROSLinkBridgeTB3.ac.wait_for_result(rospy.Duration(60))

        if(ROSLinkBridgeTB3.ac.get_state() ==  GoalStatus.SUCCEEDED):
               rospy.loginfo("You have reached the destination")
               ROSLinkBridgeTB3.sendingTwistFlag = True

        else:
               rospy.loginfo("The robot failed to reach the destination")
               ROSLinkBridgeTB3.sendingTwistFlag = True
        ROSLinkBridgeTB3.sendingTwistFlag = True


class TwistCommandThread():
    """docstring for command"""
    def __init__(self):
        t = threading.Thread(target=self.run)
        t.setName("TwistCommandThread")
        t.start()

    def run ( self ):
        print(ROSLinkBridgeTB3.sendingTwistFlag,"threading")
        while True:#ROSLinkBridgeTB3.sendingTwistFlag:
            time.sleep(0.051)
            # print "ROSLinkBridgeTB3.TwistCommand"
            if(ROSLinkBridgeTB3.sendingTwistFlag):
                ROSLinkBridgeTB3.move_publisher.publish (ROSLinkBridgeTB3.TwistCommand)


class ROSLinkMessageThread ():
    count = 0
    def __init__(self,  sock, WS,server_address,message_type ,thread_name='noname', data_rate=1.0):
        self.count = self.count +1
        self.name = thread_name
        self.socket = sock
        self.server_address = server_address
        self.data_rate = data_rate
        self.roslink_message_type = message_type
        self.WS = WS

        t = threading.Thread(target=self.run)
        t.setName(thread_name)
        t.start()
    
    def run ( self ):
        print(self.name, " started")
        while True:
            self.count=self.count+1
            time.sleep(1.0/self.data_rate)
            if(debug):
                print(self.roslink_message_type,"sent")
            # print 'thread %s %d\n'%(self.name, self.count)
            #self.send(self.socket, json.dumps(self.roslink_message.__dict__))
            try:
                if (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT):
                    if (ROSLinkStateVariables.WSconnected):
                        self.send(self.socket, json.dumps(ROSLinkBridgeTB3.static_build_heartbeat_message()))
                        self.WS.send(json.dumps(ROSLinkBridgeTB3.static_build_heartbeat_message()))
                        #print('heartbeat_message')
                    else:
                        self.send(self.socket, json.dumps(ROSLinkBridgeTB3.static_build_heartbeat_message()))
                        self.WS = ROSLinkBridgeTB3.ws;

                elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS):
                    if (ROSLinkStateVariables.WSconnected):
                        self.WS.send(json.dumps(ROSLinkBridgeTB3.static_build_robot_status_message()))
                    else:
                        self.send(self.socket, json.dumps(ROSLinkBridgeTB3.static_build_robot_status_message()))
                        self.WS = ROSLinkBridgeTB3.ws;
                
                elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION):
                    if (ROSLinkStateVariables.WSconnected):
                        self.WS.send(json.dumps(ROSLinkBridgeTB3.static_build_global_motion_message()))
                    else:
                        self.send(self.socket, json.dumps(ROSLinkBridgeTB3.static_build_global_motion_message()))
                        self.WS = ROSLinkBridgeTB3.ws;
                
                elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO):
                    if (ROSLinkStateVariables.WSconnected):
                        self.WS.send(json.dumps(ROSLinkBridgeTB3.static_build_gps_raw_info_message()))
                    else:
                        self.send(self.socket, json.dumps(ROSLinkBridgeTB3.static_build_gps_raw_info_message()))
                        self.WS = ROSLinkBridgeTB3.ws;

                elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_IMAGE):
                    if (ROSLinkStateVariables.WSconnected):
                        image_message = json.dumps(ROSLinkBridgeTB3.static_build_ROSLink_image_message())
                        self.WS.send(image_message)

                        #print ('ROSLink_image sent ws', len(image_message))
                        # print(ROSLinkStateVariables.sequence_numbers[MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_IMAGE])

                    else:
                        # self.send(self.socket, json.dumps(ROSLinkBridgeTB3.static_build_ROSLink_image_message()))
                        # print ('ROSLink_image sent udp')
                        self.WS = ROSLinkBridgeTB3.ws;
            except Exception as e:
                print(e)
                print (self.name)
            

                
                #ROSLinkStateVariables.ROSLinkImageSendingCounter+=1;
                

    '''
        Sending method
    '''
    def send (self, sock , msg): 
        #for Python 3 use: self.socket.sendto(msg.encode(), self.server_address)
        #for Python 2 use: self.socket.sendto(msg, self.server_address)
        #print(msg.encode())
        #+++print(msg)
        self.socket.sendto(msg.encode(), self.server_address)   




class ROSLinkCommandProcessingThread ( ):
    def __init__(self, sock,thread_name='noname'):
        self.name = thread_name
        self.socket = sock
        t = threading.Thread(target=self.run)
        t.setName(thread_name)
        t.start()
    def run ( self):
        print ("Start ROSLINK Command Processing Thread")
        while not rospy.is_shutdown():
            try:
                msg, address = self.socket.recvfrom(MESSAGE_MAX_LENGTH)
                ROSLinkBridgeTB3.process_roslink_command_message(msg)
            except socket.timeout:
                continue   


if __name__ == '__main__':
    print ('\n************** Starting ROSLink Bridge for Turtlebot Burger **************\n' )
    # initialize ROS node for this client
    myDroneBridge = ROSLinkBridgeTB3() 