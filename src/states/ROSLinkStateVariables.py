__package__ = 'states'
from enums.MESSAGE_TYPE import MESSAGE_TYPE


class ROSLinkStateVariables:
    
    '''
        Class variables
    '''
    key='0000-0000-0000-0000'
    sequence_number=0
    ROSLINK_MESSAGE_HEARTBEAT           = 0
    ROSLINK_MESSAGE_ROBOT_STATUS        = 1
    ROSLINK_MESSAGE_GLOBAL_MOTION       = 2
    ROSLINK_MESSAGE_GPS_RAW_INFO        = 3
    ROSLINK_MESSAGE_RANGE_FINDER_DATA   = 4
    ROSLINK_MESSAGE_ROSLINK_IMAGE       = 5
    ROSLINK_MESSAGE_ROSLINK_MAP         = 6
    ROSLINK_MESSAGE_ROSLINK_GEO_TAGED_IMAGE       = 7

    ROSLINK_MESSAGE_COMMAND_TWIST           =100
    ROSLINK_MESSAGE_COMMAND_GO_TO_WAYPOINT  = 101
    ROSLINK_MESSAGE_COMMAND_TAKEOFF         = 102
    ROSLINK_MESSAGE_COMMAND_LAND            = 103
    ROSLINK_MESSAGE_COMMAND_ARM            = 104
    ROSLINK_MESSAGE_COMMAND_DISARM            = 105
    ROSLINK_MESSAGE_COMMAND_SET_MODE            = 106
    ROSLINK_MESSAGE_COMMAND_GET_MAP         = 107
    ROSLINK_MESSAGE_COMMAND_ESTOP               = 108



    # MAVlink messages 
    MAVLINK_MESSAGE_HEARTBEAT           = 0;
    MAVLINK_GPS_RAW_INT                 = 24;
    sequence_numbers={
    MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT:0,
    MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS:0,
    MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION:0,
    MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO:0,
    MESSAGE_TYPE.ROSLINK_MESSAGE_RANGE_FINDER_DATA:0,
    MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_IMAGE:0,
    MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_MAP:0,
    MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_GEO_TAGED_IMAGE:0,
    }
    
    messages_time_recorder={
    MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT:{},
    MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS:{},
    MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION:{},
    MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO:{},
    MESSAGE_TYPE.ROSLINK_MESSAGE_RANGE_FINDER_DATA:{},
    MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_IMAGE:{},
    MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_MAP:{},
    MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_GEO_TAGED_IMAGE:{},
    }
    sequence_frames_time={}
    type = 0
    robot_name = 'noname'
    system_status = 0
    owner_id = 0
    mode = 0
    max_sequence = 15000
    #roslink_version = 0 
    ros_version = 0
    system_id = 0
    message_id = 0
    time_boot_ms=0.0
    seq_number =0;
    vx = 0.0
    vy = 0.0
    vz = 0.0
    wx = 0.0
    wy = 0.0
    wz = 0.0
    battery = 0.0
    state = 0.0
    magX = 0.0
    magY = 0.0
    magZ = 0.0
    pressure = 0.0
    temp = 0.0
    wind_speed = 0.0
    wind_angle = 0.0
    rotX = 0.0
    rotY = 0.0
    rotZ = 0.0
    altitude = 0.0
    motor1 = 0.0
    motor2 = 0.0
    motor3 = 0.0
    motor4 = 0.0
    tags_count = 0.0
    tags_type = 0.0
    


    ROSLinkImageData=""
    ROSLinkImageHeight=240
    ROSLinkImageWidth=320
    

    x= 0.0
    y= 0.0
    z= 0.0
    roll =0.0
    pitch = 0.0
    yaw = 0.0
    vx_truth = 0.0
    vy_truth = 0.0
    vz_truth = 0.0
    wx_truth = 0.0
    wy_truth = 0.0
    wz_truth = 0.0
    
    
    
    fix_type = 0
    lat = 0.0
    lon = 0.0
    alt = 0.0
    eph = 0.0
    epv = 0.0
    vel = 0.0
    cog = 0.0
    satellites_visible = 0.0

    map_location= 'non'



