#!/usr/bin/env python2

NODE_NAME = "GantryNode"

import rospy
import numpy as np
import ctypes

from threading import Thread

from can_msgs.msg import Frame
from geometry_msgs.msg import Pose, Twist, TransformStamped
from nav_msgs.msg import Odometry

import tf2_ros
import tf_transformations

from Controls import PIDController, LPController
from Utils import Vector

class numhex64(ctypes.Union):
    _fields_ = [("num", ctypes.c_double),
                ("sint", ctypes.c_int64),
                ("uint", ctypes.c_uint64),
                ("hex", ctypes.c_ubyte * 8)]

class numhex32(ctypes.Union):
    _fields_ = [("num", ctypes.c_float),
                ("sint", ctypes.c_int32),
                ("uint", ctypes.c_uint32),
                ("hex", ctypes.c_ubyte * 4)]

class GantryNode:
    """
    This node interfaces with the ODrive boards and attached encoders, to drive the gantry at either a desired velocity or to a desired position

    Reports gantry position and pendant status at some frequency

    positive x: towards door
        positive y: towards machine wall

        y   [machine wall]
        ^  ______
        | |gantry|  [door]
        | |______|
        |-----> x

    """
    DATA_LENGTH = {
        'double':8,
        'int32':4,
        'bool':1,
        'bool3':3
    }


    # Experimental data:
    #     id (hex)  data_length
    # ['1'   '0x1'   '8'] < double - control
    # ['2'   '0x2'   '8'] < double - control
    # [''    '0x4'   '4']
    # ['16'  '0x10'  '3'] < bool3 - status
    # ['17'  '0x11'  '8'] < double
    # ['18'  '0x12'  '8'] < double
    # ['19'  '0x13'  '8'] < double
    # ['32'  '0x20'  '1'] < bool1 - status
    # ['33'  '0x21'  '8'] < double
    # ['257' '0x101' '4'] < int32
    # ['258' '0x102' '4'] < int32
# baffling
# 1074790400 when moved in x-
# 3222274048 when moved in x+

# 1074266112 when moved in y+
# 3221749760 when moved in y-

    # ['259' '0x103' '4'] < int32
    # ['260' '0x104' '4'] < int32
    # ['261' '0x105' '4'] < int32
    # ['273' '0x111' '4'] < int32
    # ['274' '0x112' '4'] < int32
    # ['275' '0x113' '4'] < int32
    # ['276' '0x114' '4'] < int32
    # ['277' '0x115' '4'] < int32
    # ['290' '0x122' '4'] < int32
    # ['291' '0x123' '4'] < int32
    # ['292' '0x124' '4'] < int32
    # ['293' '0x125' '4'] < int32
    # message types: 22

    NAME_TABLE = {
        # rad/sec
        'control_x_speed': [0x01, 'double'], # YES
        'control_y_speed': [0x02, 'double'], # YES
        
        'main_crane_driver_status': [0x10, 'bool3'], # LIKELY
        'sub_crane_driver_status': [0x20, 'double'], # LIKELY

        'x_axis_actual_control_speed_0': [0x11, 'double'],  # LIKELY
        'x_axis_actual_control_speed_1': [0x12, 'double'],  # LIKELY
        'y_axis_actual_control_speed': [0x13, 'double'],    # LIKELY
        
        'y_axis_actual_control_speed_mirror': [0x21, 'double'], # LIKELY

        # mm
        'laser_distance_0': [0x101, 'int32'],
        'laser_distance_1': [0x111, 'int32'],
        # 'laser_distance_2': [0x121, 'int32'], # no laser on the a axis

        # Counts, counts/sec (2048 counts = 1rev = 300mm)
        'encoder_ppr_0': [0x102, 'int32'],
        'encoder_inc_0': [0x103, 'int32'], # overflows at 4294967295
        'encoder_abs_0': [0x104, 'int32'],
        'encoder_speed_0': [0x105, 'int32'],
        
        'encoder_ppr_1': [0x112, 'int32'],
        'encoder_inc_1': [0x113, 'int32'],
        'encoder_abs_1': [0x114, 'int32'],
        'encoder_speed_1': [0x115, 'int32'],

        'encoder_ppr_2': [0x122, 'int32'],
        'encoder_inc_2': [0x123, 'int32'],
        'encoder_abs_2': [0x124, 'int32'],
        'encoder_speed_2': [0x125, 'int32'],


        # 'reset_encoder_0': [0x100, 'bool'],
        # 'reset_encoder_1': [0x200, 'bool'],
        # 'reset_encoder_2': [0x300, 'bool'],
    }
    ID_TABLE = {}
    CAN_STATE = {}

    
    RATE = 20 # hz

    def init_publishers(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.state_pub = rospy.Publisher(
            rospy.get_param("~/gantry_state_topic", "gantry/state"),
            Odometry, queue_size=1)

            # received by the CAN node on the gantry (which in turn speaks to the CAN arduino over serial via dark and unknowable magics)
        self.gantry_pub = rospy.Publisher(
            rospy.get_param('~/can_sent_messages','/sent_messages'),
            Frame, queue_size=1)

    def init_subscribers(self):
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(
            rospy.get_param("~/can_received_messages", "/received_messages"),
            Frame, self.can_message_callback, queue_size=10)

        rospy.Subscriber(
            rospy.get_param("~/gantry_velocity_set_topic", "gantry/velocity_set"),
            Twist, self.set_velocity_callback, queue_size=10)
        rospy.Subscriber(
            rospy.get_param("~/gantry_position_set_topic", "gantry/position_set"),
            Pose, self.set_position_callback, queue_size=10)
    
    def init_state(self):
        # copy NAME_TABLE into ID_TABLE, initialize CAN_STATE
        for key in self.NAME_TABLE.keys():
            id, can_type = self.NAME_TABLE[key]
            id = hex(id)
            self.ID_TABLE[id] = [key, can_type]
            self.CAN_STATE[key] = None
        
        
        self.velocity_controller = LPController(0.1)
        self.position_controller = PIDController(1.,0.001,0.001)

        self.target_velocity = [0,0]
        self.target_position = None

        self.velocity = [0,0]
        self.position = [0,0] # TODO: Get this from encoder data
        
        self.prev_time = rospy.get_rostime().to_sec() - 1.0/self.RATE

    def __init__(self):
        # Publishers
        self.init_publishers()

        # Subscribers
        self.init_subscribers()
        

        rospy.logwarn(NODE_NAME + " is initializing...")
        # Await lidar distance estimation
        while self.CAN_STATE['laser_distance_0'] == None or self.CAN_STATE['laser_distance_1'] == None:
            rospy.sleep(0.1)
        
        # Set approximate map -> base_link transform
        
        # Later: use limit switches to find precise y coordinate
        # Much Later: use limit switches to find precise x coordinate
        # go to home position
        

        rospy.logwarn(NODE_NAME + " is online")

        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            self.loop_callback()
            rate.sleep()

    def loop_callback(self):
        """
        Write velocity, do position control, and publish any updates
        """
        time = rospy.get_rostime().to_sec()
        delta_t = time - self.prev_time
        
        if self.target_velocity != None:
            velocity_control = self.velocity_controller.do_control(self.velocity, self.target_velocity, delta_t)
        else:
            velocity_control = self.position_controller.do_control(self.position, self.target_position, delta_t)

        self.write_velocity(velocity_control)
        # TODO: figure out unit conversions
        x_speed = self.CAN_STATE['x_axis_actual_control_speed_0']
        y_speed = self.CAN_STATE['y_axis_actual_control_speed']

        self.velocity = velocity_control # TODO: get this from sensor feedback
        self.position = [0,0]

        self.publish_state()


        if self.CAN_STATE['encoder_speed_0'] != None:
            val = self.CAN_STATE['encoder_speed_0']
            rospy.loginfo("encoder info: %s" % val)

        self.prev_time = time

    def set_velocity_callback(self, msg):
        """
        Set the target velocity of the gantry
        """
        
        self.target_velocity = Vector.to_array(msg.linear)[:2]
        self.target_position = None

        rospy.loginfo("Velocity set to x: %f rps, y: %f rps", self.target_velocity[0], self.target_velocity[1])
        
    def set_position_callback(self, msg):
        """
        Set the target position of the gantry
        """
        self.target_velocity = None
        self.target_position = Vector.to_array(msg.position)[:2]
        
        rospy.loginfo("Position set to x: %f rps, y: %f rps", self.target_position[0], self.target_position[1])

    def can_message_callback(self, msg):
        """
        Process the messages recieved from the CAN node. Store them in state
        """
        
        if not hex(msg.id) in self.ID_TABLE.keys():
            rospy.logwarn("unknown id: %s with length %s" % (hex(msg.id), msg.dlc))
            return
        
        # rospy.logwarn("received control_x_speed")

        key, can_type = self.ID_TABLE[hex(msg.id)]



        numhex = numhex64()
        for idx in range(msg.dlc):
            # if msg.dlc == 4:
            #     print(ord( msg.data[idx]))
            numhex.hex[idx] = ord(msg.data[idx]) # big endian
            # numhex.hex[-idx] = ord(msg.data[idx]) # little endian
            # numhex.hex[4 + idx] = ord(msg.data[idx]) # big endian offset
            # numhex.hex[4 - idx] = ord(msg.data[idx]) # little endian offset

        # if msg.dlc == 4:
        #     print([ord(c) for c in msg.data])
        #     rospy.loginfo("GAH")

        if can_type == 'int32':
            val = numhex.sint
        elif can_type == 'double':
            val = numhex.num
        else:
            val = numhex.sint != 0
        
        self.CAN_STATE[key] = [numhex.sint, numhex.num]
        return (key, numhex.sint, numhex.num)

    def send_can_frame(self, name, value, publish=True):
        """
        A reasonably generic way to send message to the CAN node that doesnt rot the mind
        """

        id, can_type = self.NAME_TABLE[name]
        length = self.DATA_LENGTH[can_type]
        
        msg = Frame()
        msg.is_rtr = False
        msg.is_extended = False
        msg.dlc = length
        msg.id = id
        msg.data = ""
        
        numhex = numhex64()
        if can_type == 'int32':
            numhex.sint = value
        elif can_type == 'double':
            numhex.num = value
        
        for idx in range(length):
            msg.data += chr(numhex.hex[idx])
        
        msg.header.stamp = rospy.get_rostime()
        if publish: self.gantry_pub.publish(msg)

        return msg

    def write_velocity(self, velocity):
        """
        Send velocity commands to the gantry motors
        """
        self.send_can_frame('control_x_speed', velocity[0])
        self.send_can_frame('control_y_speed', velocity[1])

    def publish_state(self):
        """
        Report the position and velocity of the gantry, computed from encoder counts
        """
        msg = TransformStamped()
        msg.transform.translation.x = 0
        msg.transform.translation.y = 0
        msg.transform.translation.z = 0
        msg.transform.rotation.x = 1
        msg.transform.rotation.y = 0
        msg.transform.rotation.z = 0
        msg.transform.rotation.x = 1
        self.tf_broadcaster.sendTransform(msg)

        pass

if __name__ == '__main__':
    # init ros node
    rospy.init_node(NODE_NAME, anonymous=True)
    g = GantryNode()

    rospy.spin()