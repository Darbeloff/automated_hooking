#!/usr/bin/env python2

NODE_NAME = "GantryNode"

import rospy
import numpy as np
import ctypes

from can_msgs.msg import Frame
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry

from Controls import PIDController, LPController

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

class State:

    DATA_LENGTH = {
        'double':8,
        'int32':4,
        'bool':1,
        'bool3':3
    }
    NAME_table = {
        'x_axis_actual_control_speed_0': [0x11, 'double'],
        'x_axis_actual_control_speed_1': [0x12, 'double'],
        'y_axis_actual_control_speed': [0x13, 'double'],

        'laser_distance_0': [0x101, 'int32'],
        'laser_distance_1': [0x201, 'int32'],
        'laser_distance_2': [0x301, 'int32'],

        'encoder_ppr_0': [0x102, 'int32'],
        'encoder_inc_0': [0x103, 'int32'],
        'encoder_abs_0': [0x104, 'int32'],
        'encoder_speed_0': [0x105, 'int32'],
        
        'encoder_ppr_1': [0x202, 'int32'],
        'encoder_inc_1': [0x203, 'int32'],
        'encoder_abs_1': [0x204, 'int32'],
        'encoder_speed_1': [0x105, 'int32'],

        'encoder_ppr_2': [0x302, 'int32'],
        'encoder_inc_2': [0x303, 'int32'],
        'encoder_abs_2': [0x304, 'int32'],
        'encoder_speed_2': [0x305, 'int32'],


        'control_x_speed': [0x01, 'double']
        'control_y_speed': [0x02, 'double']
        
        'reset_encoder_0': [0x100, 'bool']
        'reset_encoder_1': [0x200, 'bool']
        'reset_encoder_2': [0x300, 'bool']
    }
    ID_table = {}

    state = {}

    def __init__(self):
        # copy NAME_table into refactored ID_table
        for key in self.NAME_table.keys():
            id, can_type = self.NAME_table[key]
            self.ID_table[id] = [key, can_type] 
        
    
    def process(self, msg):
        """ Takes a Frame message and updates state accordingly
        """
        key = self.id_table[msg.id]

        numhex = numhex64()
        numhex.hex = msg.data # TODO: check this

        state[key] = numhex.num

    def write(self, name, value):
        id, can_type = self.NAME_table[name]
        length = self.DATA_LENGTH[can_type]

        
        msg = Frame()
        msg.is_rtr = False
        msg.is_extended = False
        msg.dlc = length
        msg.id = id
        msg.data = ""
        
        numhex = numhex64()
        numhex.num = value
        
        for idx in range(length):
            msg.data += chr(numhex.hex[idx])
        
        msg.header.stamp = rospy.get_rostime()
        self.gantry_pub.publish(msg)



class GantryNode:
    """
    This node interfaces with the ODrive boards and attached encoders, to drive the gantry at either a desired velocity or to a desired position

    Reports gantry position at some frequency
    """
    RATE = 20 # hz


    def __init__(self):
        # Publishers
        self.state_pub = rospy.Publisher(
            rospy.get_param("~/gantry_state_topic", "gantry/state"),
            Odometry, queue_size=10)

        self.gantry_pub = rospy.Publisher(
            rospy.get_param('~/can_sent_messages','/sent_messages'),
            Frame, queue_size=10)
            # received by the CAN node on the gantry (which in turn speaks to the CAN arduino over serial via dark and unknowable magics)


        # Subscribers
        rospy.Subscriber(
            rospy.get_param("~/can_received_messages", "/received_messages"),
            Frame, self.can_message_callback, queue_size=10)

        rospy.Subscriber(
            rospy.get_param("~/gantry_velocity_set_topic", "gantry/velocity_set"),
            Twist, self.set_velocity_callback, queue_size=10)
        rospy.Subscriber(
            rospy.get_param("~/gantry_position_set_topic", "gantry/position_set"),
            Pose, self.set_position_callback, queue_size=10)
        
        self.velocity_controller = LPController(1.)
        self.position_controller = PIDController(1.,0.001,0.001)

        self.target_velocity = [0,0]
        self.target_position = None

        self.velocity = [0,0]
        self.position = [0,0] # TODO: Get this from encoder data
        # 
        self.state = np.zeros(4)

        self.prev_time = rospy.get_rostime().to_sec() - 1.0/self.RATE

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
            velocity_control = self.position_controller.do_control(self.velocity, self.target_position, delta_t)

        self.write_velocity(velocity_control)
        self.velocity = self.velocity_control # TODO: get this from sensor feedback

        self.publish_position()

        self.prev_time = time

    def set_velocity_callback(self, msg):
        """
        Set the target velocity of the gantry

        positive x: towards door
        positive y: towards machine wall
        positive z: up
        """
        # TODO: might be in wrong form
        self.target_velocity = msg.linear
        self.target_position = None
        

    def set_position_callback(self, msg):
        """
        Set the target position of the gantry
        """
        self.target_velocity = None
        self.target_position = msg.position

    def write_velocity(self, velocity):
        """
        Send velocity commands to the gantry motors
        """

        xSpeed = numhex64()
        xSpeed.num = velocity[0]

        ySpeed = numhex64()
        ySpeed.num = velocity[1]


        msgData1 = ""
        frame1 = Frame()
        frame1.is_rtr = False
        frame1.is_extended = False
        frame1.dlc = 8

        msgData2 = ""
        frame2 = Frame()
        frame2.is_rtr = False
        frame2.is_extended = False
        frame2.dlc = 8
        
        

        # set x velocity
        frame1.id = 0x01
        msgData1 = ""
        for idx in range(8):
            msgData1 += chr(xSpeed.hex[idx])
        frame1.data = msgData1
        frame1.header.stamp = rospy.Time.now()
        self.gantry_pub.publish(frame1)
        
        # set y velocity
        frame2.id = 0x02
        msgData2 = ""
        for idx in range(8):
            msgData2 += chr(ySpeed.hex[idx])
        frame2.data = msgData2
        frame2.header.stamp = rospy.Time.now()
        self.gantry_pub.publish(frame2)

        # rospy.loginfo("x: %f rps, y: %f rps, z: %f perc", xSpeed.num, ySpeed.num, -zSpeed)

    def publish_state(self):
        """
        Report the position and velocity of the gantry, computed from encoder counts
        """
        pass

if __name__ == '__main__':
    # init ros node
    rospy.init_node(NODE_NAME, anonymous=True)
    g = GantryNode()

    g.set_velocity_callback([0.2,0])
    rospy.sleep(1)
    g.set_velocity_callback([0,0])

    rospy.spin()

