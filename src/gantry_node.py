#!/usr/bin/env python2

NODE_NAME = "GantryNode"

import rospy
import numpy as np
import ctypes

from threading import Thread

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


class Vector:
    def __init__(self, array):
        self.x = array[0]
        self.y = array[1]
        if len(array) > 2:
            self.z = array[2]
    
    @staticmethod
    def to_array(vector):
        try:
            return [vector.x, vector.y, vector.z]
        except:
            return [vector.x, vector.y]

class GantryNode:
    """
    This node interfaces with the ODrive boards and attached encoders, to drive the gantry at either a desired velocity or to a desired position

    Reports gantry position at some frequency
    """
    DATA_LENGTH = {
        'double':8,
        'int32':4,
        'bool':1,
        'bool3':3
    }
    NAME_TABLE = {
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


        'control_x_speed': [0x01, 'double'],
        'control_y_speed': [0x02, 'double'],
        
        'reset_encoder_0': [0x100, 'bool'],
        'reset_encoder_1': [0x200, 'bool'],
        'reset_encoder_2': [0x300, 'bool']
    }
    ID_TABLE = {}

    CAN_STATE = {}
        
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
        
        self.velocity_controller = LPController(0.1)
        self.position_controller = PIDController(1.,0.001,0.001)

        # copy NAME_TABLE into refactored ID_TABLE
        for key in self.NAME_TABLE.keys():
            id, can_type = self.NAME_TABLE[key]
            self.ID_TABLE[id] = [key, can_type]

        self.target_velocity = [0,0]
        self.target_position = None

        self.velocity = [0,0]
        self.position = [0,0] # TODO: Get this from encoder data
        
        self.prev_time = rospy.get_rostime().to_sec() - 1.0/self.RATE


        def test():
            tmsg = Twist()
            tmsg.linear = Vector([4,2,0])

            self.set_velocity_callback(tmsg)

            rospy.sleep(2)
            tmsg.linear = Vector([-4,-2,0])
            self.set_velocity_callback(tmsg)

            rospy.sleep(2)
            tmsg.linear = Vector([0,0,0])
            self.set_velocity_callback(tmsg)

        # worker = Thread(target=test)
        # worker.daemon = True
        # worker.start()

        # rospy.loginfo("yo")
        # msg = self.write_velocity([1,0])
        # msg2 = self.send_can_frame('control_x_speed', 1)
        # print(msg)
        # print(msg2)

        # quit()

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
        self.velocity = velocity_control # TODO: get this from sensor feedback

        self.publish_state()

        self.prev_time = time


    def set_velocity_callback(self, msg):
        """
        Set the target velocity of the gantry

        positive x: towards door
        positive y: towards machine wall
        positive z: up
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

    def can_message_callback(self, msg):
        """
        Process the messages recieved from the CAN node. Store them in state
        """
        
        pass

    def process(self, msg):
        """ Takes a Frame message and updates state accordingly
        """
        key, can_type = self.ID_TABLE[msg.id]

        numhex = numhex64()
        for idx in range(msg.dlc):
            numhex.hex[idx] = ord(msg.data[idx]) # TODO: check this

        if can_type == 'int32':
            val = numhex.uint
        elif can_type == 'double':
            val = numhex.num
        else:
            val = numhex.uint != 0
        
        CAN_STATE[key] = val

    def send_can_frame(self, name, value):
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
            numhex.uint = value
        elif can_type == 'double':
            numhex.num = value
        
        for idx in range(length):
            msg.data += chr(numhex.hex[idx])
        
        msg.header.stamp = rospy.get_rostime()
        self.gantry_pub.publish(msg)


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
        pass

if __name__ == '__main__':
    # init ros node
    rospy.init_node(NODE_NAME, anonymous=True)
    g = GantryNode()

    rospy.spin()

