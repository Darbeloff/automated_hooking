#!/usr/bin/env python2

NODE_NAME = "GantryNode"

import rospy
import numpy as np
import ctypes

from can_msgs.msg import Frame
from geometry_msgs.msg import Pose

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

    Reports gantry position at some frequency
    """
    def __init__(self):
        # publishers
        self.position_pub = rospy.Publisher(
            rospy.get_param("~/gantry_position_topic", "gantry/position"),
            Pose, queue_size=1)

        self.gantry_pub = rospy.Publisher(
            '/sent_messages',
            Frame,
            queue_size=1)

        # subscribers
        rospy.Subscriber(
            rospy.get_param("~/gantry_velocity_set_topic", "gantry/velocity_set"),
            Pose, self.set_velocity_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~/gantry_position_set_topic", "gantry/position_set"),
            Pose, self.set_position_callback, queue_size=1)
        
        
        # rate = rospy.Rate(20)
        # while(True):
        #     self.loop_callback()
        #     rate.sleep()

    def loop_callback(self):
        """
        Write velocity, do position control, and publish any updates
        """
        
        self.publish_position()
        pass

    def set_velocity_callback(self, velocity):
        """
        Set the target velocity of the gantry
        """

        ySpeed = numhex64()
        xSpeed = numhex64()

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
        # flip signs to convert from IMU coordinate system to that used by the gantry.
        xSpeed.num = velocity[0]
        ySpeed.num = velocity[1]

        # Odrive speed
        zSpeed = -velocity[2]

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


        # Control winch
        # des_vel = MAX_VEL*float(zSpeed)/100
        # odrv0.VelMove(des_vel,0)

        self.vx = xSpeed.num
        self.vy = ySpeed.num
        self.vz = -zSpeed

        rospy.loginfo("x: %f rps, y: %f rps, z: %f perc", xSpeed.num, ySpeed.num, -zSpeed) 
        

    def set_position_callback(self, position):
        """
        Set the target position of the gantry
        """
        pass

    def write_velocity(self, velocity):
        """
        Send velocity commands to the gantry motors
        """ 
        pass

    def publish_position(self):
        """
        Report the position of the gantry, computed from encoder counts
        """
        pass

if __name__ == '__main__':
    # init ros node
    rospy.init_node(NODE_NAME, anonymous=True)
    g = GantryNode()

    g.set_velocity_callback([0.2,0,0])
    rospy.sleep(1)
    g.set_velocity_callback([0,0,0])

