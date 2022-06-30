#!/usr/bin/env python2

NODE_NAME = "GantryNode"

import rospy
import numpy as np


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
        # subscribers
        rospy.Subscriber(
            rospy.get_param("~/gantry_velocity_set_topic", "gantry_velocity_set"),
            Pose, self.set_velocity_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~/gantry_position_set_topic", "gantry_position_set"),
            Pose, self.set_position_callback, queue_size=1)
        
        
        rate = rospy.Rate(20)
        while(True):
            self.loop_callback()
            rate.sleep()

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
        pass

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
    GantryNode()
