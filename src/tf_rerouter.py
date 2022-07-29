#!/usr/bin/env python2

NODE_NAME = "TFRerouter"

import sys
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


import Utils
from Utils import Vector, Coord

class TFRerouter:
    """
    This node creates a new tf frame a' at the location of frame a, rooted at frame b.
    Use when your sensors are moving (w.r.t. the detected object) and unreliable.
    """
    RATE = 20
    def __init__(self, a, a_prime, b):

        # Publishers
        self.init_publishers()
        
        # Subscribers
        self.init_subscribers()

        self.init_state()

        self.frame_a = a
        self.frame_a_prime = a_prime
        self.frame_b = b
        
        rospy.sleep(1) # delay to allow topics to finish connecting
        rospy.logwarn(NODE_NAME + " is online")
        
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            self.loop_callback()
            rate.sleep()

    def init_publishers(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()


    def init_subscribers(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
    def init_state(self):
        self.coord = Coord(np.eye(4))

    def loop_callback(self):
        try:
            self.coord = self.get_T(self.frame_b, self.frame_a)
            # rospy.loginfo('success!')
        except:
            pass
        
        msg = TransformStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = self.frame_b
        msg.child_frame_id = self.frame_a_prime
        msg.transform = self.coord.to_tf()

        self.tf_broadcaster.sendTransform(msg)
        

    def get_T(self, from_frame, to_frame):
        """
        Returns the transform from the from_frame to the to_frame as a handy Coord object
        """
        return Coord( self.tf_buffer.lookup_transform(from_frame,to_frame, rospy.Time(0)) )


if __name__ == '__main__':
    # init ros node
    rospy.init_node(NODE_NAME, anonymous=True)
    
    TFRerouter(*sys.argv[1:4])