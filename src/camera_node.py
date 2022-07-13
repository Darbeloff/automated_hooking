#!/usr/bin/env python2

NODE_NAME = "CameraNode"

import rospy
import numpy as np

import opencv

from threading import Thread

from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class CameraNode:
    """
    This node connects to the ODrive motors that control the winches on the gantry crane
    It is separate from gantry node because it may run on a separate Pi
    """
    RATE = 20
    def __init__(self):

        # Publishers
        self.state_pub = rospy.Publisher(
            rospy.get_param("~/camera_position_topic", "camera/"),
            Odometry, queue_size=10)
        

        # Subscribers
        
        
        # def test():
        #     self.target_velocity = [0,-100,0]
        #     rospy.sleep(10)
        #     self.target_velocity = [0,0,0]

        # worker = Thread(target=test)
        # worker.daemon = True
        # worker.start()

        
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

        self.publish_state()

        self.prev_time = time

    
    def publish_state(self):
        """
        Report the position of the winch lines, computed from encoder counts
        """
        msg = JointState()
        msg.header.stamp = rospy.get_rostime()
        
        self.state_pub.publish(msg)


if __name__ == '__main__':
    # init ros node
    rospy.init_node(NODE_NAME, anonymous=True)
    CameraNode()

    rospy.spin()