#!/usr/bin/env python2

NODE_NAME = "WinchNode"

import rospy
import numpy as np

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry

from OdriveClass import Odrive


class WinchNode:
    """
    This node connects to the ODrive motors that control the winches on the gantry crane
    It is separate from gantry node because it may run on a separate Pi
    """
    MAX_VEL = 100000 # Max. speed for winch in encoder counts per second
    RATE = 20
    def __init__(self):

        # Publishers
        self.position_pub = rospy.Publisher(
            rospy.get_param("~/winch_position_topic", "winch/position"),
            Pose, queue_size=10)


        # Subscribers
        rospy.Subscriber(
            rospy.get_param("~/winch_velocity_set_topic", "winch/velocity_set"),
            Twist, self.set_velocity_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~/winch_position_set_topic", "winch/position_set"),
            Pose, self.set_position_callback, queue_size=1)

        # Initialize Odrive interfaces
        odrv0 = Odrive('20673881304E') # Only has 1 winch
        odrv1 = Odrive('2087377E3548') # Has 2 winches



        self.velocity_controller = LPController(1.)
        self.position_controller = PIDController(1.,0.001,0.001)

        self.target_velocity = [0,0]
        self.target_position = None

        # position 1,2,3; velocity 1,2,3; current 1,2,3
        self.position = [0,0,0]
        self.velocity = [0,0,0]
        self.current = [0,0,0]

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
            velocity_control = self.position_controller.do_control(self.position, self.target_position, delta_t)

        self.write_velocity(velocity_control)
        self.velocity = self.velocity_control # TODO: get this from sensor feedback

        self.publish_state()

        self.prev_time = time

    def set_velocity_callback(self, msg):
        """
        Set the target velocity of the gantry

        positive x: towards door
        positive y: towards machine wall
        positive z: up
        """
        self.target_velocity = msg.linear
        self.target_position = None

    def set_position_callback(self, msg):
        """
        Set the target position of the gantry
        """
        self.target_velocity = None
        self.target_position = msg.position


    def write_velocity(self, velocity):
        # Odrive speed
        zSpeed = -velocity
        # Control winch
        des_vel = self.MAX_VEL*float(zSpeed)/100.
        odrv0.VelMove(des_vel,0)

    def publish_state(self):
        """
        Report the position of the winch lines, computed from encoder counts
        """
        pass


if __name__ == '__main__':
    # init ros node
    rospy.init_node(NODE_NAME, anonymous=True)
    WinchNode()

    rospy.spin()