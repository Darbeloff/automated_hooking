#!/usr/bin/env python3

NODE_NAME = "WinchNode"

import rospy
import numpy as np

from threading import Thread

from geometry_msgs.msg import Pose, Twist, Vector3
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from OdriveClass import Odrive
from ODrive import ODrive
from Controls import PIDController, LPController
from Utils import Vector

class WinchNode:
    """
    This node connects to the ODrive motors that control the winches on the gantry crane
    It is separate from gantry node to allow running on a separate Pi
    """
    MAX_VEL = 100000 # Max. speed for winch in encoder counts per second
    
    COUNTS_PER_M = -5*400000 / 1.875
    M_PER_COUNT = 1./COUNTS_PER_M
    
    # TODO: Allow saving offsets for re-zeroing
    OFFSET = np.zeros(3)

    POSITION_BOUNDS = (-1.7, 0.025)
    
    RATE = 20
    def __init__(self):

        # Publishers
        self.init_publishers()

        # Subscribers
        self.init_subscribers()

        # Initialize Odrive interfaces
        self.init_state()


        rospy.logwarn(NODE_NAME + " is online")
        rate = rospy.Rate(self.RATE)
        self.prev_time = rospy.get_rostime().to_sec() - 1.0/self.RATE
        while not rospy.is_shutdown():
            self.loop_callback()
            rate.sleep()

    def init_publishers(self):
        self.state_pub = rospy.Publisher(
            rospy.get_param("~/winch_state_topic", "winch/state"),
            JointState, queue_size=10)

    def init_subscribers(self):
        rospy.Subscriber(
            rospy.get_param("~/winch_control_topic", "winch/control"),
            JointState, self.control_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~/winch_pid_set_topic", "winch/pid_set"),
            Vector3, self.set_pid_callback, queue_size=1)

    def init_state(self):
        self.driver = ODrive('20673881304E', '2087377E3548')
        # self.driver.axes = self.driver.axes[[0,2,3]]

        self.effort_filter = LPController(0.5)

        self.position = [0,0,0]
        self.velocity = [0,0,0]
        self.effort_raw = [0,0,0]
        self.effort = [0,0,0]

    def loop_callback(self):
        """
        Write velocity, do position control, and publish any updates
        """
        # timekeeping
        time = rospy.get_rostime().to_sec()
        delta_t = time - self.prev_time
        self.prev_time = time

        self.position = self.driver.get_position() * self.M_PER_COUNT + self.OFFSET
        self.velocity = self.driver.get_velocity() * self.M_PER_COUNT
        self.effort_raw = self.driver.get_effort()

        self.effort = self.effort_filter.do_control(self.effort, self.effort_raw, delta_t)
        # TODO: If too much current, do something. Disengage? Freeze?

        # SAFETY
        if not all(self.position == np.clip(self.position, *self.POSITION_BOUNDS)):
            # Winch has exceeded safety bounds

            # Stop motion
            rospy.logwarn("STOPPED")
            self.driver.set_velocity_all( 0 )
            rospy.logwarn(self.position)
            rospy.logwarn(np.clip(self.position, *self.POSITION_BOUNDS))
            rospy.sleep(2)

            # Return to home
            self.driver.set_position_all( -0.05 )
            rospy.sleep(2)

        self.publish_state()

    def set_current_position(self, position):
        diff = np.array(position) - self.position
        self.OFFSET += diff


    def control_callback(self, msg):
        """
        Set the target position, velocity, or effort of each winch
        """
        ids = [int(name) for name in msg.name]

        if msg.velocity:
            self.driver.set_velocity(ids, np.array(msg.velocity) * self.COUNTS_PER_M)
            rospy.loginfo("Received Velocity: " + str(msg.velocity))
        elif msg.position:
            self.driver.set_position(ids, np.array(msg.position) * self.COUNTS_PER_M)
            rospy.loginfo("Received Position: " + str(msg.position))
        elif msg.effort:
            self.driver.set_effort(ids, msg.effort)
            rospy.logwarn("Received Effort: " + str(msg.effort))


    def set_pid_callback(self, msg):
        self.driver.set_gains(*Vector.to_array(msg))

        rospy.loginfo("PID set to: %f, %f, %f", *Vector.to_array(msg))

    def publish_state(self):
        """
        Report the position of the winch lines, computed from encoder counts
        """
        msg = JointState()
        msg.header.stamp = rospy.get_rostime()
        msg.name = ['0','1','2']
        msg.position =  self.position   # in rotations
        msg.velocity = self.velocity    # in rotations per second
        msg.effort = self.effort      # in Nm

        self.state_pub.publish(msg)


if __name__ == '__main__':
    # init ros node
    rospy.init_node(NODE_NAME, anonymous=True)
    WinchNode()

    rospy.spin()