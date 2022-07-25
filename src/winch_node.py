#!/usr/bin/env python2

NODE_NAME = "WinchNode"

import rospy
import numpy as np

from threading import Thread

from geometry_msgs.msg import Pose, Twist, Vector3
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from OdriveClass import Odrive
from Controls import PIDController, LPController
from Utils import Vector

class WinchNode:
    """
    This node connects to the ODrive motors that control the winches on the gantry crane
    It is separate from gantry node because it may run on a separate Pi
    """
    MAX_VEL = 100000 # Max. speed for winch in encoder counts per second
    
    COUNTS_PER_M = -5*400000 / 1.875
    M_PER_COUNT = 1./COUNTS_PER_M
    # TODO: Allow saving offsets for re-zeroing
    
    POSITION_BOUNDS = (-1.7, 0.025)
    
    RATE = 20
    def __init__(self):

        # Publishers
        self.state_pub = rospy.Publisher(
            rospy.get_param("~/winch_state_topic", "winch/state"),
            JointState, queue_size=10)


        # Subscribers
        rospy.Subscriber(
            rospy.get_param("~/winch_velocity_set_topic", "winch/velocity_set"),
            Twist, self.set_velocity_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~/winch_position_set_topic", "winch/position_set"),
            Pose, self.set_position_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~/winch_pid_set_topic", "winch/pid_set"),
            Vector3, self.set_pid_callback, queue_size=1)

        # Initialize Odrive interfaces
        self.odrv0 = Odrive('20673881304E') # Only has 1 winch
        self.odrv1 = Odrive('2087377E3548') # Has 2 winches
        # self.odrv0.reboot()
        # self.odrv1.reboot()

        

        self.current_filter = LPController(0.1)
        # self.velocity_controller = LPController(1.)
        # self.position_controller = PIDController(1.,0.001,0.001)


        self.target_velocity = np.zeros(3) 
        self.target_position = None

        # position 1,2,3; velocity 1,2,3; current 1,2,3
        self.position = [0,0,0]
        self.velocity = [0,0,0]
        self.current = [0,0,0]


        # def test():

        #     msg = Pose()
        #     msg.position = Vector([100000,0,0])
        #     self.set_position_callback(msg)
            
            
        #     rospy.sleep(4)

        #     msg = Pose()
        #     msg.position = Vector([0,0,0])
        #     self.set_position_callback(msg)

        # worker = Thread(target=test)
        # worker.daemon = True
        # worker.start()

        
        
        rospy.logwarn(NODE_NAME + " is online")
        rate = rospy.Rate(self.RATE)
        self.prev_time = rospy.get_rostime().to_sec() - 1.0/self.RATE
        while not rospy.is_shutdown():
            self.loop_callback()
            rate.sleep()

    def loop_callback(self):
        """
        Write velocity, do position control, and publish any updates
        """
        time = rospy.get_rostime().to_sec()
        delta_t = time - self.prev_time
        self.prev_time = time

        if self.target_position is None:
            self.write_velocity()
        else:
            self.write_position()
        
        self.position = np.array([   # in meters
            self.odrv0.get_encoder_count(0).pos_estimate,
            self.odrv1.get_encoder_count(0).pos_estimate,
            self.odrv1.get_encoder_count(1).pos_estimate
        ]) * self.M_PER_COUNT
        self.velocity = np.array([    # in meters per second
            self.odrv0.get_encoder_count(0).vel_estimate,
            self.odrv1.get_encoder_count(0).vel_estimate,
            self.odrv1.get_encoder_count(1).vel_estimate
        ]) * self.M_PER_COUNT
        self.raw_current = [      # in amps
            self.odrv0.get_current(0),
            self.odrv1.get_current(0),
            self.odrv1.get_current(1)
        ]


        # SAFETY
        # for bound in self.POSITION_BOUNDS:
        if any(self.position < self.POSITION_BOUNDS[0]) or any(self.position > self.POSITION_BOUNDS[1]):
            rospy.logwarn("STOPPED")
            # rospy.logwarn(self.position)
            self.target_velocity = [0,0,0]
            self.target_position = None
            self.write_velocity()

            rospy.sleep(2)

            self.target_position = [-0.05,-0.05,-0.05]
            self.write_position()

            rospy.sleep(2)

        # TODO: If too much current, do something. Disengage? Freeze?

        self.current = self.current_filter.do_control(self.current, self.raw_current, delta_t)
        
        self.publish_state()


    def set_velocity_callback(self, msg):
        """
        Set the target velocity of the gantry

        positive x: towards door
        positive y: towards machine wall
        positive z: up
        """
        self.target_velocity = Vector.to_array(msg.linear)
        self.target_position = None
        rospy.loginfo("velocity set:")
        rospy.loginfo(self.target_velocity)

    def set_position_callback(self, msg):
        """
        Set the target position of the gantry
        """
        self.target_velocity = None
        self.target_position = Vector.to_array(msg.position)

        rospy.loginfo("position set:")
        rospy.loginfo(self.target_position)

    def set_pid_callback(self, msg):
        
        Kp = msg.x
        Kd = msg.y
        Ki = msg.z
        self.odrv0.set_gains(0, Kp, Kd, Ki)
        self.odrv0.set_gains(0, Kp, Kd, Ki)
        self.odrv1.set_gains(1, Kp, Kd, Ki)

        rospy.loginfo(f"PID set to: {Kp}, {Kd}, {Ki}")

    def write_velocity(self):
        # Control winch
        self.target_velocity = np.array(self.target_velocity)

        des_vel = self.target_velocity*self.COUNTS_PER_M
        self.odrv0.VelMove(des_vel[0],0)
        self.odrv1.VelMove(des_vel[1],0)
        self.odrv1.VelMove(des_vel[2],1)

        # rospy.loginfo(des_vel)
    
    def write_position(self):
        # Control winch
        self.target_position = np.array(self.target_position)
        des_vel = np.clip(self.target_position, -1.875, 0)  * self.COUNTS_PER_M


        self.odrv0.PosMove(des_vel[0],0)
        self.odrv1.PosMove(des_vel[1],0)
        self.odrv1.PosMove(des_vel[2],1)

        # rospy.loginfo(des_vel)

    def publish_state(self):
        """
        Report the position of the winch lines, computed from encoder counts
        """
        msg = JointState()
        msg.header.stamp = rospy.get_rostime()
        msg.name = ['0','1','2']
        msg.position =  self.position   # in rotations
        msg.velocity = self.velocity    # in rotations per second
        msg.effort = self.current      # in amps

        self.state_pub.publish(msg)


if __name__ == '__main__':
    # init ros node
    rospy.init_node(NODE_NAME, anonymous=True)
    WinchNode()

    rospy.spin()