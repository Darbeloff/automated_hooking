#!/usr/bin/env python2

NODE_NAME = "HookingController"

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist

import Utils
from Utils import Vector



class HookingController:
    """
    This node speaks to all the others, keeping track of the progress toward hooking and attachment and ordering the other nodes around.
    
    The core algorithm runs here
    """
    def __init__(self):

        # Publishers
        self.init_publishers()
        
        # Subscribers
        self.init_subscribers()


        self.init_state()
        
        rospy.sleep(1) # delay to allow topics to finish connecting
        rospy.logwarn(NODE_NAME + " is online")
        self.do_hooking_simple()

    def init_publishers(self):
        self.gantry_velocity_pub = rospy.Publisher(
            rospy.get_param("~/gantry_velocity_set_topic", "gantry/velocity_set"),
            Twist, queue_size=1)
        self.gantry_position_pub = rospy.Publisher(
            rospy.get_param("~/gantry_position_set_topic", "gantry/position_set"),
            Pose, queue_size=10)

        self.winch_velocity_pub = rospy.Publisher(
            rospy.get_param("~/winch_velocity_set_topic", "winch/velocity_set"),
            Twist, queue_size=1)
        self.winch_position_pub = rospy.Publisher(
            rospy.get_param("~/winch_position_set_topic", "winch/position_set"),
            Pose, queue_size=1)

    def init_subscribers(self):
        # subscribe to camera, gantry node, winch node, hook node
        rospy.Subscriber(
            rospy.get_param("~/winch_state_topic", "winch/state"),
            JointState, self.winch_state_callback, queue_size=1)

        

    def init_state(self):
        self.winch_effort = [0,0,0]

        # TODO: should I store winch positions as 3 poses? I could publish commands as just one... no, winch and gantry nodes should stay decoupled at this point
        self.winch_position = [0,0,0]


    def do_hooking_simple(self):
        tmsg = Twist()
        tmsg.linear = Vector([2,0,0])
        self.gantry_velocity_pub.publish(tmsg)
        rospy.logwarn("move")
        
        Utils.await_condition(20, lambda: abs(self.winch_effort[0]) > 0.8, on_timeout=lambda: rospy.logwarn("timed out"))

        tmsg.linear = Vector([0,0,0])
        self.gantry_velocity_pub.publish(tmsg)
        rospy.logwarn("stop")

        rospy.logwarn("end")

    def do_hooking_V2(self):
        # Get position of table_tag in world space
        # Get position of target position in world space
        # Get movement direction from table_tag
        
        # Move to target position in world space
        # Await arrival in world space
        # Lower hook appropriately
        # Move in direction
        # Await amperage trigger
        # Stop moving
        # Done

        pass


    def do_hooking(self):
        # do CV to get peg coords
        # compute box of acceptable ICs
        # tell gantry node to drive to center of IC box
        
        collision_back = True
        collision_low = True
        while collision_back or collision_low:
            # drive gantry toward peg until hook node reports collision
            
            collision_back = False # if collision was on back of hook
            collision_low = False # if collision was below the lip of the hook
            
            if collision_back or collision_low:
                # back off
                pass

            if collision_low:
                # decrease z_height of hook via winch node
                pass
        
        # move gantry toward peg until winch_node or gantry_node reports amperage spike

        # engage retainer via hook_node

        # move gantry above peg coords
        # retract winch until winch_node reports amperage spike
        # if collision is detected in an acceptable place, return
        # else, disengage, do_hooking again
        


    def winch_state_callback(self, msg):
        self.winch_effort = np.array(msg.effort)

        # rospy.loginfo(self.winch_effort)

if __name__ == '__main__':
    # init ros node
    rospy.init_node(NODE_NAME, anonymous=True)
    HookingController()

    # rospy.spin()