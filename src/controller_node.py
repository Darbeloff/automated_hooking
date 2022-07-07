#!/usr/bin/env python2

NODE_NAME = "HookingController"

import rospy
import numpy as np

from geometry_msgs.msg import Pose, Twist

from gantry_node import GantryNode
from winch_node import WinchNode


class HookingController:
    """
    This node speaks to all the others, keeping track of the progress toward hooking and attachment and ordering the other nodes around.
    
    The core algorithm runs here
    """
    def __init__(self):
        # subscribe to camera, gantry node, winch node, hook node

        # Publishers
        self.gantry_velocity_pub = rospy.Publisher(
            rospy.get_param("~/gantry_velocity_set_topic", "gantry/velocity_set"),
            Twist, queue_size=10)
        self.gantry_position_pub = rospy.Publisher(
            rospy.get_param("~/gantry_position_set_topic", "gantry/position_set"),
            Pose, queue_size=10)

        self.winch_velocity_pub = rospy.Publisher(
            rospy.get_param("~/winch_velocity_set_topic", "winch/velocity_set"),
            Twist, queue_size=1)
        self.winch_position_pub = rospy.Publisher(
            rospy.get_param("~/winch_position_set_topic", "winch/position_set"),
            Pose, queue_size=1)
        

        # Subscribers
        


        self.do_hooking_simple()

    def do_hooking_simple(self):
        tmsg = Twist()
        tmsg.linear = [1,0,0]
        self.gantry_velocity_pub.publish(tmsg)
        rospy.sleep(2)

        tmsg.linear = [0,0,0]
        self.gantry_velocity_pub.publish(tmsg)


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

if __name__ == '__main__':
    # init ros node
    rospy.init_node(NODE_NAME, anonymous=True)
    HookingController()