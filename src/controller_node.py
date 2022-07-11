#!/usr/bin/env python2

NODE_NAME = "HookingController"

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist

# from gantry_node import GantryNode
# from winch_node import WinchNode

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

class HookingController:
    """
    This node speaks to all the others, keeping track of the progress toward hooking and attachment and ordering the other nodes around.
    
    The core algorithm runs here
    """
    def __init__(self):

        # Publishers
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
        

        # Subscribers
        # subscribe to camera, gantry node, winch node, hook node
        rospy.Subscriber(
            rospy.get_param("~/winch_state_topic", "winch/state"),
            JointState, self.winch_state_callback, queue_size=1)


        self.winch_effort = [0,0,0]
        
        rospy.sleep(1) # delay to allow topics to finish connecting
        rospy.logwarn(NODE_NAME + " is online")
        self.do_hooking_simple()

    def do_hooking_simple(self):
        tmsg = Twist()
        tmsg.linear = Vector([2,0,0])
        self.gantry_velocity_pub.publish(tmsg)
        rospy.logwarn("move")
        
        self.await_condition(20, lambda: abs(self.winch_effort[0]) > 0.8, on_timeout=lambda: rospy.logwarn("timed out"))

        tmsg.linear = Vector([0,0,0])
        self.gantry_velocity_pub.publish(tmsg)
        rospy.logwarn("stop")

        rospy.logwarn("end")


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

    def await_condition(self, timeout, condition, on_timeout=lambda: 0):
        start_time = rospy.get_rostime().to_sec()
        
        while rospy.get_rostime().to_sec() - start_time < timeout:
            if condition():
                return
            
            rospy.sleep(0.0001)

        on_timeout()
        


    def winch_state_callback(self, msg):
        self.winch_effort = np.array(msg.effort)

        # rospy.loginfo(self.winch_effort)

if __name__ == '__main__':
    # init ros node
    rospy.init_node(NODE_NAME, anonymous=True)
    HookingController()

    # rospy.spin()