#!/usr/bin/env python2

NODE_NAME = "HookingController"

import rospy
import numpy as np

import tf2_ros

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, Twist, TransformStamped

import Utils
from Utils import Vector, Coord



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
        
        self.do_hooking_V2()

    def init_publishers(self):
        self.gantry_control_pub = rospy.Publisher(
            rospy.get_param("~/gantry_control_topic", "gantry/control"),
            JointState, queue_size=1)
        self.winch_control_pub = rospy.Publisher(
            rospy.get_param("~/winch_control_topic", "winch/control"),
            JointState, queue_size=1)
        

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    def init_subscribers(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # subscribe to camera, gantry node, winch node, hook node
        rospy.Subscriber(
            rospy.get_param("~/winch_state_topic", "winch/state"),
            JointState, self.winch_state_callback, queue_size=1)

    def init_state(self):
        self.winch_effort = [0,0,0]

        # TODO: should I store winch positions as 3 poses? I could publish commands for both gantry and winch by specifying one 3D coord of a hook... no, winch and gantry nodes should stay decoupled at this point
        # TODO: maybe only store this in the tf_tree
        self.winch_heights = [0,0,0]

    def do_hooking_simple(self):
        """
        Simple hooking algorithm: move in a direction until winch reports amperage above a threshold

        Note: OUTDATED
        """
        tmsg = Twist()
        tmsg.linear = Vector([2,0,0])
        self.gantry_velocity_pub.publish(tmsg)
        rospy.logwarn("move")
        
        Utils.await_condition(
            lambda: abs(self.winch_effort[0]) > 0.8,
            timeout=20,
            on_timeout=lambda: rospy.logwarn("Timed Out"))

        tmsg.linear = Vector([0,0,0])
        self.gantry_velocity_pub.publish(tmsg)
        rospy.logwarn("stop")

        rospy.logwarn("end")

    def do_hooking_V2(self):
        # Get position of table_tag in world space
        # Get position of target position in world space
        # Get movement direction from table_tag
        rospy.sleep(0.1)
        
        rospy.logwarn('moving!')
        while not rospy.is_shutdown():
            T_base = self.get_T('map', 'base_link')
            T_diff = self.get_T('pulley_arm_1_pulley_link', 'target_zone_1_link') # TODO: add offset here, instead of in target description

            if np.linalg.norm(T_diff.get_translation()[:2]) < 0.05:
                break
            
            # where we want the base to be
            T_base_target = T_base + T_diff

            # Move to target position in world space
            # Await arrival in world space
            self.control_gantry_position(T_base_target)

            rospy.sleep(0.1)
        
        rospy.logwarn('Arrived!')
        quit()

        # Lower hook appropriately
        # height = T_diff.get_translation()[2] - 0.1 # go 10cm below the target point
        # self.control_winch_position(height,0, wait=True)
        

        # Move in direction
        T_target = self.get_T('map', 'target_zone_1_link')
        move_direction = (T_target.T[:3,0]).flatten() # the x column of the rotation matrix
        move_speed = 0.2 
        self.control_gantry_velocity( move_direction * move_speed )
        
        rospy.sleep(1)
        # Await amperage trigger
        # Utils.await_condition(
        #     lambda: abs(self.winch_effort[0]) > 0.8,
        #     timeout=20,
        #     on_timeout=lambda: rospy.logwarn("Timed Out"))

        # Stop moving
        self.control_gantry_velocity([0,0,0])

        # Done
        rospy.logwarn("Done")

    def do_hooking_V3(self):
        # map target links to winches in an un-tangle-y way
        pairs = [(0,0),(1,1),(2,2)]
        
        # for winch/link pair
        for link,winch in pairs:
            # do_hooking_V2 on link and winch
            # command winch to maintain a certain amperage
            effort = [None]*3
            effort[winch] = 0.8
            self.control_winch_effort( effort )

        # move to above center of mass
        T_target = self.get_T('map', 'target_zone_1_link')
        self.control_gantry_position(T_target)

        # raise all winches
        self.control_winch_position([-0.5,-0.5,-0.5])
        
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



    def get_T(self, from_frame, to_frame):
        """
        Returns the transform from the from_frame to the to_frame as a handy Coord object
        """
        return Coord( self.tf_buffer.lookup_transform(from_frame,to_frame, rospy.Time(0)) )

    def winch_state_callback(self, msg):
        self.winch_effort = np.array(msg.effort)
        self.winch_position = np.array(msg.position)
        self.winch_velocity = np.array(msg.velocity)

    def control_gantry_velocity(self, velocity):
        msg = JointState()

        msg.name = ['x','y']
        msg.velocity = velocity

        self.gantry_control_pub.publish(msg)

    def control_gantry_position(self, coord, wait=False, error=0.05):
        msg = JointState()

        msg.name = ['x','y']
        msg.position = coord.get_translation()[:2]

        self.gantry_control_pub.publish(msg)

        if wait:
            Utils.await_condition(
                lambda: np.linalg.norm((coord - self.get_T('map', 'base_link')).get_translation()[:2]) < error
            )

    def control_winch_position(self, position, wait=False, error=0.05):
        msg = JointState()
        msg.name = [] # will be '0','1','2' if all heights are specified
        msg.position = []

        for i in range(len(position)):
            if not position[i] is None:
                msg.name.append(str(i))
                msg.position.append(position[i])

        self.winch_control_pub.publish(msg)

        # if wait:
        #     Utils.await_condition(
        #         lambda: np.linalg.norm(self.winch_position - position) < error
        #     )


    # UNTESTED
    def control_winch_position(self, position, winch, wait=False, error=0.05):
        msg = JointState()
        msg.name = [str(winch)]
        msg.effort = [effort]
        self.winch_control_pub.publish(msg)

        if wait:
            Utils.await_condition(
                lambda: abs(self.winch_position[winch] - position) < error
            )

    def control_winch_velocity(self, velocity):
        msg = JointState()
        msg.name = [] # will be '0','1','2' if all heights are specified
        msg.velocity = []

        for i in range(len(velocity)):
            if not velocity[i] is None:
                msg.name.append(str(i))
                msg.velocity.append(velocity[i])
        self.winch_control_pub.publish(msg)

    def control_winch_velocity(self, velocity, winch):
        msg = JointState()
        msg.name = [str(winch)]
        msg.velocity = [velocty]
        self.winch_control_pub.publish(msg)
    
    def control_winch_effort(self, effort):
        msg = JointState()
        msg.name = [] # will be '0','1','2' if all heights are specified
        msg.effort = []

        for i in range(len(effort)):
            if not effort[i] is None:
                msg.name.append(str(i))
                msg.effort.append(effort[i])

        self.winch_control_pub.publish(msg)
    
    def control_winch_effort(self, effort, winch):
        msg = JointState()
        msg.name = [str(winch)]
        msg.effort = [effort]
        self.winch_control_pub.publish(msg)

if __name__ == '__main__':
    # init ros node
    rospy.init_node(NODE_NAME, anonymous=True)
    HookingController()

    # rospy.spin()