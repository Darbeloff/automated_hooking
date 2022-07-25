#!/usr/bin/env python2

NODE_NAME = "HookingController"

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist

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

rospy.init_node(NODE_NAME, anonymous=True)

# Publishers
gantry_velocity_pub = rospy.Publisher(
    rospy.get_param("~/gantry_velocity_set_topic", "gantry/velocity_set"),
    Twist, queue_size=1)
gantry_position_pub = rospy.Publisher(
    rospy.get_param("~/gantry_position_set_topic", "gantry/position_set"),
    Pose, queue_size=10)

# winch_velocity_pub = rospy.Publisher(
#     rospy.get_param("~/winch_velocity_set_topic", "winch/velocity_set"),
#     Twist, queue_size=1)
# winch_position_pub = rospy.Publisher(
#     rospy.get_param("~/winch_position_set_topic", "winch/position_set"),
#     Pose, queue_size=1)
        

msg = Twist()
rospy.sleep(0.5)
duration = 1

msg.linear = Vector([0.5,0,0])
rospy.logwarn("start!")
gantry_velocity_pub.publish(msg)

rospy.sleep(duration)

msg.linear = Vector([0,0,0])
rospy.logwarn("stop!")
gantry_velocity_pub.publish(msg)