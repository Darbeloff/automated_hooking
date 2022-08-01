#!/usr/bin/env python2

NODE_NAME = "WinchDebug"

import rospy
import numpy as np

import sys

from Utils import Vector

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState



rospy.init_node(NODE_NAME, anonymous=True)

pub = rospy.Publisher(
    rospy.get_param("~/winch_control_topic", "winch/control"),
    JointState, queue_size=10)
pid_pub = rospy.Publisher(
    rospy.get_param("~/winch_pid_set_topic", "winch/pid_set"),
    Vector3, queue_size=1)



def set_pid():
    msg = Vector3()
    msg.x = 10.0
    msg.y = 0.000005
    msg.z = 0.0001
    pid_pub.publish(msg)



rospy.sleep(0.5)
# set_pid()
# rospy.sleep(1.5)

msg = JointState()
msg.name = ['0']
msg.position = [0.]
# msg.effort = [-0.9]
# 2.804 kg
# F = m*g = 2.804*9.81

pub.publish(msg)
quit()