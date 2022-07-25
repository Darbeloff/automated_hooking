#!/usr/bin/env python2

NODE_NAME = "GantryDebug"

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist

from Utils import Vector

rospy.init_node(NODE_NAME, anonymous=True)

# Publishers
gantry_pub = rospy.Publisher(
    rospy.get_param("~/gantry_control_topic", "gantry/control"),
    JointState, queue_size=1)
        

msg = JointState()
msg.name = ['x','y']
rospy.sleep(0.5)

msg.position = [1.75,-0.25]
rospy.logwarn("start!")
gantry_pub.publish(msg)
rospy.sleep(5)

msg.position = [1.75,0.25]
rospy.logwarn("start!")
gantry_pub.publish(msg)
rospy.sleep(5)

# msg.position = []
# msg.velocity = [0.00111754,-0.15]
# rospy.logwarn("stop!")
# gantry_pub.publish(msg)
# rospy.sleep(3)


msg.position = []
msg.velocity = [0,0]
rospy.logwarn("stop!")
gantry_pub.publish(msg)