#!/usr/bin/env python2

NODE_NAME = "WinchDebug"

import rospy
import numpy as np

import sys

from Utils import Vector

from geometry_msgs.msg import Pose, Twist, Vector3



rospy.init_node(NODE_NAME, anonymous=True)

v_pub = rospy.Publisher(
    rospy.get_param("~/winch_velocity_set_topic", "winch/velocity_set"),
    Twist, queue_size=1)
p_pub = rospy.Publisher(
    rospy.get_param("~/winch_position_set_topic", "winch/position_set"),
    Pose, queue_size=1)
pid_pub = rospy.Publisher(
    rospy.get_param("~/winch_pid_set_topic", "winch/pid_set"),
    Vector3, queue_size=1)


v_msg = Twist()

def pos_test():
    # 400,000 counts = 14.5in = 0.3683m
    # -1.875
    p_msg = Pose()
    p_msg.position = Vector([-1.875,0,0])
    p_pub.publish(p_msg)

    # rospy.sleep(6)

    # p_msg.position = Vector([0,0,0])
    # p_pub.publish(p_msg)

def vel_test():
    v_msg.linear = Vector([-0.2,0,0])
    v_pub.publish(v_msg)

    # rospy.sleep(2)
    
    # v_msg.linear = Vector([-100,0,0])
    # v_pub.publish(v_msg)

    # rospy.sleep(2)

    # v_msg.linear = Vector([0,0,0])
    # v_pub.publish(v_msg)

def set_pid():
    msg = Vector3()
    msg.x = 10.0
    msg.y = 0.000005
    msg.z = 0.0001
    pid_pub.publish(msg)

if __name__ == '__main__':
    rospy.sleep(0.5)
    set_pid()
    rospy.sleep(1.5)

    if sys.argv[1] == '-v':
        print("velocity testing...")
        t = rospy.get_rostime().to_sec()
        vel_test()
        print('time taken: ' + str(rospy.get_rostime().to_sec() - t))
        print('done')

    if sys.argv[1] == '-p':
        t = rospy.get_rostime().to_sec()
        print("position testing...")
        pos_test()
        print('time taken: ' + str(rospy.get_rostime().to_sec() - t))
        print('done')
