#!/usr/bin/env python2

NODE_NAME = "HookNode"

import rospy
import numpy as np
import serial

from sensor_msgs.msg import Imu
from std_msgs.msg import Bool as Boolmsg

from hook_bridge import HookBridge

class HookNode:
    """
    This node interfaces with the hook-mounted arduino via the bridge class, recieving updates about the orientation and acceleration of the board. This node can also command the arduino to open and close the retainer
    """
    def __init__(self):
        # Publishers
        self.state_publisher = rospy.Publisher(
            rospy.get_param("~/hook_state_topic", "hook/state"),
            Imu, queue_size=10)

        # Subscribers
        rospy.Subscriber(
            rospy.get_param("~/hook_retainer_topic", "hook/retainer"),
            Boolmsg, queue_size=1
            self.retainer_callback)

        self.bridge = HookBridge(new_thread=True)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.loop_callback()
            rate.sleep()
        
    def loop_callback(self):
        self.publish_state()

    def retainer_callback(self, msg):
        self.bridge.set_retainer(msg.data)

    def publish_state(self):
        """ Copy data from hook_bridge to a new message, publish
        """

        msg = Imu()
        msg.header.stamp = rospy.get_rostime()

        q = self.bridge.state['quaternion']
        msg.quaternion.x = q[0]
        msg.quaternion.y = q[1]
        msg.quaternion.z = q[2]
        msg.quaternion.w = q[3]

        msg.angular_velocity = self.bridge.state['gyro']
        msg.acceleration = self.bridge.state['accelaration']

        self.state_publisher.publish(msg)


if __name__ == '__main__':
    # init ros node
    rospy.init_node(NODE_NAME, anonymous=True)
    HookNode()