#!/usr/bin/env python2

NODE_NAME = "HookNode"

import rospy
import numpy as np
import serial

class HookNode:
    """
    This node interfaces with the hook-mounted arduino via the bridge class, recieving updates about the orientation and acceleration of the board. This node can also command the nano to open and close the retainer
    """
    def __init__(self):
        pass


if __name__ == '__main__':
    # init ros node
    rospy.init_node(NODE_NAME, anonymous=True)
    HookNode()