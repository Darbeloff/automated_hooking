#!/usr/bin/env python2

NODE_NAME = "WinchNode"

import rospy
import numpy as np


class WinchNode:
    """
    This node connects to the ODrive motors that control the winches on the gantry crane
    It is separate from gantry node, because it will run on a separate PI
    """
    def __init__(self):
        pass


if __name__ == '__main__':
    # init ros node
    rospy.init_node(NODE_NAME, anonymous=True)
    WinchNode()