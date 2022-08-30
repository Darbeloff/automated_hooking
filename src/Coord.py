import rospy
import numpy as np

import tf2_ros
import tf.transformations as tf_t
from geometry_msgs.msg import Transform
from Utils import *


class Coord:
    """
    A handy class to do coordinate arithmetic with. Usefully converts poses, transforms, and matrices. Only works in python2 due to dependence on tf.transformations
    """

    # I = Coord(np.eye(4)) # the identity coordinate
    def __init__(self, *inputs):
        if len(inputs) == 2:
            # input is two vectors
            position = inputs[0]
            orientation = inputs[1]

            # cleanse position input
            if len( position ) != 3:
                position = list(position)
                position.append(0)

            P = tf_t.translation_matrix( position )

            # accept orientation as euler angles or quaternion
            if len(orientation) == 4:
                M = tf_t.quaternion_matrix( orientation )
            else:
                M = tf_t.euler_matrix( orientation[0], orientation[1], orientation[2], 'rzyx' )
            
            self.T = P.dot(M)
            
        elif isinstance(inputs[0], np.ndarray):
            # input is a matrix
            # TODO: input is lone position vector
            self.T = inputs[0]
        else:
            # input is a ROS message
            x,q = search_recursive(inputs[0], [['translation','position'],['rotation','orientation']])

            P = tf_t.translation_matrix(Vector.to_array( x ))
            M = tf_t.quaternion_matrix( Vector.to_array( q ))
            
            self.T = P.dot(M)

    def to_tf(self):
        tf = Transform()
        _,_,_,x,_ = tf_t.decompose_matrix(self.T)
        q = tf_t.quaternion_from_matrix(self.T)
        tf.translation = Vector(x)
        tf.rotation = Vector(q)

        return tf

    def to_pose(self):
        pose = Pose()
        _,_,_,x,_ = tf_t.decompose_matrix(self.T)
        q = tf_t.quaternion_from_matrix(self.T)
        
        pose.position = Vector(x)
        pose.orientation = Vector(q)
        
        return pose

    def get_translation(self):
        _,_,_,x,_ = tf_t.decompose_matrix(self.T)
        return x

    def __neg__(self):
        return Coord( np.linalg.inv(self.T) )

    def __add__(self, other):
        return Coord(self.T.dot(other.T))
    def __mul__(self,other):
        return self + other 

    def __sub__(self, other):
        return self + (- other)
    def __div__(self,other):
        return self - other

    def __iadd__(self,other):
        self.T = (self+other).T
    def __imul__(self,other):
        self += other
    
    def __isub__(self,other):
        self.T = (self-other).T
    def __idiv__(self,other):
        self -= other

    def __abs__(self):
        return np.linalg.norm(self.get_translation())

    def __repr__(self):
        return self.T.__str__()