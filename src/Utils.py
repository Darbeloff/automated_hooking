import rospy
import numpy as np

import tf2_ros
import tf.transformations as tf_t
from geometry_msgs.msg import Transform



class Vector:
    """
    A handy class to convert between iterables and .xyzw datatypes
    """
    def __init__(self, array):
        self.x = array[0]
        self.y = array[1]
        if len(array) > 2:
            self.z = array[2]
        if len(array) > 3:
            self.w = array[3]
    
    @staticmethod
    def to_array(vector):
        if vector is None:
            return None
            
        if hasattr(vector, 'w'):
            return np.array([vector.x, vector.y, vector.z, vector.w])
        elif hasattr(vector, 'z'):
            return np.array([vector.x, vector.y, vector.z])
        else:
            return np.array([vector.x, vector.y])

class Coord:
    """
    A handy class to do coordinate arithmetic with. Usefully converts poses, transforms, and matrices
    """

    def __init__(self, tf):
        if isinstance(tf, np.ndarray):
            self.T = tf
            return

        x,q = search_recursive(tf, [['translation','position'],['rotation','orientation']])

        P = tf_t.translation_matrix(Vector.to_array( x ))
        M = tf_t.quaternion_matrix( Vector.to_array( q ) )
        
        self.T = P.dot(M)

    def to_tf(self):
        tf = Transform()
        _,_,_,x,q = tf_t.decompose_matrix(self.T)

        tf.translation = Vector(x)
        tf.rotation = Vector(q)

        return tf

    def to_pose(self):
        pose = Pose()
        _,_,_,x,q = tf_t.decompose_matrix(self.T)
        
        pose.position = Vector(x)
        pose.orientation = Vector(q)
        
        return pose

    def __neg__(self):
        return Coord( numpy.linalg.inv(self.T) )

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

    def __repr__(self):
        return self.T.__str__()


def search_recursive(object, attributes, max_depth=3):
    # reshape keys to be a 2d array. Allows more general inputs
    for i in range(len(attributes)):
        if isinstance( attributes[i], str ):
            attributes[i] = [attributes[i]]

    # define output
    out = [None] * len(attributes)


    # define recursion function
    def search(sub_objects, depth):
        for sub_object in sub_objects:
            if sub_object is None:
                continue

            # search top level
            for i in range(len(attributes)):
                if attributes[i] is None:
                    continue
                
                for attribute in attributes[i]:
                    if hasattr(sub_object, attribute):
                        out[i] = getattr(sub_object, attribute)
            
            # check if done
            if ((np.array(out) != None).all()):
                return

        # stop if depth exceeded
        if depth < max_depth:
            search( [getattr(sub_object, attr, None)
                for sub_object in sub_objects
                for attr in dir(sub_object)],
                depth + 1)
    
    # search recurseively
    search([object], 1)

    return out

def await_condition(condition, timeout=60, on_condition=lambda: 0, on_timeout=lambda: 0, sleep_time=0.1):
    start_time = rospy.get_rostime().to_sec()
    
    while not rospy.is_shutdown() and rospy.get_rostime().to_sec() - start_time < timeout:
        if condition():
            on_condition()
            return
        
        rospy.sleep(sleep_time)

    on_timeout()