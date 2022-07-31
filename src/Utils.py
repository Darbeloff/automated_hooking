import rospy
import numpy as np

# import tf2_ros
# import tf.transformations as tf_t
# from geometry_msgs.msg import Transform

"""
A file of handy functions and small, handy classes. All run in both python 2 and 3.
"""


class LogFile:
    """
    A handy class to open and deal with logging files. Use when you want to update a csv over time. Mostly threadsafe
    """
    def __init__(self, filename, headers):
        #headers should be an array of headers
        #filename should just be a name, string
        self.filename = filename
        self.headers = headers
        with open(self.filename, "w") as file:
            file.write("time")
            for header in headers:
                file.write(",")
                file.write(header)

            file.write('\n')
    #call this every time you want to log, give it the ros timestamp
    def log(self, time, columns):
        try:
            assert(len(columns) == len(self.headers))
        except AssertionError:
            rospy.logwarn("Attempting to log an inconsistant number of values")
            return

        with open(self.filename, "a") as file:
            file.write(str(time))
            for col in columns:
                file.write(",")
                file.write(str(col))

            file.write('\n')

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

    def __repr__(self):
        return Vector.to_array(self).__repr__()

# class Coord:
#     """
#     A handy class to do coordinate arithmetic with. Usefully converts poses, transforms, and matrices
#     """

#     # I = Coord(np.eye(4)) # the identity coordinate
#     def __init__(self, *inputs):
#         if len(inputs) == 2:
#             # input is two vectors
#             position = inputs[0]
#             orientation = inputs[1]

#             # cleanse position input
#             if len( position ) != 3:
#                 position = list(position)
#                 position.append(0)

#             P = tf_t.translation_matrix( position )

#             # accept orientation as euler angles or quaternion
#             if len(orientation) == 4:
#                 M = tf_t.quaternion_matrix( orientation )
#             else:
#                 M = tf_t.euler_matrix( orientation[0], orientation[1], orientation[2], 'rxyz' )
            
#             self.T = P.dot(M)
            
#         elif isinstance(inputs[0], np.ndarray):
#             # input is a matrix
#             # TODO: input is lone position vector
#             self.T = inputs[0]
#         else:
#             # input is a ROS message
#             x,q = search_recursive(inputs[0], [['translation','position'],['rotation','orientation']])

#             P = tf_t.translation_matrix(Vector.to_array( x ))
#             M = tf_t.quaternion_matrix( Vector.to_array( q ))
            
#             self.T = P.dot(M)

#     def to_tf(self):
#         tf = Transform()
#         _,_,_,x,_ = tf_t.decompose_matrix(self.T)
#         q = tf_t.quaternion_from_matrix(self.T)
#         tf.translation = Vector(x)
#         tf.rotation = Vector(q)

#         return tf

#     def to_pose(self):
#         pose = Pose()
#         _,_,_,x,_ = tf_t.decompose_matrix(self.T)
#         q = tf_t.quaternion_from_matrix(self.T)
        
#         pose.position = Vector(x)
#         pose.orientation = Vector(q)
        
#         return pose

#     def get_translation(self):
#         _,_,_,x,_ = tf_t.decompose_matrix(self.T)
#         return x

#     def __neg__(self):
#         return Coord( np.linalg.inv(self.T) )

#     def __add__(self, other):
#         return Coord(self.T.dot(other.T))
#     def __mul__(self,other):
#         return self + other 

#     def __sub__(self, other):
#         return self + (- other)
#     def __div__(self,other):
#         return self - other

#     def __iadd__(self,other):
#         self.T = (self+other).T
#     def __imul__(self,other):
#         self += other
    
#     def __isub__(self,other):
#         self.T = (self-other).T
#     def __idiv__(self,other):
#         self -= other

#     def __abs__(self):
#         return np.linalg.norm(self.get_translation())

#     def __repr__(self):
#         return self.T.__str__()


def search_recursive(object, attributes, max_depth=3):
    """
    Handy function to look for attribute names in an object recursively. Particularly useful when trying to extract `position` or `translation` from many possible types of ROS message
    """


    # reshape attributes to be a 2d array. Allows more general inputs
    for i in range(len(attributes)):
        attributes[i] = np.ravel(attributes[i])

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
    """
    Handy function to delay until a condition is met. Offers optional arguments for on_condition and on_timeout (which will be returned if the condition is met or the function times out). Also allows the timeout duration to be specified
    """

    start_time = rospy.get_rostime().to_sec()
    
    while not rospy.is_shutdown() and rospy.get_rostime().to_sec() - start_time < timeout:
        if condition():
            return on_condition()
            
        rospy.sleep(sleep_time)

    return on_timeout()
