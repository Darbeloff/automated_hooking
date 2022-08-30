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

def await_condition(condition, timeout=60, on_condition=lambda: 0, on_timeout=lambda: 0, sleep_time=0.01):
    """
    Handy function to delay until a condition is met. Offers optional arguments for on_condition and on_timeout (which will be returned if the condition is met or the function times out). Also allows the timeout duration to be specified
    """

    start_time = rospy.get_rostime().to_sec()
    
    while not rospy.is_shutdown() and rospy.get_rostime().to_sec() - start_time < timeout:
        if condition():
            return on_condition()
            
        rospy.sleep(sleep_time)

    return on_timeout()
