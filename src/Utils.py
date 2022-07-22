import numpy as np

class Vector:
    def __init__(self, array):
        self.x = array[0]
        self.y = array[1]
        if len(array) > 2:
            self.z = array[2]
    
    @staticmethod
    def to_array(vector):
        try:
            return np.array([vector.x, vector.y, vector.z])
        except:
            return np.array([vector.x, vector.y])


def await_condition(timeout, condition, on_timeout=lambda: 0, sleep_time=0.01):
        start_time = rospy.get_rostime().to_sec()
        
        while rospy.get_rostime().to_sec() - start_time < timeout:
            if condition():
                return
            
            rospy.sleep(sleep_time)

        on_timeout()