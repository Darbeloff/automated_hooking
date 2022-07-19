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