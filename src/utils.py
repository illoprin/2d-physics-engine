from settings import *
import math


class Transform:
    def __init__(self,
                 pos: list[float, float] = [.0, .0],
                 rot: list[float, float] = [.0, .0]):
        self.position = np.asarray(pos, dtype='f4')
        self.rotation = np.asarray(rot, dtype='f4')

def get_rnd_color():
    rnd = np.random.random(3) * 255
    return tuple(rnd.tolist())

######## Vector Operations ########
def normalize(vec: np.ndarray):
    length = np.linalg.norm(vec)
    return vec / (length if length != 0 else 0.001)

def triple_cross_product(vec_1: np.ndarray, vec_2: np.ndarray, vec_3: np.ndarray):
    return np.cross(np.cross(vec_1, vec_2), vec_3)

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))
##################################