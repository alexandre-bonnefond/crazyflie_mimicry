import math
import numpy as np

def VelDecayLinSqrt(x, p, acc, v_max, r0):

    vel = (x - r0) * p

    if (acc <= 0 or p <= 0 or vel <= 0):
        return 0
    if (vel < acc / p):
        if vel >= v_max:
            return v_max
        return vel
    
    if (2 * acc * (x - r0) - acc * acc / p / p < 0):
        print("ouuuuuuch")
    vel = math.sqrt(2 * acc * (x - r0) - acc * acc / p / p)
    if (vel >= v_max):
        return v_max

    return vel

def unitVect(vect):

    norm = np.linalg.norm(vect)

    if norm < 10e-8:
        return np.zeros(3)
    
    else:
        return vect / norm
    
    
def compute_distance(pos1, pos2):

    return np.linalg.norm(pos1 - pos2)