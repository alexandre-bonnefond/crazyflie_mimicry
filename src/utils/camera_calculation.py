import numpy as np
import utils.maths as maths

def compute_prefered_direction(left_wrist, right_wrist, v_0):
    # The idea is to output a velocity vector depending on the position of the wrists with regard to the center of the image on the camera
    pref_dir = np.zeros(3)
    
    if left_wrist[1] == -1 or right_wrist[1] == -1:
        return pref_dir
    
    average_height = (left_wrist[1] + right_wrist[1]) / 2
    pref_dir[2] = (0.5 - average_height) * 2 * v_0
    
    return pref_dir

def compute_equilibrium_distance(left_wrist, right_wrist, eq_dist):
    l_w = np.array(left_wrist)
    r_w = np.array(right_wrist)

    if left_wrist[0] == -1 or left_wrist[1] == -1 or right_wrist[0] == -1 or right_wrist[1] == -1:
        return eq_dist
    
    wrist_vect = r_w - l_w
    dist = np.linalg.norm(wrist_vect)
    print(dist)
    return dist

def compute_yaw(left_wrist, right_wrist, yaw_init):
    l_w = np.array(left_wrist)
    r_w = np.array(right_wrist)

    if left_wrist[0] == -1 or left_wrist[1] == -1 or right_wrist[0] == -1 or right_wrist[1] == -1:
        return yaw_init
    
    wrist_vect = l_w - r_w
    # x = np.array([1, 0])
    # yaw = np.arctan2(wrist_vect[:1], x)
    yaw = maths.angle_with_x(wrist_vect)
    yaw = np.copysign(yaw, (-right_wrist[1] + left_wrist[1]))
    yaw = np.rad2deg(yaw)
    print(yaw)
    return yaw