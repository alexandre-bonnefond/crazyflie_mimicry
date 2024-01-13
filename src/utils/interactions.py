import numpy as np
import utils.maths as maths

def compute_repulsion_force(positions, whichCF, r_rep, p_rep, normalize = False):

    repulsion_force = np.zeros(3)
    interacting_units = 0

    for i in positions.keys():

        if i == whichCF: 
            continue

        neighbor_pos_diff = positions[whichCF] - positions[i]
        distance  = np.linalg.norm(neighbor_pos_diff)

        if distance > r_rep : 
            continue

        interacting_units += 1

        repulsion_force += ((r_rep - distance) * p_rep * (maths.unitVect(neighbor_pos_diff)))

    if normalize == True:
        repulsion_force /= interacting_units
    
    # strg = np.linalg.norm(repulsion_force)
    # if strg !=0:
    #     print("Repulsion vector of {} is x:{} y:{} z:{} and strength is {}".format(whichCF, repulsion_force[0], repulsion_force[1], repulsion_force[2], strg))

    return repulsion_force

def compute_attraction_force(positions, whichCF, r_att, p_att, normalize = False):

    attraction_force = np.zeros(3)
    interacting_units = 0

    for i in positions.keys():

        if i == whichCF: 
            continue

        neighbor_pos_diff = positions[i] - positions[whichCF]
        distance  = np.linalg.norm(neighbor_pos_diff)

        if distance < r_att : 
            continue

        interacting_units += 1

        attraction_force += ((distance - r_att) * p_att * (maths.unitVect(neighbor_pos_diff)))

    if normalize == True:
        attraction_force /= interacting_units
    
    # strg = np.linalg.norm(attraction_force)
    # if strg != 0:
    #     print("Attraction vector of {} is x:{} y:{} z:{} and strength is {}".format(whichCF, attraction_force[0], attraction_force[1], attraction_force[2], strg))

    return attraction_force

def compute_self_propulsion(velocities, whichCF, v_0):

    vel = velocities[whichCF]
    spp_force = v_0 * maths.unitVect(vel)

    # print("spp vector of {} is x:{} y:{} z:{} and speed was {}".format(whichCF, spp_force[0], spp_force[1], spp_force[2], np.linalg.norm(vel)))

    return spp_force



def compute_friction_alignment(positions, velocities, C_frict_l, V_frict_l,
                               Acc_l, p_l, R_0_l, whichCF):
    
    pos = positions[whichCF]
    vel = velocities[whichCF]
    alignment_force = np.zeros(3)

    for i in positions.keys():
        if i == whichCF: 
            continue

        neighbor_pos_diff = positions[i] - positions[whichCF]
        distance  = np.linalg.norm(neighbor_pos_diff)
        neighbor_vel_diff = velocities[i] - velocities[whichCF]
        speed = np.linalg.norm(neighbor_vel_diff)
        unit_vect = maths.unitVect(neighbor_vel_diff)
        maxVelDiff = max(V_frict_l,
                         maths.VelDecayLinSqrt(distance, p_l, Acc_l,
                                               speed, R_0_l))
        
        if speed > maxVelDiff:
            unit_vect *= (C_frict_l * (speed - maxVelDiff))
            alignment_force += unit_vect

    # strg = np.linalg.norm(alignment_force)
    # if strg != 0:
    #     print("Align vector of {} is x:{} y:{} z:{} and strength is {}".format(whichCF, alignment_force[0], alignment_force[1], alignment_force[2], strg))

    return alignment_force


def map_wrist_pos_to_box(pos, box_z):

    for i in range(len(pos)):
        if pos[i] == -1:
            continue
        pos[i] = (box_z[0] - box_z[1]) * pos[i] + box_z[1]

    return pos

