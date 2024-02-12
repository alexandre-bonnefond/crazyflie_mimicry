import time
import numpy as np
import math
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig
import pose_input
import utils.interactions as interactions
import utils.maths as maths
import utils.camera_calculation as camcalc


pos_dict = {}
vel_dict = {}

def log_callback(uri, timestamp, data, logconf):
    x = data['stateEstimate.x']
    y = data['stateEstimate.y']
    z = data['stateEstimate.z']
    pos = np.array([x, y, z])
    pos_dict[uri] = pos

    vx = data['stateEstimate.vx']
    vy = data['stateEstimate.vy']
    vz = data['stateEstimate.vz']
    vel = np.array([vx, vy, vz])
    vel_dict[uri] = vel
    # print('uri: {} pos: ({}, {}, {})'.format(uri, x, y, z))


def start_states_log(scf):
    log_conf = LogConfig(name='States', period_in_ms=10)
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')

    log_conf.add_variable('stateEstimate.vx', 'float')
    log_conf.add_variable('stateEstimate.vy', 'float')
    log_conf.add_variable('stateEstimate.vz', 'float')
    uri = scf.cf.link_uri
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(lambda timestamp, data, logconf: log_callback(uri, timestamp, data, logconf))
    log_conf.start()



def start_distance_printing(scf):

    for i in pos_dict.keys():
        if i == scf.cf.link_uri: continue
        dist = maths.compute_distance(vel_dict[scf.cf.link_uri], vel_dict[i])

        print('Distance between URI {} and URI {} is {}'.format(i, scf.cf.link_uri, dist))
    print("\n")



uris = [
    'radio://0/80/2M/E7E7E7E701',
    # 'radio://0/80/2M/E7E7E7E702',
    'radio://0/80/2M/E7E7E7E703',
]

def is_in_box_limit(box_limits, positions, whichCF, v_0):

    x = positions[whichCF][0]
    y = positions[whichCF][1]
    z = positions[whichCF][2]

    back_inside_dir = [0, 0, 0]

    if abs(x) >= box_limits[0]:
        back_inside_dir[0] = -math.copysign(v_0, x)
        # print("x limit reached")    
    if abs(y) >= box_limits[1]:
        back_inside_dir[1] = -math.copysign(v_0, y)
        # print("y limit reached")
    if z >= box_limits[2]:
        back_inside_dir[2] = -v_0
        # print("z+ limit reached")
    if z < 0.2:
        back_inside_dir[2] = v_0/2
        # print("z- limit")
    
    return back_inside_dir

def take_off(cf, height):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = height / take_off_time
    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)


def land(cf, position):
    print('landing...')
    landing_time = 4.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position[2] / landing_time
    for _ in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

    cf.commander.send_stop_setpoint()
    cf.commander.send_notify_setpoint_stop()
    time.sleep(0.1)


def run_sequence(scf):
    try:
        cf = scf.cf

        take_off(cf, 1)
        start_time = time.time()
        current_time = start_time
        total_time = 15
        box_limits = [1.0, 1.0, 1.8]
        v_0 = 0.35
        eq_dist = 0.35
        yaw = 0
        while current_time < total_time + start_time:
            # start_distance_printing(scf)
            # print(pose_input.wrists_pos[0])
            going_back = is_in_box_limit(box_limits, pos_dict, scf.cf.link_uri, v_0)
            eq_dist = camcalc.compute_equilibrium_distance(pose_input.left_wrist_pos, pose_input.right_wrist_pos, eq_dist)
            # yaw = camcalc.compute_yaw(pose_input.left_wrist_pos, pose_input.right_wrist_pos, yaw)
            # print(eq_dist)
            if all(el == 0 for el in going_back):
                att = interactions.compute_attraction_force(pos_dict, scf.cf.link_uri, eq_dist + 0.1, 0.3, False)
                rep = interactions.compute_repulsion_force(pos_dict, scf.cf.link_uri, eq_dist, 2, False)
                spp = interactions.compute_self_propulsion(vel_dict, scf.cf.link_uri, v_0)
                ali = interactions.compute_friction_alignment(pos_dict, vel_dict, 0.8, 0.2, 1, 5, 0.6, scf.cf.link_uri)

                zforce = camcalc.compute_prefered_direction(pose_input.left_wrist_pos, pose_input.right_wrist_pos, 2*v_0)
                # pos = interactions.map_wrist_pos_to_box(pose_input.wrists_pos, [0.2, box_limits[-1]])
                # print("{} and {}".format(pos[0], pos[1]))
                force = rep + ali + spp + att + zforce

                # force = rep + att + zforce

                # if scf.cf.link_uri == uris[0]:
                #     if pos[0] != -1:
                #         zdist = pos[0]
                #         # cf.commander.send_hover_setpoint(0, 0, 0, zdist)
                #         print("left is going at {}".format((0.5 - zdist) * v_0 * 2))
                #         cf.commander.send_velocity_world_setpoint(0, 0, (0.5 - zdist) * v_0 * 2, 0)
                #     else:
                #         cf.commander.send_velocity_world_setpoint(force[0], force[1], force[2], 0)

                # else:
                #     if pos[1] != -1:
                #         zdist = pos[1]
                #         # cf.commander.send_hover_setpoint(0, 0, 0, zdist)
                #         print("right is going at {}".format((0.5 - zdist) * v_0 * 2))
                #         cf.commander.send_velocity_world_setpoint(0, 0, (0.5 - zdist) * v_0 * 2, 0)
                #     else:
                #         cf.commander.send_velocity_world_setpoint(force[0], force[1], force[2], 0)

                cf.commander.send_velocity_world_setpoint(force[0], force[1], force[2], 0)
                print('uri: {} force: ({}, {}, {})'.format(scf.cf.link_uri, force[0], force[1], force[2]))
                time.sleep(0.1)
            
            else:
                cf.commander.send_velocity_world_setpoint(going_back[0], going_back[1], going_back[2], 0)
                print('back')
                time.sleep(0.1)
            
            current_time = time.time()
        land(cf, pos_dict[scf.cf.link_uri])

    except Exception as e:
        print(e)


def test_run_sequence_multiprocess():
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    with Swarm(uris, factory=factory) as swarm:
        print('Connected to  Crazyflies')
        swarm.reset_estimators()
        print('Estimators have been reset')
        swarm.parallel_safe(start_states_log)
        print('Logging states info...')
        swarm.parallel_safe(run_sequence)