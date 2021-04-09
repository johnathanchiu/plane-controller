import sys
sys.path.append('..')

from kalman import KFilter, find_kalman_controls
from controller import apply_kalman_controls, PID
from geometry import signed_rejection_dist, quaternion_for
from simple_solver import solve_states2

from definitions import XPlaneDefs
from xpc import XPlaneConnect

import multiprocessing
import numpy as np
import argparse
import time
import math
import yaml


def kalman_solver(queues, kalman, init_states, config, runway, center_line):
    
    controls_queue, conditions_queue, states_queue, stop_queue = queues

    # TODO: fix this (only for testing purposes)
    winds = [[0, 0]]
    
    runtime = config['simulation_steps']
    time_step, sim_size = config['time_step'], config['simulation_size']
    receding_horizon = config['receding_horizon']
    
    terminal_velocity = config['terminal_velocity']
    plane_specs = [config['plane_cs'], config['plane_mass'], config['plane_half_length']]
    acceleration_constraint = config['acceleration_constraint']
    turning_constraint = config['turning_constraint']
    solver_constraints = (acceleration_constraint, turning_constraint)
    
    runway_heading = runway['runway_heading']
    end_x, end_z = runway['terminate_X'], runway['terminate_Z']
    origin_x, origin_z = runway['origin_X'], runway['origin_Z']
    desired_states = [runway_heading, terminal_velocity]

    # winds = conditions_queue.get()

    controls, cost = solve_states2(init_states, desired_states, center_line, winds, plane_specs, solver_constraints, time_step=time_step, sim_time=sim_size)
    c_controls = [[c[0], c[1]] for c in controls] 
    ax, az = find_kalman_controls(controls[0][2], init_states[-1])
    k_controls = np.array([ax, az])
    controls_queue.put(c_controls)
    
    start, next_input = time.time(), time.time()
    while time.time() - start < runtime:
        kalman.predict(k_controls)
        x, z, pred_ground_speed, pred_heading = kalman.get_state()
        print("KALMAN PREDICTION:")
        print("X, Z: {}, {}".format(x, z))
        print("ground speed, heading: {}, {}".format(pred_ground_speed, pred_heading))
        dist = signed_rejection_dist(center_line, x, z)
        init_states = [dist, pred_ground_speed, pred_heading]
        controls, cost = solve_states2(init_states, desired_states, center_line, winds, plane_specs, solver_constraints, time_step=time_step, sim_time=sim_size)
        c_controls = [[c[0], c[1]] for c in controls]
        ax, az = find_kalman_controls(controls[0][2], init_states[-1])
        k_controls = np.array([ax, az])
        time.sleep(max(0, receding_horizon - (time.time() - next_input)))
        controls_queue.put(c_controls)
        
        while not states_queue.empty():
            measurement = states_queue.get()
            x, z, vx, vz = measurement
            me_gs = math.sqrt(vx**2 + vz**2)
            me_head = math.degrees(math.atan(vz / vx)) % 360 - 270
            print("MEASUREMENTS:")
            print("X, Z: {}, {}".format(x, z))
            print("ground speed, heading: {}, {}".format(me_gs, me_head))
            kalman.update(measurement)

        next_input = time.time()
        
        print("----------------------------------------------")

    stop_queue.put(1)
                
    
def controller(client, queues, rudder, throttle, runway, config):
    controls_queue, states_queue, stop_queue = queues

    read_drefs = XPlaneDefs.control_dref + XPlaneDefs.position_dref
    
    origin_x, origin_z = runway['origin_X'], runway['origin_Z']
    control_sr = config['sample_time']
    time_step = config['time_step']
    receding_horizon = config['receding_horizon']
    
    apply = True
    control = None
    control_count = 0
    prev_sample = time.time()
    while apply:
        if time.time() - prev_sample >= time_step:
            gs, psi, _, x, _, z = client.getDREFs(read_drefs)
            gs, psi, x, z = gs[0], psi[0], x[0], z[0]
            vx, vz = find_kalman_controls(gs, psi)
            states_queue.put(np.array([x - origin_x, z - origin_z, vx, vz]))
            prev_sample = time.time()
        if not controls_queue.empty():
            control_count = 0
            control = controls_queue.get()
            immediate_control = control[control_count]
            prev_sample = apply_kalman_controls(client, states_queue, prev_sample, (origin_x, origin_z), throttle, 
                                                rudder, immediate_control, control_sr, time_step)
        else:
            if control is not None:
                immediate_control = control[control_count]
                prev_sample = apply_kalman_controls(client, states_queue, prev_sample, (origin_x, origin_z), throttle, 
                                                    rudder, immediate_control, control_sr, time_step)
                control_count = max(control_count + 1, len(control) - 1)
        apply = stop_queue.empty()

        
def update_environment(wind_speed, wind_heading):
    pass
    
    
def run_controller():
    controls_queue = multiprocessing.Queue()
    states_queue = multiprocessing.Queue()
    environment_queue = multiprocessing.Queue()
    kill_queue = multiprocessing.Queue()
    
    kalman_queues = (controls_queue, environment_queue, states_queue, kill_queue)
    
#    p1 = multiprocessing.Process(target=update_environment, args=())
    p2 = multiprocessing.Process(target=kalman_solver, args=(kalman_queues, kf, init_states, config, runway, center_line))
    p3 = multiprocessing.Process(target=controller, args=(xp_client, (controls_queue, states_queue, kill_queue), rudder_controller, 
                                 throttle_controller, runway, config))
#    p1.start()
    p2.start()
    p3.start()
#    p1.join()
    p2.join()
    p3.join()
    
    
def setup(client, runway, init_phi, init_theta, fuel):

    init_phi, init_theta = math.radians(init_phi), math.radians(init_theta)

    starting_elevation = runway['starting_elevation']
    runway_heading = runway['runway_heading']

    read_drefs = XPlaneDefs.control_dref + XPlaneDefs.position_dref

    xp_client.sendDREFs(XPlaneDefs.velocity_drefs, [0]*len(XPlaneDefs.velocity_drefs))
    xp_client.sendCTRL([0, 0, 0, 0])
    xp_client.sendDREF(XPlaneDefs.park_dref, 1)
    quaternion = quaternion_for(init_theta, init_phi, math.radians(runway_heading))
    xp_client.sendDREF(XPlaneDefs.quaternion_dref, quaternion)
    time.sleep(0.1)

    # place plane at origin
    xp_client.sendDREFs(XPlaneDefs.position_dref, [origin_x, starting_elevation, origin_z])
    time.sleep(0.1)
 
    gs, psi, _, x, _, z = client.getDREFs(read_drefs)
    gs, psi, x, z = gs[0], psi[0], x[0], z[0]
    time.sleep(0.1)

    xp_client.sendDREF(XPlaneDefs.fuel_dref, fuel)
    
    # fix all systems
    xp_client.sendCOMM(XPlaneDefs.fix_systems)

    # release park brake
    xp_client.sendDREF(XPlaneDefs.park_dref, 0)
    time.sleep(0.1)
    
    return gs, psi, x, z


def load_yaml(filename):
    with open(filename, 'r') as stream:
        options = yaml.safe_load(stream)
    return options
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--runway_config', help='runway configuration file', default='../runway.yaml')
    parser.add_argument('-c', '--controller_config', help='PID configuration file', default='controller.yaml')
    args = parser.parse_args()
    
    runway = load_yaml(args.runway_config)
    config = load_yaml(args.controller_config)

    fuel_tank = config['fuel']
    init_phi, init_theta = config['init_phi'], config['init_theta']

    dt = config['time_step']
    # state transition matrix
    F = np.array([[1.0, 0.0, dt, 0.0],
                  [0.0, 1.0, 0.0, dt],
                  [0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 1.0]])
    # controls matrix
    B = np.array([[0.5*dt*dt, 0],
                  [0, 0.5*dt*dt],
                  [0.1 * dt, 0],
                  [0, 0.1 * dt]])
    # measurements matrix
    H = np.eye(4)
    # uncertainty for prediction
    Q = np.eye(4) * 10000
    # uncertainty for measurements
    R = np.eye(4) * 100

    xp_client = XPlaneConnect()

    end_x, end_z = runway['terminate_X'], runway['terminate_Z']
    origin_x, origin_z = runway['origin_X'], runway['origin_Z']
    starting_elevation = runway['starting_elevation']
    
    gs, psi, x, z = setup(xp_client, runway, init_phi, init_theta, fuel_tank)
    center_line = np.array([end_x - origin_x, end_z - origin_z])
    dist = signed_rejection_dist(center_line, x - origin_x, z - origin_z)
    init_states = [dist, gs, psi]
    
    vx, vz = find_kalman_controls(gs, psi)

    kf = KFilter(F, B, Q, H, R, state=np.array([x - origin_x, z - origin_z, vx, vz]))
    throttle_controller = PID(2.0, 0.0, 1.0, 10.0, config['sample_time'])
    rudder_controller = PID(0.3, 0.4, 1.5, 10.0, config['sample_time'])
    
    run_controller()
    
