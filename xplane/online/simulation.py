import sys
sys.path.append('..')

from kalman import KFilter, find_kalman_controls
from controller import apply_kalman_controls, PID
from geometry import signed_rejection_dist, quaternion_for, kn_to_ms
from simple_solver import solve_states2

from definitions import XPlaneDefs
from xpc import XPlaneConnect

import multiprocessing
import numpy as np
import argparse
import time
import math
import yaml


def kalman_solver(client, queues, kalman, init_states, config, runway, center_line):
    
    controls_queue, conditions_queue, states_queue, data_queue, stop_queue = queues
    
    run_time = config['satisfy_steps']
    abort_time = config['abort_steps']
    time_step, sim_size = config['time_step'], config['simulation_size']
    receding_horizon = config['receding_horizon']
    
    terminal_velocity = config['terminal_velocity'] + 5
    plane_specs = [config['plane_cs'], config['plane_mass'], config['plane_half_length']]
    acceleration_constraint = config['acceleration_constraint']
    turning_constraint = config['turning_constraint']
    solver_constraints = (acceleration_constraint, turning_constraint)
    
    runway_heading = runway['runway_heading']
    end_x, end_z = runway['terminate_X'], runway['terminate_Z']
    origin_x, origin_z = runway['origin_X'], runway['origin_Z']
    desired_states = [runway_heading, terminal_velocity]

    winds = [[0, 0]]
    if not conditions_queue.empty():
        winds = conditions_queue.get()

    controls, cost = solve_states2(init_states, desired_states, center_line, winds, plane_specs, solver_constraints, time_step=time_step, sim_time=sim_size)
    c_controls = [[c[0], c[1]] for c in controls] 
    accs = [c[2] for c in controls]
    omegas = [c[3] for c in controls]
    controls_queue.put(c_controls)

    need_abort_takeoff = True
    breaks_set = False
    
    start, next_input = time.time(), time.time()
    while time.time() - start < run_time + abort_time and need_abort_takeoff:
        pred_heading = init_states[-1]
        for i in range(int(receding_horizon / time_step)): 
            ax, az = find_kalman_controls(accs[i], pred_heading + omegas[i] * time_step, winds[0], plane_specs[:-1])
            kalman.predict(np.array([ax, az]))
            _, _, pred_ground_speed, pred_heading = kalman.get_state()
        x, z, pred_ground_speed, pred_heading = kalman.get_state()
        # print("----------------------------------------------")
        # print("KALMAN PREDICTION:")
        # print("X, Z: {}, {}".format(x, z))
        # print("ground speed, heading: {}, {}".format(pred_ground_speed, pred_heading))
        # print("----------------------------------------------\n")
        dist = signed_rejection_dist(center_line, x, z)
        init_states = [dist, pred_ground_speed, pred_heading]
        if not conditions_queue.empty():
            winds = conditions_queue.get()
        controls, cost = solve_states2(init_states, desired_states, center_line, winds, plane_specs, solver_constraints, time_step=time_step, sim_time=sim_size)
        c_controls = [[c[0], c[1]] for c in controls]
        accs = [c[2] for c in controls]
        omegas = [c[3] for c in controls]
        time.sleep(max(0, receding_horizon - (time.time() - next_input)))
        controls_queue.put(c_controls)
        
        while not states_queue.empty():
            measurement = states_queue.get()
            x, z, vx, vz = measurement
            me_gs = math.sqrt(vx**2 + vz**2)
            me_head = math.degrees(math.atan(vz / vx)) % 360 - 270
            # print("----------------------------------------------")
            # print("MEASUREMENTS:")
            # print("X, Z: {}, {}".format(x, z))
            # print("ground speed, heading: {}, {}".format(me_gs, me_head))
            # print("----------------------------------------------\n")
            kalman.update(measurement)

        check_dist = abs(signed_rejection_dist(center_line, x, z))
        check_heading = abs(runway_heading - me_head)
        check_velocity = me_gs - terminal_velocity

        data_queue.put((time.time() - start, cost, me_head, me_gs, x, z, winds[0][0], winds[0][1]))
        
        if check_dist < 2 and check_heading < 2 and check_velocity > -5:
            # takeoff presumably successful
            print("Takeoff success with conditions:")
            print(f"groundspeed: {me_gs}, heading: {me_head}, cte: {check_dist}")
            need_abort_takeoff = False
            
        if time.time() - start > run_time:
            # set abort takeoff
            # set parking break when velocity is low
            if not breaks_set:
                print("Aborting takeoff, conditions not satisfied")
                break_set = True
                client.sendDREF(XPlaneDefs.park_dref, 1)
            if abs(me_gs) < 0.3:
                break

            desired_states = [runway_heading, 0]

        next_input = time.time()

    if need_abort_takeoff: 
        data_queue.put(0)
    else:
        # takeoff should be successful
        data_queue.put(1)

    print(time.time() - start)

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
                control_count = min(control_count + 1, len(control) - 1)
        apply = stop_queue.empty()

        
def update_environment(client, queues, wind_speed_bounds, wind_heading_bounds, num_samples, runway):

    environment_queue, stop_queue = queues
    runway_heading = runway['runway_heading']

    lb_ws, ub_ws = wind_speed_bounds
    lb_wh, ub_wh = wind_heading_bounds

    apply = True
    while apply:

        wind_speeds = np.random.randint(lb_ws, ub_ws, size=num_samples+1).tolist()
        wind_headings = np.random.randint(lb_wh, ub_wh, size=num_samples+1).tolist()

        wind_speed = wind_speeds[0]
        wind_heading = wind_headings[0]

        # print("----------------------------------------------------")
        # print("Using (wind speed, wind heading):", wind_speed, wind_heading + runway_heading)
        # print("----------------------------------------------------\n")

        xp_wind_direction = -1 * wind_heading + runway_heading # since positive rotation to right
        xp_wind_direction += 180 # wind is counter clockwise of true north
        
        client.sendDREFs(XPlaneDefs.condition_drefs, [0, xp_wind_direction, wind_speed])
        wind_headings = [h + runway_heading for h in wind_headings]
        wind_speeds = [kn_to_ms(s) for s in wind_speeds]
        environment_queue.put(list(zip(wind_speeds, wind_headings)))
        apply = stop_queue.empty()

        time.sleep(5)
    
    
def run_controller():
    controls_queue = multiprocessing.Queue()
    states_queue = multiprocessing.Queue()
    environment_queue = multiprocessing.Queue()
    kill_queue = multiprocessing.Queue()
    data_queue = multiprocessing.Queue()
    
    kalman_queues = (controls_queue, environment_queue, states_queue, data_queue, kill_queue)
    
    p1 = multiprocessing.Process(target=update_environment, args=(xp_client, (environment_queue, kill_queue), ws_bound, wh_bound, num_samples, runway))
    p2 = multiprocessing.Process(target=kalman_solver, args=(xp_client, kalman_queues, kf, init_states, config, runway, center_line))
    p3 = multiprocessing.Process(target=controller, args=(xp_client, (controls_queue, states_queue, kill_queue), rudder_controller, 
                                 throttle_controller, runway, config))
    p1.start()
    p2.start()
    p3.start()
    p1.join()
    p2.join()
    p3.join()

    results = []
    while not data_queue.empty():
        results.append(data_queue.get())
    decision = results[-1]
    results = results[:-1]

    return results, decision
    
    
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
    receding_horizon = config['receding_horizon']
    sim_size = config['simulation_size']
    assert receding_horizon % dt == 0.0, 'receding_horizon must be a multiple of time_step'
    assert dt * sim_size >= receding_horizon, 'not enough states computed to reach horizon'

    # state transition matrix
    F = np.array([[1.0, 0.0, dt, 0.0],
                  [0.0, 1.0, 0.0, dt],
                  [0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 1.0]])
    # controls matrix
    B = np.array([[0.5 * dt * dt, 0],
                  [0, 0.5 * dt * dt],
                  [0.6 * dt, 0],
                  [0, 0.6 * dt]])
    # measurements matrix
    H = np.eye(4)
    # uncertainty for prediction
    Q = np.eye(4) * 10000
    # covariant noise
    N = np.random.randint(1, 100, size=(4, 4))
    np.fill_diagonal(N, 0)
    Q = Q + N
    # uncertainty for measurements
    R = np.eye(4) * 5

    xp_client = XPlaneConnect()

    end_x, end_z = runway['terminate_X'], runway['terminate_Z']
    origin_x, origin_z = runway['origin_X'], runway['origin_Z']
    starting_elevation = runway['starting_elevation']

    throttle_controller = PID(2.0, 0.0, 1.0, 10.0, config['sample_time'])
    rudder_controller = PID(0.3, 0.4, 1.5, 10.0, config['sample_time'])
    
    num_samples = config['environment_samples'] 
    ws_bound = config['windspeed_bounds']
    wh_bound = config['wind_heading_bounds']

    center_line = np.array([end_x - origin_x, end_z - origin_z])
       
    kf = KFilter(F, B, Q, H, R)

    no_sim_runs = 100

    num_samples = 0
    log = open("sample0.csv", "w")
    log.write("sample_no,data,prediction\n")
    
    for i in range(no_sim_runs):  
        gs, psi, x, z = setup(xp_client, runway, init_phi, init_theta, fuel_tank)
        dist = signed_rejection_dist(center_line, x - origin_x, z - origin_z)
        init_states = [dist, gs, psi]
        vx, vz = find_kalman_controls(gs, psi)
        kf.reset()
        kf.set_state(np.array([x - origin_x, z - origin_z, vx, vz]))
        sim_results, takeoff_decision = run_controller()
        print(f"Simulation number: {i+1} with results: {takeoff_decision}")
        log.write(f"{i},{sim_results},{takeoff_decision}\n")
    log.close()

    num_samples = 5
    log = open("sample5.csv", "w")
    log.write("sample_no,data,prediction\n")

    for i in range(no_sim_runs):
        gs, psi, x, z = setup(xp_client, runway, init_phi, init_theta, fuel_tank)
        dist = signed_rejection_dist(center_line, x - origin_x, z - origin_z)
        init_states = [dist, gs, psi]
        vx, vz = find_kalman_controls(gs, psi)
        kf.reset()
        kf.set_state(np.array([x - origin_x, z - origin_z, vx, vz]))
        sim_results, takeoff_decision = run_controller()
        print(f"Simulation Number: {i+1}")
        print(f"Results: {takeoff_decision}")
        log.write(f"{i},{sim_results},{takeoff_decision}\n")

    log.close()

    num_samples = 10
    log = open("sample5.csv", "w")
    log.write("sample_no,data,prediction\n")

    for i in range(no_sim_runs):
        gs, psi, x, z = setup(xp_client, runway, init_phi, init_theta, fuel_tank)
        dist = signed_rejection_dist(center_line, x - origin_x, z - origin_z)
        init_states = [dist, gs, psi]
        vx, vz = find_kalman_controls(gs, psi)
        kf.reset()
        kf.set_state(np.array([x - origin_x, z - origin_z, vx, vz]))
        sim_results, takeoff_decision = run_controller()
        print(f"Simulation Number: {i+1}")
        print(f"Results: {takeoff_decision}")
        log.write(f"{i},{sim_results},{takeoff_decision}\n")

    log.close()


