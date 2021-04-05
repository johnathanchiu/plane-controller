from kalman import KFilter
from controller import apply_kalman_controls, PID
from geometry import signed_rejection_dist

from definitions import XPlaneDefs
from xpc import XPlaneConnect

import multiprocessing
import argparse
import time
import math
import yaml

# state transition matrix
F = np.array([[1.0, 0.0, dt, 0.0],
              [0.0, 1.0, 0.0, dt],
              [0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 0.0, 1.0]])
# controls matrix
B = np.array([[0.5*dt*dt, 0],
              [0, 0.5 * dt * dt],
              [dt, 0],
              [0, dt]])
# measurements matrix
H = np.eye(4)
#
Q = np.eye(4) * 1000
# covariance matrix
R = np.eye(4) * 1000


def kalman_solver(queues, kalman, init_states, config, runway, center_line, terminal_velocity, plane_specs, solver_constraints):
    
    controls_queue, conditions_queue, states_queue, stop_queue = queues

    # TODO: fix this (only for testing purposes)
    winds = [[0, 0]]
    
    time_step = config['time_step']
    sim_size = config['simulation_size']
    runtime = config['simulation_steps']
    
    runway_heading = runway['runway_heading']
    end_x, end_z = runway['terminate_X'], runway['terminate_Z']
    origin_x, origin_z = runway['origin_X'], runway['origin_Z']
    desired_states = [runway_heading, terminal_velocity]

    # winds = conditions_queue.get()

    controls, cost = solve_states(init_states, desired_states, center_line, winds, plane_specs, solver_constraints, time_step=time_step, sim_time=sim_size)
    c_controls, k_controls = [[c[0], c[1]] for c in controls], np.array([controls[0][2], controls[0][3]])
    queue.put(c_controls)
    
    for _ in range(runtime):
        kalman.predict(k_controls)
        x, z, vx, vz = kalman.state
        pred_ground_speed = math.sqrt(vx**2 + vy**2)
        pred_heading = math.degrees(math.atan(vz / vx)) % 360
        dist = signed_rejection_dist(center_line, x - origin_x, z - origin_z)
        init_states = [dist, pred_ground_speed, pred_heading]
        controls, cost = solve_states(init_states, desired_states, center_line, winds, plane_specs, solver_constraints, time_step=time_step, sim_time=sim_size)
        c_controls, k_controls = [[c[0], c[1]] for c in controls], np.array([controls[0][2], controls[0][3]])
        controls_queue.put(c_controls)
        
        while not states_queue.empty():
            kalman.update(states_queue.get())
    
    stop_queue.put(1)
                
    
def controller(client, queues, rudder, throttle, config):
    controls_queue, stop_queue = queues
    
    control_sr = config['sample_time']
    time_step = config['time_step']
    receding_horizon = config['receding_horizon']
    
    apply = True
    control = None
    control_count = 0
    while apply:
        if not controls_queue.empty():
            control = controls_queue.get()
            control_count = receding_horizon
            immediate_control = control[:control_count]
            apply_controls(client, throttle, rudder, immediate_control, control_sr, time_step)
        else:
            if control not None:
                immediate_control = control[control_count]
                apply_controls(client, throttle, rudder, immediate_control, control_sr, time_step)
                control_count = max(control_count + 1, receding_horizon - 1)
        apply = stop_queue.empty()
        

def track_states(client, queues, runway, config):
    states_queue, stop_queue = queues

    origin_x, origin_z = runway['origin_x'], runway['origin_z']
    time_step = config['time_step']
    
    apply = True
    while apply:
        time.sleep(time_step)
        
        gs, psi, _, x, _, z = client.getDREFs(read_drefs)
        gs, psi, x, z = gs[0], psi[0], x[0], z[0]
        vx = gs * np.cos(np.radians(solver_heading(psi)))
        vz = gs * np.sin(np.radians(solver_heading(psi)))
        states_queue.put(np.array([x - origin_x, z - origin_z, vx, vz]))
        
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
    p2 = multiprocessing.Process(target=kalman_solver, args=(kalman_queues, kf, init_states, config, runway, center_line, terminal_velocity, plane_specs, solver_constraints))
    p3 = multiprocessing.Process(target=track_states, args=(xp_client, (states_queue, kill_queue), runway, config))
    p4 = multiprocessing.Process(target=controller, args=(xp_client, (controls_queue, stop_queue), rudder_controller, throttle_controller, control_sample, time_step, receding_horizon))
#    p1.start()
    p2.start()
    p3.start()
    p4.start()
#    p1.join()
    p2.join()
    p3.join()
    p4.join()
    
    
def setup(xp_client):
    # place plane at origin
    _, start_y, _ = xp_client.getDREFs(XPlaneDefs.position_dref)
    xp_client.sendDREFs(XPlaneDefs.position_dref, [origin_x, start_y[0], origin_z])
    time.sleep(0.1)
    
    gs, psi, _, x, _, z = client.getDREFs(read_drefs)
    gs, psi, x, z = gs[0], psi[0], x[0], z[0]
    time.sleep(0.1)
    
    # fix all systems
    xp_client.sendCOMM("sim/operation/fix_all_systems")
    # release park brake
    xp_client.sendDREF("sim/flightmodel/controls/parkbrake", 0)
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
    
    read_drefs = XPlaneDefs.control_dref + XPlaneDefs.position_dref
    xp_client = XPlaneConnect()
    
    gs, psi, x, z = setup(xp_client)
    dist = signed_rejection_dist(center_line, x - origin_x, z - origin_z)
    center_line = np.array([end_x - origin_x, end_z - origin_z])
    init_states = [dist, gs, psi]
    
    vx = gs * np.cos(np.radians(solver_heading(psi)))
    vz = gs * np.sin(np.radians(solver_heading(psi)))

    kf = KFilter(F, B, Q, H, R, state=np.array([x, z, vx, vz]))
    throttle_controller = PID(2.0, 0.0, 1.0, 10.0, config['sample_time'])
    rudder_controller = PID(0.3, 0.4, 1.5, 10.0, config['sample_time'])
    
    run_controller()
    
