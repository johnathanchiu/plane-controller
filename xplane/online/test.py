import multiprocessing
import time
import math

from kalman import KFilter

from definitions import XPlaneDefs


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
H = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0]])
# measurement noise matrix
Q = np.eye(4) * 10
# covariance matrix
R = np.eye(2) * 10


def kalman_solver(queues, kalman, init_states, runway, plane_specs, terminal_velocity, solver_constraints, time_step, sim_size, runtime):
    
    controls_queue, conditions_queue, states_queue, stop_queue = queues
    # TODO: fix this (only for testing purposes)
    winds = [[0, 0]]
    
    constraints = solver_constraints
    runway_heading = runway['runway_heading']
    end_x, end_z = runway['end_x'], runway['end_z']
    origin_x, origin_z = runway['origin_x'], runway['origin_z']
    center_line = np.array([end_x - origin_x, end_z - origin_z])
    desired_states = [runway_heading, terminal_velocity]

    # winds = conditions_queue.get()

    controls, cost = solve_states(init_states, desired_states, center_line, winds, plane_specs, constraints, time_step=time_step, sim_time=sim_size)
    c_controls, k_controls = [[c[0], c[1]] for c in controls], np.array([controls[0][2], controls[0][3]])
    queue.put(c_controls)
    
    for _ in range(runtime):
        kalman.predict(k_controls)
        x, z, vx, vz = kalman.state
        pred_ground_speed = math.sqrt(vx**2 + vy**2)
        pred_heading = math.degrees(math.atan(vz / vx)) % 360
        dist = signed_rejection_dist(center_line, x - origin_x, z - origin_z)
        init_states = [dist, pred_ground_speed, pred_heading]
        controls, cost = solve_states(init_states, desired_states, center_line, winds, plane_specs, constraints, time_step=time_step, sim_time=sim_size)
        c_controls, k_controls = [[c[0], c[1]] for c in controls], np.array([controls[0][2], controls[0][3]])
        controls_queue.put(c_controls)
        
        while not states_queue.empty():
            kalman.update(states_queue.get())
    
    stop_queue.put(1)
                
    
def controller(client, queues, rudder, throttle, control_sr, time_step, receding_horizon):
    controls_queue, stop_queue = queues
    
    apply = True
    control = None
    while apply:
        if not controls_queue.empty():
            control = controls_queue.get()
            apply_controls(client, throttle, rudder, control, control_sr, time_step, receding_horizon)
        else:
            if control not None:
                apply_controls(client, throttle, rudder, control[-1:], control_sr, time_step, receding_horizon)
        apply = stop_queue.empty()
        

def track_states(client, runway, queues, time_step):
    states_queue, stop_queue = queues

    origin_x, origin_z = runway['origin_x'], runway['origin_z']
    
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
    p2 = multiprocessing.Process(target=kalman_solver, args=(kalman_queues, kalman, init_states, runway, plane_specs, terminal_velocity, solver_constraints, time_step, sim_size, runtime))
    p3 = multiprocessing.Process(target=controller, args=(xp_client, (controls_queue, stop_queue), rudder_controller, throttle_controller, control_sample, time_step, receding_horizon))
#    p1.start()
    p2.start()
    p3.start()
#    p1.join()
    p2.join()
    p3.join()
    

if __name__ == '__main__':
    xp_client = XPlaneConnect()
    read_drefs = XPlaneDefs.control_dref + XPlaneDefs.position_dref
    
    gs, psi, _, x, _, z = client.getDREFs(read_drefs)
    gs, psi, x, z = gs[0], psi[0], x[0], z[0]
    dist = signed_rejection_dist(center_line, x - origin_x, z - origin_z)
    init_states = [dist, gs, psi]
    
    # TODO: fix this
    kf = KFilter(F, B, Q, H, R, state=[x, z, ...])
    run_controller()
    
