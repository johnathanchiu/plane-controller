import numpy as np
import scipy.optimize as opt

from multiprocessing import Pool

from definitions import XPlaneDefs
from geometry import signed_rejection_dist, rotate, solver_heading, true_heading

import random
import time


get_states_controls = lambda x: [x[i+2:i+6] for i in range(0, len(x), 6)]

def get_controls(states, time_step):
    controls = []
    prev_heading = states[0][1]
    prev_velocity = states[0][0]
    for state in states:
        prev_heading += state[-1] * time_step
        prev_velocity += state[-2] * time_step
        c_tup = (prev_velocity, true_heading(prev_heading))
        controls.append(c_tup)
    return controls


def compute_states(init_state, controls, wind_dynamics, plane_specs, time_step=1):
    
    wind_speed, wind_heading = wind_dynamics
    wind_heading = solver_heading(wind_heading)
    plane_cs, plane_mass, plane_half_length = plane_specs
    wind_force = wind_speed * plane_cs
    wind_acc = wind_force / plane_mass
    
    states = []
    x, y, v, h = init_state

    for i in range(0, len(controls), 2):
        
        a, w = controls[i:i+2]
        states = np.concatenate((states, [x, y, v, h, a, w]))
        
        vx = (v * np.cos(np.radians(h))) + (time_step * wind_acc * np.cos(np.radians(wind_heading)))
        vy = (v * np.sin(np.radians(h))) + (time_step * wind_acc * np.sin(np.radians(wind_heading)))
        
        x += time_step * vx
        y += time_step * vy
        
        # rF * sin(theta)
        wind_torque = np.abs(plane_half_length * wind_force * np.sin(np.radians(wind_heading - h)))
        wind_w = 0.1 * time_step * (wind_torque / plane_mass) * plane_half_length
        v += time_step * a
        h += time_step * (w + wind_w)
        
        h %= 360

    return states


def formulate_objective(init_state, center_line, desired_states, environment, plane_specs, time_step=1):
    
    control_weight = 0.1
    centerline_weight = 100
    velocity_weight = 3
    heading_weight = 5
    
    desired_h, desired_v = desired_states
    
    def objective(params):
        cost = 0
        for env in environment:
            states = compute_states(init_state, params, env, plane_specs, time_step=time_step)
            for i in range(6, len(states), 6):
                px, py, v, h, a, w = states[i:i+6]
                cost += centerline_weight * np.abs(signed_rejection_dist(center_line, px, py))
                cost += velocity_weight * np.linalg.norm([desired_v - v], ord=2)
                cost += heading_weight * np.linalg.norm([desired_h - h], ord=2)
        return cost / len(environment)
    
    return objective


def formulate_guess(sim_time):
    return np.zeros((sim_time*2,))


def solve_states(initial_states, desired_states, center_line, extern_conditions, plane_specs, acceleration_constraint, turning_constraint, time_step=1, sim_time=10):

    rejection, init_velocity, init_heading = initial_states
    desired_heading, desired_velocity = desired_states
    
    init_heading = solver_heading(init_heading)
    
    init_guess = formulate_guess(sim_time)
    bounds = [(-acceleration_constraint, acceleration_constraint),
              (-turning_constraint, turning_constraint)] * sim_time
              
    rinit_x, rinit_y = rotate(0, rejection, solver_heading(desired_heading) - 360)
    state0 = [rinit_x, rinit_y, init_velocity, init_heading]

    obj = formulate_objective(state0, center_line, [solver_heading(desired_heading), desired_velocity], extern_conditions, plane_specs, time_step=time_step)
    
    start_time = time.time()
    result = opt.minimize(obj, init_guess, method='SLSQP', bounds=bounds,
                          options={'eps': 0.1, 'maxiter': 300})
    print('----', time.time() - start_time, 'seconds ----')

    states = compute_states(state0, result.x, extern_conditions[0], plane_specs, time_step=time_step)
    states = get_states_controls(states)
    return get_controls(states, time_step), result.fun
