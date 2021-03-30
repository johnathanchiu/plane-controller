import numpy as np
import scipy.optimize as opt

from multiprocessing import Pool

from geometry import rejection_dist

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
        c_tup = (prev_velocity, prev_heading)
        controls.append(c_tup)
    return controls


def compute_states(init_state, controls, wind_dynamics, plane_specs, time_step=1):
    states = []
    x, y, v, h = init_state
    wind_speed, wind_heading = wind_dynamics
    plane_cs, plane_mass, plane_half_length = plane_specs
    wind_force = wind_speed * plane_cs
    wind_acc = wind_force / plane_mass
    for i in range(0, len(controls), 2):
        a, w = controls[i], controls[i+1]
        states = np.concatenate((states, [x, y, v, h, a, w]))
        vx = (v * np.cos(np.radians(h))) + (time_step * wind_acc * np.cos(np.radians(wind_heading)))
        vy = (v * np.sin(np.radians(h))) + (time_step * wind_acc * np.sin(np.radians(wind_heading)))
        x += time_step * vx
        y += time_step * vy

        wind_torque = plane_half_length * wind_force * np.sin(np.radians(wind_heading - h))
        wind_w = time_step * (wind_torque / plane_mass) * plane_half_length

        v += time_step * a
        h += time_step * (w + wind_w)
        h %= 360
    return states
    

def cost_sum(wrapped_args):

    state_weight = 1
    constraint_weight = 1
    control_weight = 0.01
    dstate_weight = 5
    fvelocity_weight = 3

    init_states, desired_states, params, env, plane_specs, time_step = wrapped_args
    desired_x, desired_y, desired_v = desired_states
    states = compute_states(init_states, params, env, plane_specs, time_step=time_step)
    # cost = control_weight * (params[0] ** 2 + params[1] ** 2) ** 2
    cost = 0
    for i in range(6, len(states) - 6, 6):
        px, py, v, h, a, w = states[i:i+6]
        # closest centerline point
        cost += constraint_weight * rejection_dist(desired_x, desired_y, px, py)
        # cost += control_weight * (a ** 2 + w ** 2) ** 2
        # cost += state_weight * np.sqrt((desired_x - px) ** 2 + (desired_y - py) ** 2)
        # cost += fvelocity_weight * abs(desired_v - v)
    px, py, v, h, a, w = states[-6:]
    cost += dstate_weight * np.sqrt((desired_x - px) ** 2 + (desired_y - py) ** 2)
    cost += fvelocity_weight * abs(desired_v - v)
    return cost


def formulate_objective(init_states, desired_states, environment, plane_specs, time_step=1):

    desired_x, desired_y, desired_v = desired_states
    
    def objective(params):
        # wrap_args = [(init_states, desired_states, params, env, plane_specs, time_step) for env in environment]
        # with Pool(8) as p:
        #     cost = sum(p.map(cost_sum, wrap_args))
        # return cost / len(environment)
        
        state_weight = 1
        constraint_weight = 1
        control_weight = 0.01
        dstate_weight = 5
        fvelocity_weight = 10
        
        cost = 0
        for env in environment:
            states = compute_states(init_states, params, env, plane_specs, time_step=time_step)
            cost += control_weight * np.linalg.norm(np.vstack([params[0], params[1]]), ord=2) ** 2
            for i in range(6, len(states) - 6, 6):
                px, py, v, h, a, w = states[i:i+6]
                # closest centerline point
                cost += constraint_weight * rejection_dist(desired_x, desired_y, px, py)
                # minimize controls
                cost += control_weight * np.linalg.norm(np.vstack([a, w]), ord=2) ** 2
                # minimize distance to final states
                cost += state_weight * np.linalg.norm(np.vstack([desired_x - px, desired_y - py]), ord=2)
                # close gap between desired velocity
                cost += fvelocity_weight * np.linalg.norm([desired_v - v], ord=2)
            px, py, v, h, a, w = states[-6:]
            cost += dstate_weight * np.linalg.norm(np.vstack([desired_x - px, desired_y - py]), ord=2)
            cost += fvelocity_weight * np.linalg.norm([desired_v - v], ord=2)
        return cost / len(environment)
    return objective


def formulate_guess(sim_time):
    return np.zeros((sim_time*2,))


def solve_states(initial_states, desired_states, extern_conditions, plane_specs, acceleration_constraint,
                 turning_constraint, time_step=1, sim_time=10):

    init_x, init_y, init_velocity, init_heading = initial_states
    desired_x, desired_y, desired_velocity = desired_states

    state0 = initial_states
    init_guess = formulate_guess(sim_time)
    bounds = [(-acceleration_constraint, acceleration_constraint),
              (-turning_constraint, turning_constraint)] * sim_time

    obj = formulate_objective(state0, [desired_x, desired_y, desired_velocity], extern_conditions, plane_specs, time_step=time_step)

    start_time = time.time()
    result = opt.minimize(obj, init_guess, method='SLSQP', bounds=bounds,
                          options={'eps': 0.01, 'maxiter': 100})
    print('----', time.time() - start_time, 'seconds ----')

    states = compute_states(initial_states, result.x, extern_conditions[0], plane_specs, time_step=time_step)
    states = get_states_controls(states)
    return get_controls(states, time_step), result.fun
