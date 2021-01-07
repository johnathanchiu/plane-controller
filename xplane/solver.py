import numpy as np
import scipy.optimize as opt

import random


get_controls = lambda x: [x[i+2:i+4] for i in range(0, len(x), 6)]


def compute_states(init_state, controls, wind_dynamics, plane_specs, time_step=1):
    states = []
    x, y, v, h = init_state
    wind_speed, wind_heading = wind_dynamics
    plane_cs, plane_mass = plane_specs
    wind_acc = wind_speed * plane_cs / plane_mass
    for i in range(0, len(controls), 2):
        a, w = controls[i:i+2]
        states = np.concatenate((states, [x, y, v, h, a, w]))
        vx = (v * np.cos(np.radians(h))) + (time_step * wind_acc * np.cos(np.radians(wind_heading)))
        vy = (v * np.sin(np.radians(h))) + (time_step * wind_acc * np.sin(np.radians(wind_heading)))
        x += time_step * vx
        y += time_step * vy
        v += time_step * a
        h += time_step * w
        h %= 360
    return states


def rejection_dist(desired_x, desired_y, curr_x, curr_y):
    # TODO: fix polyline to start from an initial given point
    a = np.array([desired_x, desired_y])
    b = np.array([curr_x, curr_y])
    projection = a@b / np.linalg.norm(a, ord=2)**2 * a
    proj_x, proj_y = projection
    return np.linalg.norm([curr_x - proj_x, curr_y - proj_y], ord=2) ** 2


def formulate_objective(init_states, desired_states, environment, plane_specs, time_step=1, state_weight=0.1,
                        control_weight=0.1, constraint_weight=0.1, dstate_weight=0.001, fvelocity_weight=1.0):
    desired_x, desired_y, desired_v = desired_states
    def objective(params):
        states = compute_states(init_states, params, environment, plane_specs, time_step=time_step)
        cost = control_weight * np.linalg.norm(np.vstack([params[0], params[1]]), ord=2) ** 2
        for i in range(6, len(states) - 6, 6):
            px, py, v, h, a, w = states[i:i+6]
            # closest centerline point
            cost += constraint_weight * rejection_dist(desired_x, desired_y, px, py)
            cost += control_weight * np.linalg.norm(np.vstack([a, w]), ord=2) ** 2
            cost += state_weight * np.linalg.norm(np.vstack([desired_x - px, desired_y - py]), ord=2)
            cost += fvelocity_weight * np.linalg.norm([desired_v - v], ord=2)
        px, py, v, h, a, w = states[-6:]
        cost += dstate_weight * np.linalg.norm(np.vstack([desired_x - px, desired_y - py]), ord=2)
        cost += fvelocity_weight * np.linalg.norm([desired_v - v], ord=2)
        return cost
    return objective


def formulate_guess(sim_time):
    return np.zeros((sim_time*2,))


def solve_states(initial_states, desired_states, extern_conditions, plane_specs, acceleration_constraint,
                    turning_constraint, time_step=1, sim_time=10):

    init_x, init_y, init_velocity, init_heading = initial_states
    desired_x, desired_y, desired_velocity = desired_states

    state0 = [init_x, init_y, init_velocity, init_heading]
    init_guess = formulate_guess(sim_time)
    bounds = [(-acceleration_constraint, acceleration_constraint),
              (-turning_constraint, turning_constraint)] * sim_time

    obj = formulate_objective(state0, [desired_x, desired_y, desired_velocity], extern_conditions, plane_specs,
                              time_step=time_step, state_weight=1, constraint_weight=1, control_weight=2,
                              dstate_weight=5, fvelocity_weight=10)

    result = opt.minimize(obj, init_guess, method='SLSQP', bounds=bounds,
                          options={'eps': 0.01, 'maxiter': 1000})
    states = compute_states(initial_states, result.x, extern_conditions, time_step=time_step)
    return get_controls(states)[1:], result.success, result.message
