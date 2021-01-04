import numpy as np
import scipy.optimize as opt

import random


get_controls = lambda x: [x[i+2:i+4] for i in range(0, len(x), 6)]


def compute_states(init_state, controls, time_step=1):
    states = []
    x, y, v, h = init_state
    for i in range(0, len(controls), 2):
        a, w = controls[i:i+2]
        states = np.concatenate((states, [x, y, v, h, a, w]))
        x += v * time_step * np.cos(np.radians(h))
        y += v * time_step * np.sin(np.radians(h))
        v += a * time_step
        h += w * time_step
    return states


def rejection_dist(desired_x, desired_y, curr_x, curr_y):
    a = np.array([desired_x, desired_y])
    b = np.array([curr_x, curr_y])
    projection = a@b / np.linalg.norm(a, ord=2)**2 * a
    proj_x, proj_y = projection
    return np.linalg.norm([curr_x - proj_x, curr_y - proj_y], ord=2) ** 2


def formulate_objective(init_states, desired_states, time_step=1, state_weight=0.1,
                        control_weight=0.1, constraint_weight=0.1, dstate_weight=0.001):
    desired_x, desired_y, desired_v = desired_states
    def objective(params):
        states = compute_states(init_states, params, time_step=time_step)
        cost = control_weight * np.linalg.norm(np.vstack([params[0], params[1]]), ord=2) ** 2
        for i in range(6, len(states) - 6, 6):
            px, py, v, h, a, w = states[i:i+6]
            # closest centerline point
            cost += constraint_weight * rejection_dist(desired_x, desired_y, px, py)
            cost += control_weight * np.linalg.norm(np.vstack([a, w]), ord=2) ** 2
            cost += state_weight * np.linalg.norm(np.vstack([desired_x - px, desired_y - py, desired_v - v]), ord=2)
        px, py, v, h, a, w = states[-6:]
        cost += dstate_weight * np.linalg.norm(np.vstack([desired_x - px, desired_y - py, desired_v - v]), ord=2)
        return cost
    return objective


def formulate_guess(sim_time, guess_range):
    #return np.random.randint(guess_range[0], guess_range[1], size=sim_time*2)
    return np.ones(sim_time*2)


def solve_states(initial_states, desired_states, acceleration_constraint, turning_constraint, time_step=1, sim_time=10, guess_range=(0, 3)):

    init_x, init_y, init_velocity, init_heading = initial_states
    desired_x, desired_y, desired_velocity = desired_states

    state0 = [init_x, init_y, init_velocity, init_heading]
    init_guess = formulate_guess(sim_time, guess_range)
    bounds = [(-acceleration_constraint, acceleration_constraint),
              (-turning_constraint, turning_constraint)] * sim_time

    obj = formulate_objective(state0, [desired_x, desired_y, desired_velocity], time_step=time_step,
                              state_weight=1, constraint_weight=1, control_weight=0.5,
                              dstate_weight=10)

    result = opt.minimize(obj, init_guess, method='SLSQP', bounds=bounds,
                          options={'eps': 0.01, 'maxiter': 1000})
    states = compute_states(initial_states, result.x, time_step=time_step)
    return get_controls(states), result.success, result.message
