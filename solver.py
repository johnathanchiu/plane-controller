import numpy as np
import scipy.optimize as opt

import random


get_controls = lambda x: [x[i+2:i+4] for i in range(0, len(x), 6)]


def shape_states(variables):
    states = variables[:4]
    controls = np.concatenate(([0, 0], variables[4:6]))
    controls[0] = states[2] * np.cos(np.radians(states[3]))
    controls[1] = states[2] * np.sin(np.radians(states[3]))
    return states, controls


def formulate_objective(desired_x, desired_y, desired_v, state_weight=0.5, control_weight=0.5,
                        constraint_weight=0.5):
    def objective(params):
        cost = 0
        cost += control_weight * np.linalg.norm(np.vstack([params[4], params[5]]), ord=2) ** 2
        for i in range(6, len(params) - 6, 6):
            px, py, v, h, a, w = params[i:i+6]
            cost += constraint_weight * np.linalg.norm([px], ord=2) ** 2
            cost += control_weight * np.linalg.norm(np.vstack([a, w]), ord=2) ** 2
            cost += np.linalg.norm([desired_v - v], ord=2) ** 2
        px, py, v, h, a, w = params[-6:]
        cost += state_weight * np.linalg.norm(np.vstack([desired_x - px, desired_y - py, desired_v - v]), ord=2)
        return cost
    return objective


def formulate_constraints(state0, sim_time, ts=1):
    def dynamic_constraints(params):
        errors = np.zeros((sim_time,))
        states_t, controls_t = shape_states(state0)
        errors[0] = np.linalg.norm(params[:4] - states_t, ord=2)
        
        for i in range(6, len(params), 6):
            states_t1, controls_t1 = shape_states(params[i:i+6])
            #error[t+1] = norm(x_(t+1) - f(x_t, u_t))
            errors[int(i // 6)] = np.linalg.norm(states_t1 - (states_t + ts * controls_t), ord=2)
            states_t, controls_t = states_t1, controls_t1

        return np.sum(errors)
    return dynamic_constraints


def formulate_guess(state0, sim_time, guess_range, ts=1, random_seed=0):
    
    guess_a = random.randint(guess_range[0], guess_range[1])
    guess_w = random.randint(guess_range[0], guess_range[1])
    state0_controlled = state0 + [guess_a, guess_w]
    init_state = state0_controlled[:]
    ix, iy, iv, ih, ia, iw = init_state
    
    for _ in range(sim_time - 1):
        
        new_x = ix + ts * iv * np.cos(np.radians(ih))
        new_y = iy + ts * iv * np.sin(np.radians(ih))
        new_v = iv + ts * ia
        new_h = ih + ts * iw
        guess_a = random.randint(guess_range[0], guess_range[1])
        guess_w = random.randint(guess_range[0], guess_range[1])
        init_state += [new_x, new_y, new_v, new_h, guess_a, guess_w]
        
        ix, iy, iv, ih, ia, iw = new_x, new_y, new_v, new_h, guess_a, guess_w
        
    return state0_controlled, np.array(init_state)
    

def solve_states(initial_states, desired_states, time_step, sim_time, guess_range=(0, 3)):

    init_x, init_y, init_velocity, init_heading = initial_states
    desired_x, desired_y, desired_velocity = desired_states

    state0 = [init_x, init_y, init_velocity, init_heading]
    state0, init_guess = formulate_guess(state0, sim_time, guess_range, ts=time_step)

    con = formulate_constraints(state0, sim_time, ts=time_step)
    obj = formulate_objective(desired_x, desired_y, desired_velocity, state_weight=10,
                              constraint_weight=1, control_weight=0.5)

    result = opt.minimize(obj, init_guess, method='SLSQP', constraints={"fun": con, "type": "eq"},
                          options={'eps': 0.01}) #'maxiter': 1000})
                          
    return get_controls(result.x), result.success, result.message
