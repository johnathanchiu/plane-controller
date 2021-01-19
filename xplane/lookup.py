from solver import solve_states
from definitions import XPlaneDefs
from geometry import rotate, solver_heading, true_heading

import numpy as np
import random
import time
import json


### runway setup ###

runway_end_x = 2421
runway_end_y = -1737

runway_start_x = 0
runway_start_y = 0

runway_heading = 54.331

runway_width = 100
runway_extend_length = 50

### solver setup ###

desired_x = runway_end_x
desired_z = runway_end_z

desired_velocity = 50 # m/s

acceleration_constraint = 10 # m/s^2
turning_constraint = 40 # degrees/s

plane_cs = 27.41 # square meters
plane_mass = 6175 * 0.45359237 # lbs -> kg
plane_half_length = 10.82 / 2 # meters

plane_specs = [plane_cs, plane_mass, plane_half_length]
desired_states = [desired_x, desired_z, desired_velocity]

### solver config ###

# solver time step
time_step = 1 # seconds
# solver number of states to solve
num_steps = 5 # seconds


### sampling config ###

ws_bound = 5
wh_bound = 30

velocity_bins=(-10, 60)
wind_speed=(0, 10)
wind_headings = (-40, 40)

box_dim=(5, 5)

num_samples = 1

### setup runway coordinates ###

ex, ey = rotate(runway_end_x, runway_end_y, -(runway_heading + XPlaneDefs.zero_heading - 360))
sx, sy = rotate(runway_start_x, runway_start_y, -(runway_heading + XPlaneDefs.zero_heading - 360))
ex, ey, sx, sy = int(np.floor(ex)), int(np.floor(ey)), int(np.floor(sx)), int(np.floor(sy))

sx -= runway_extend_length

runway_left, runway_bot = sx, sy - runway_width
runway_right, runway_top = ex, ey + runway_width


### functions ###

def sample_environments(real_ws, real_wh):
    env = [(real_ws, solver_heading(real_wh)]
    for _ in range(num_samples):
        env.append((np.random.randint(real_ws - ws_bound, real_ws + ws_bound + 1),
                    solver_heading(np.random.randint(real_wh - wh_bound, real_wh + wh_bound + 1))))
    return env


def generate_control_lookup(box_dim):
    stride_x, stride_y = box_dim

    assert runway_right > runway_left, "right side of runway must be numerically greater than left side"
    assert runway_top > runway_bot, "top side of runway must be numerically greater than bottom side"
    
    leftover_x = stride_x - (runway_right - runway_left) % stride_x
    leftover_y = stride_y - (runway_top - runway_bot) % stride_y

    grid_search = {}
    lookup_table = {}
    grid_no = 0
    
    for i in range(runway_left, runway_right + leftover_x, stride_x):
        for j in range(runway_bot, runway_top + leftover_y, stride_y):
            center_x, center_y = i + stride_x / 2, j + stride_y / 2
            # ITERATE THROUGH ALL COMBINATIONS OF INIT VELOCITY, INIT HEADING, WINDSPEEDS, WINDHEADINGS
            local_controls = {}
            for h in range(0, 360):
                for v in range(velocity_bins[0], velocity_bins[1]):
                    for ws in range(wind_speed[0], wind_speed[1]):
                        for wh in range(wind_headings[0], wind_headings[1]):
                            center_x, center_y = rotate(center_x, center_y, (runway_heading + XPlaneDefs.zero_heading - 360))
                            state0 = [center_x, center_y, v, solver_heading(h)]
                            # offset wind heading by runway heading
                            winds = sample_environments(ws, wh + runway_heading)
                            controls, _, _ = solve_states(state0, desired_states, winds, plane_specs, acceleration_constraint,
                                  turning_constraint, time_step=time_step, sim_time=num_steps)
                            controls = [[c[0], true_heading(c[1])] for c in controls]
                            local_controls[(v, h, ws, wh)] = controls
            lookup_table[grid_no] = local_controls
            # GENERATE GRID POINTS LOOK UP
            for k in range(stride_x):
                for l in range(stride_y):
                    grid_search[(i + k, j + l)] = grid_no
            grid_no += 1
            
    return grid_search, lookup_table

grids, table = generate_control_lookup(box_dim)

# save lookup table
json.dump(table, open("table.json", 'w'))
json.dump(grids, open("grid.json", 'w'))
