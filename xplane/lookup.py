from solver import solve_states
from definitions import XPlaneDefs
from geometry import rotate, solver_heading, true_heading

import numpy as np
import random
import time

import pickle

### runway setup ###

runway_end_x = 2421
runway_end_z = -1737

runway_start_x = 0
runway_start_z = 0

runway_heading = 54.331

runway_width = 20 # * 2
runway_extend_length = 5
runway_end_decrease = 2850

### solver setup ###

desired_x = runway_end_x
desired_z = runway_end_z

desired_velocity = 4.5 # m/s

acceleration_constraint = 2 # m/s^2
turning_constraint = 10 # degrees/s

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

ws_bound = 1
wh_bound = 1

velocity_bins = (0, 6, 1)
heading_bins = (-30, 30, 5)

ws_bins = (0, 6, ws_bound)
wh_bins = (0, 6, wh_bound)

box_dim=(40, 8)

num_samples = 5
    
### setup runway coordinates ###

ex, ez = rotate(runway_end_x, runway_end_z, -(runway_heading + XPlaneDefs.zero_heading - 360))
sx, sz = rotate(runway_start_x, runway_start_z, -(runway_heading + XPlaneDefs.zero_heading - 360))
ex, ez, sx, sz = int(np.floor(ex)), int(np.floor(ez)), int(np.floor(sx)), int(np.floor(sz))

sx -= runway_extend_length
ex -= runway_end_decrease

runway_left, runway_bot = sx, sz - runway_width
runway_right, runway_top = ex, ez + runway_width

### functions ###

def sample_environments(real_ws, real_wh):
    env = [(real_ws, solver_heading(real_wh))]
    for _ in range(num_samples):
        env.append((np.random.randint(real_ws - ws_bound, real_ws + ws_bound + 1),
                    solver_heading(np.random.randint(real_wh - wh_bound, real_wh + wh_bound + 1))))
    return env


def generate_control_lookup(box_dim):
    stride_x, stride_z = box_dim

    assert runway_right > runway_left, "right side of runway must be numerically greater than left side"
    assert runway_top > runway_bot, "top side of runway must be numerically greater than bottom side"
    
    leftover_x = stride_x - (runway_right - runway_left) % stride_x
    leftover_z = stride_z - (runway_top - runway_bot) % stride_z
    leftover_x = 0 if leftover_x == stride_x else leftover_x
    leftover_z = 0 if leftover_z == stride_z else leftover_z

    no_of_grids = runway_right + leftover_x - runway_left
    no_of_grids *= runway_top + leftover_z - runway_bot
    no_of_grids /= stride_x * stride_z
    print("No. Grids to Generate:", no_of_grids)

    grid_search = {}
    lookup_table = {}
    grid_no = 0
    
    for i in range(runway_left, runway_right + leftover_x, stride_x):
        for j in range(runway_bot, runway_top + leftover_z, stride_z):
            center_x, center_z = i + stride_x / 2, j + stride_z / 2
            # ITERATE THROUGH ALL COMBINATIONS OF INIT VELOCITY, INIT HEADING, WINDSPEEDS, WINDHEADINGS
            local_controls = {}
            count = 0
            for h in range(heading_bins[0], heading_bins[1], heading_bins[2]):
                for v in range(velocity_bins[0], velocity_bins[1], velocity_bins[2]):
                    for ws in range(ws_bins[0], ws_bins[1], ws_bins[2]):
                        for wh in range(wh_bins[0], wh_bins[1], wh_bins[2]):
                            center_h, center_v = h + heading_bins[2] / 2, v + velocity_bins[2] / 2
                            center_x, center_z = rotate(center_x, center_z, (runway_heading + XPlaneDefs.zero_heading - 360))
                            state0 = [center_x, center_z, center_v, solver_heading(center_h + runway_heading)]
                            # offset wind heading by runway heading
                            winds = sample_environments(ws, wh + runway_heading)
                            controls, _, _ = solve_states(state0, desired_states, winds, plane_specs, acceleration_constraint,
                                  turning_constraint, time_step=time_step, sim_time=num_steps)
                            controls = [[c[0], true_heading(c[1])] for c in controls]
                            local_controls[(v, h, ws, wh)] = controls
                            print("finished!", count)
                            count += 1
            lookup_table[grid_no] = local_controls
            # GENERATE GRID POINTS LOOK UP
            for k in range(stride_x):
                for l in range(stride_z):
                    grid_search[(i + k, j + l)] = grid_no
            print("Finished Grid:", grid_no)
            grid_no += 1            
    return grid_search, lookup_table

print("Generating Controls Table...")
start = time.time()

grids, table = generate_control_lookup(box_dim)

print("--- %s seconds ---" % (time.time() - start))

# save lookup table
pickle.dump(table, open('table.pkl', 'wb'))
pickle.dump(grids, open('grid.pkl', 'wb'))

