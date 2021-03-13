import sys
sys.path.append('..')

from solver import solve_states
from definitions import XPlaneDefs
from geometry import rotate, solver_heading, true_heading, kn_to_ms

import numpy as np
import random
import time

import pickle
import yaml
import argparse


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

    grid_no = 0
    grid_search, lookup_table = {}, {}
    
    start = time.time()
    for i in range(runway_left, runway_right + leftover_x, stride_x):
        for j in range(runway_bot, runway_top + leftover_z, stride_z):
            init_x, init_z = i + stride_x / 2, j + stride_z / 2
            center_x, center_z = rotate(init_x, init_z, (runway_heading + XPlaneDefs.zero_heading - 360))
            # ITERATE THROUGH ALL COMBINATIONS OF INIT VELOCITY, INIT HEADING, WINDSPEEDS, WINDHEADINGS
            local_controls = {}
            generate_update = 0
            for h in range(heading_bins[0], heading_bins[1], heading_bins[2]):
                for v in range(velocity_bins[0], velocity_bins[1], velocity_bins[2]):
                    for ws in range(ws_bins[0], ws_bins[1], ws_bins[2]):
                        for wh in range(wh_bins[0], wh_bins[1], wh_bins[2]):
                            center_h, center_v = h + heading_bins[2] / 2, v + velocity_bins[2] / 2
                            state0 = [center_x, center_z, center_v, solver_heading(center_h + runway_heading)]
                            
                            # offset wind heading by runway heading
                            winds = sample_environments(kn_to_ms(ws), wh + runway_heading)
                            controls, cost = solve_states(state0, desired_states, winds, plane_specs, acceleration_constraint,
                                  turning_constraint, time_step=time_step, sim_time=num_steps)
                                  
                            controls = [[c[0], true_heading(c[1])] for c in controls]
                            local_controls[(v, h, ws, wh)] = (controls, cost)
                            if generate_update % 1000 == 0:
                                print("Update: {} of {} controls in grid {}".format(generate_update, no_of_controls, grid_no))
                            generate_update += 1
            lookup_table[grid_no] = local_controls
            # GENERATE GRID POINTS LOOK UP
            for k in range(stride_x):
                for l in range(stride_z):
                    grid_search[(i + k, j + l)] = grid_no
            print("Finished Grid:", grid_no)
            print("--- Grid completed in %s seconds ---" % (time.time() - start))
            grid_no += 1            
    return grid_search, lookup_table
    
    
def load_yaml(filename):
    with open(filename, 'r') as stream:
        options = yaml.safe_load(stream)
    return options


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', help='configuration file', default='config.yaml')
    parser.add_argument('-r', '--runway_config', help='runway configuration file', default='runway.yaml')
    parser.add_argument('-t', '--table', help='lookup table name')
    args = parser.parse_args()

    config = load_yaml(args.config)
    runway = load_yaml(args.runway_config)

    runway_start_x, runway_start_z = 0, 0
    runway_end_x = runway['terminate_X'] - runway['origin_X']
    runway_end_z = runway['terminate_Z'] - runway['origin_Z']
    runway_heading = runway['runway_heading']

    runway_half_width = config['runway_half_width']
    
    plane_specs = [config['plane_cs'], config['plane_mass'], config['plane_half_length']]
    desired_states = [runway_end_x, runway_end_z, config['desired_velocity']]
    
    runway_front_extend = config['runway_front_extend']
    runway_end_decrease = config['runway_end_decrease']
    
    ### lookup specification ###
    box_dimension = config['box_dimension']
    velocity_bins = config['velocity_bins']
    heading_bins = config['heading_bins']
    ws_bins = config['wind_speed_bins']
    wh_bins = config['wind_heading_bins']
    
    ### solver environment sampling ###
    num_samples = config['number_samples']
    
    ### setup runway coordinates ###
    ex, ez = rotate(runway_end_x, runway_end_z, -(runway_heading + XPlaneDefs.zero_heading - 360))
    sx, sz = rotate(runway_start_x, runway_start_z, -(runway_heading + XPlaneDefs.zero_heading - 360))
    ex, ez, sx, sz = int(np.floor(ex)), int(np.floor(ez)), int(np.floor(sx)), int(np.floor(sz))

    sx -= runway_extend_length
    ex -= runway_end_decrease
    sz -= runway_half_width
    ez += runway_half_width

    runway_left, runway_bot = sx, sz
    runway_right, runway_top = ex, ez
    
    ### generate lookup table ###
    
    no_of_controls = ((heading_bins[1] - heading_bins[0]) / heading_bins[2]) * \
                     ((velocity_bins[1] - velocity_bins[0]) / velocity_bins[2]) * \
                     ((ws_bins[1] - ws_bins[0]) / ws_bins[2]) * \
                     ((wh_bins[1] - wh_bins[0]) / wh_bins[2])
    
    print("Generating Controls Table...")
    start = time.time()
    grids, table = generate_control_lookup(box_dim)
    print("--- %s seconds ---" % (time.time() - start))
    
    controls_table = [desired_velocity, velocity_bins, heading_bins, ws_bins, wh_bins, grids, table]
    
    # save lookup table
    pickle.dump(controls_table, open(args.table, 'wb'))
    
    
    
    
    

