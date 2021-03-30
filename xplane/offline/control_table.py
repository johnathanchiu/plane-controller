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
        env.append((np.random.randint(real_ws - ws_bins[2], real_ws + ws_bins[2] + 1),
                    solver_heading(np.random.randint(real_wh - wh_bins[2], real_wh + wh_bins[2] + 1))))
    return env


def generate_control_lookup():
    
    lookup_table = {}
    
    for c in range(cross_track_bins[0], cross_track_bins[1], cross_track_bins[2]):
        for h in range(heading_bins[0], heading_bins[1], heading_bins[2]):
            for v in range(velocity_bins[0], velocity_bins[1], velocity_bins[2]):
                for ws in range(ws_bins[0], ws_bins[1], ws_bins[2]):
                    for wh in range(wh_bins[0], wh_bins[1], wh_bins[2]):
                    
                            desired_states = [runway_heading, desired_velocity]
                            init_states = [c, v, h]
                            winds = sample_environments(kn_to_ms(ws), wh + runway_heading)
                            controls, cost = solve_states(init_states, desired_states, center_line, winds, plane_specs, acceleration_constraint, turning_constraint, time_step=time_step, sim_time=num_steps)
                            controls = [[c[0], c[1]] for c in controls]
                            lookup_table[(c, h, v, ws, wh)] = (controls, cost)

    return lookup_table
    
    
def load_yaml(filename):
    with open(filename, 'r') as stream:
        options = yaml.safe_load(stream)
    return options


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', help='configuration file', default='simple-lookup.yaml')
    parser.add_argument('-r', '--runway_config', help='runway configuration file', default='../runway.yaml')
    parser.add_argument('-t', '--table', help='lookup table name')
    args = parser.parse_args()

    config = load_yaml(args.config)
    runway = load_yaml(args.runway_config)

    ### runway points ###
    runway_end_x = runway['terminate_X'] - runway['origin_X']
    runway_end_z = runway['terminate_Z'] - runway['origin_Z']
    runway_heading = runway['runway_heading']
    center_line = np.array([runway_end_x, runway_end_z])
    
    ### physical plane constraints ###
    plane_specs = [config['plane_cs'], config['plane_mass'], config['plane_half_length']]
    acceleration_constraint = config['acceleration_constraint']
    turning_constraint = config['turning_constraint']

    ### desired position ###
    desired_states = [runway_heading, config['desired_velocity']]
    
    ### lookup specification ###
    velocity_bins = config['velocity_bins']
    heading_bins = config['heading_bins']
    ws_bins = config['wind_speed_bins']
    wh_bins = config['wind_heading_bins']
    cross_track_bins = config['cross_track_bins']
    
    ### solver environment sampling ###
    num_samples = config['number_samples']

    ### solver steps ###
    time_step = config['time_step']
    num_steps = config['num_steps']
    
    ### generate lookup table ###
    
    no_of_controls = ((cross_track_bins[1] - cross_track_bins[0]) / cross_track_bins[2]) * \
                     ((heading_bins[1] - heading_bins[0]) / heading_bins[2]) * \
                     ((velocity_bins[1] - velocity_bins[0]) / velocity_bins[2]) * \
                     ((ws_bins[1] - ws_bins[0]) / ws_bins[2]) * \
                     ((wh_bins[1] - wh_bins[0]) / wh_bins[2])
    
    print("Generating Controls Table...")
    start = time.time()
    table = generate_control_lookup()
    print("--- %s seconds ---" % (time.time() - start))
    
    controls_table = [desired_velocity, velocity_bins, heading_bins, ws_bins, wh_bins, table]
    
    # save lookup table
    pickle.dump(controls_table, open(args.table, 'wb'))
    
    
    
    
    

