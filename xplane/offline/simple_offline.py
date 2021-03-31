import sys
sys.path.append('..')

from geometry import kn_to_ms, rotate, rejection_dist, signed_rejection_dist
from controller import apply_lookup_controls, takeoff, PID
from definitions import XPlaneDefs

from xpc import XPlaneConnect

import numpy as np
import argparse
import random
import pickle
import yaml
import math
import time


def confine_bins(value, bins):
    return bins[np.argmin(np.abs(bins - value))]
    
    
def set_winds():
    wind_speed = np.random.randint(ws_bins[0], ws_bins[1])
    wind_heading = np.random.randint(wh_bins[0], wh_bins[1])

    print("Using (wind speed, wind heading):", wind_speed, wind_heading)

    xp_wind_direction = -1 * wind_heading + runway_heading # since positive rotation to right
    xp_wind_direction += 180 # wind is counter clockwise of true north
    friction = 0
    
    xp_client.sendDREFs(XPlaneDefs.condition_drefs, [friction, xp_wind_direction, wind_speed])
    return wind_speed, wind_heading

def setup():
    
    # place plane at origin
    _, start_y, _ = xp_client.getDREFs(XPlaneDefs.position_dref)
    xp_client.sendDREFs(XPlaneDefs.position_dref, [origin_x, start_y[0], origin_z])
    time.sleep(0.1)
    
    # fix all systems
    xp_client.sendCOMM("sim/operation/fix_all_systems")
    # release park brake
    xp_client.sendDREF("sim/flightmodel/controls/parkbrake", 0)
    time.sleep(0.1)
    
    
def apply_takeoff_controls():
    
    # set wind environment
    wind_speed, wind_heading = set_winds()
    
    throttle_controller = PID(2.0, 0.0, 1.0, 10.0, sample_time)
    rudder_controller = PID(0.3, 0.4, 1.5, 10.0, sample_time)
    
    read_drefs = XPlaneDefs.control_dref + XPlaneDefs.position_dref

    maximum_cle = 0
    controls = None
    
    start = time.time()
    for t in range(int(simulation_steps // receding_horizon)):
        
        gs, psi, throttle, x, _, z = xp_client.getDREFs(read_drefs)
        gs, psi, throttle, x, z = gs[0], psi[0], throttle[0], x[0], z[0]

        if TAKEOFF:
            break

#        cle = rejection_dist(desired_x, desired_z, x - origin_x, z - origin_z)
#        maximum_cle = max(cld, maximum_cle)
        
        dist = signed_rejection_dist(center_line, x - origin_x, z - origin_z)
        dist = confine_bins(dist, c_bins)
        psi = confine_bins(psi - runway_heading, h_bins)
        gs = confine_bins(gs, v_bins)
        wind_speed = confine_bins(wind_speed, wsp_bins)
        wind_heading = confine_bins(wind_heading, whe_bins)

        controls, cost = table[(dist, psi, gs, wind_speed, wind_heading)]

        # change wind conditions every second
        if time.time() - start > 1:
            wind_speed, wind_heading = set_winds()
            start = time.time()
        
        # let PID controller take over
        apply_lookup_controls(xp_client, throttle_controller, rudder_controller, controls, sample_time, receding_horizon)
    
    return maximum_cle
    

def load_yaml(filename):
    with open(filename, 'r') as stream:
        options = yaml.safe_load(stream)
    return options


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--runway_config', help='runway configuration file', default='../runway.yaml')
    parser.add_argument('-c', '--controller_config', help='PID configuration file', default='offline.yaml')
    parser.add_argument('-t', '--table', help='lookup table name')
    args = parser.parse_args()
    
    runway = load_yaml(args.runway_config)
    config = load_yaml(args.controller_config)

    origin_x = runway['origin_X']
    origin_z = runway['origin_Z']
    runway_end_x = runway['terminate_X'] - origin_x     
    runway_end_z = runway['terminate_Z'] - origin_z
    runway_heading = runway['runway_heading']
    center_line = np.array([runway_end_x, runway_end_z])
    
    simulation_steps = config['simulation_steps']
    sample_time = config['sample_time']
    receding_horizon = config['receding_horizon']
    
    TAKEOFF = config['takeoff']
    
    controls_table = pickle.load(open(args.table, 'rb'))
    _, ct_bins, velocity_bins, heading_bins, ws_bins, wh_bins, table = controls_table

    c_bins = np.array([i for i in range(ct_bins[0], ct_bins[1] + 1, ct_bins[2])])
    v_bins = np.array([i for i in range(velocity_bins[0], velocity_bins[1] + 1, velocity_bins[2])])
    h_bins = np.array([i for i in range(heading_bins[0], heading_bins[1] + 1, heading_bins[2])])
    wsp_bins = np.array([i for i in range(ws_bins[0], ws_bins[1] + 1, ws_bins[2])])
    whe_bins = np.array([i for i in range(wh_bins[0], wh_bins[1] + 1, wh_bins[2])])
    
    xp_client = XPlaneConnect()
    
    setup()
    cle = apply_takeoff_controls()
    
    print("FINISHED!")
    print("Maximum Center Line Error (CLE):", np.sqrt(cle))

    
    
