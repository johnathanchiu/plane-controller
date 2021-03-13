import sys
sys.path.append('..')

from controller import apply_lookup_controls, takeoff, PID
from definitions import XPlaneDefs
from geometry import kn_to_ms, rotate, true_heading, fix_heading, rejection_dist

from xpc import XPlaneConnect

import numpy as np
import random
import pickle
import math
import time


def confine_bins(value, bins):
    for lower_bin in range(bins[0], bins[1], bins[2]):
        if lower_bin <= value < lower_bin + bins[2]:
            return lower_bin
    return np.clip(value, bins[0], bins[1] - bins[2])
    
    
def set_winds():
    wind_speed = np.random.randint(ws_bins[0], ws_bins[1])
    wind_heading = np.random.randint(wh_bins[0], wh_bins[1])

    print("Using (wind speed, wind heading):", wind_speed, wind_heading)

    xp_wind_direction = -1 * wind_degrees + runway_heading # since positive rotation to right
    xp_wind_direction += 180 # wind is counter clockwise of true north
    friction = 0
    
    xp_client.sendDREFs(XPlaneDefs.condition_drefs, [friction, xp_wind_direction, wind_speed])


def setup():
    
    # place plane at origin
    _, start_y, _ = xp_client.getDREFs(XPlaneDefs.position_dref)
    xp_client.sendDREFs(XPlaneDefs.position_dref, [origin_x, start_y[0], origin_z])
    time.sleep(0.1)
    
    # set wind environment
    set_winds()
    time.sleep(0.1)
    
    # fix all systems
    xp_client.sendCOMM("sim/operation/fix_all_systems")
    # release park brake
    xp_client.sendDREF("sim/flightmodel/controls/parkbrake", 0)
    time.sleep(0.1)
    
    
def apply_takeoff_controls():
    
    throttle_controller = PID(2.0, 0.0, 1.0, 10.0, sample_time)
    rudder_controller = PID(0.3, 0.4, 1.5, 10.0, sample_time)
    
    read_drefs = XPlaneDefs.control_dref + XPlaneDefs.position_dref

    maximum_cle = 0
    controls = None
    
    start = time.time()
    for t in range(int(simulation_steps // receding_horizon)):
        
        gs, psi, throttle, x, _, z = xp_client.getDREFs(read_drefs)
        gs, psi, throttle, x, z = gs[0], psi[0], throttle[0], x[0], z[0]
        d_x, d_z = rotate(x - origin_x, z - origin_z, -(runway_heading + XPlaneDefs.zero_heading - 360))
        
        print("POSITION ON RUNWAY:", d_x, d_z)
        
        if TAKEOFF and cost < takeoff_cost_threshold:
            takeoff(xp_client)
            return maximum_cle

        cle = rejection_dist(desired_x, desired_z, x - origin_x, z - origin_z)
        maximum_cle = max(cld, maximum_cle)
        
        x, z = np.round(rotate(x - origin_x, z - origin_x, 360 - (runway_heading + XPlaneDefs.zero_heading)))
        x, z = int(x), int(z)

        psi = confine_bins(psi - runway_heading, heading_bins)
        gs = confine_bins(gs, velocity_bins)
        wind_speed = confine_bins(wind_speed, ws_bins)
        wind_degrees = confine_bins(wind_degrees, wh_bins)

        # print("(gs, heading, ws, wh)", (gs, psi, wind_speed, wind_degrees))
        # print("point: ", (x,z))
        grid_no = grid[(x, z)]
        controls, cost = table[grid_no][(gs, psi, wind_speed, wind_degrees)]

        # change wind conditions every second
        if time.time() - start > 1:
            set_winds()
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
    runway_heading = runway['runway_heading']
    
    simulation_steps = config['simulation_steps']
    sample_time = config['sample_time']
    receding_horizon = config['receding_horizon']
    
    TAKEOFF = config['takeoff']
    
    controls_table = pickle.load(open(args.table, 'rb'))
    desired_veloctiy, velocity_bins, heading_bins, ws_bins, wh_bins, grids, table = controls_table
    
    xp_client = XPlaneConnect()
    
    setup()
    cle = apply_takeoff_controls()
    
    print("FINISHED!")
    print("Maximum Center Line Error (CLE):", np.sqrt(cle))
    print("HEADING:", psi - runway_heading)

    
    
