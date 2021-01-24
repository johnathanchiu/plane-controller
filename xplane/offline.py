from controller import apply_lookup_controls, takeoff, PID
from definitions import XPlaneDefs
from geometry import kn_to_ms, rotate, true_heading, fix_heading, rejection_dist

from xpc import XPlaneConnect

import numpy as np
import random
import pickle
import math
import time
import sys


def confine_bins(value, bins):
    for lower_bin in range(bins[0], bins[1], bins[2]):
        if lower_bin <= value < lower_bin + bins[2]:
            return lower_bin
    return np.clip(value, bins[0], bins[1] - bins[2])

# load lookup tables
table = pickle.load(open('tables/table8.pkl', 'rb'))
grid = pickle.load(open('tables/grid8.pkl', 'rb'))

ws_bound = 1
wh_bound = 1

velocity_bins = (0, 60, 3)
heading_bins = (-30, 30, 4)

ws_bins = (0, 10, 1)
wh_bins = (0, 30, 1)

TAKEOFF = True

desired_velocity = 50

TAKEOFF_DIST = 250
 
### runway config ###

runway_origin_x = -25163.9477
runway_origin_z = 33693.3450

runway_end_x = -22742.57617
runway_end_z = 31956.02344

origin_x = runway_origin_x
origin_z = runway_origin_z

desired_x = runway_end_x - origin_x
desired_z = runway_end_z - origin_z

runway_heading = 54.331

# PID controller recompute time step
sample_time = 0.1
# number of seconds to run the PID controller + solver
simulation_steps = 50
# recompute using solver after t seconds
receding_horizon = 1 / 4

### environment config ###

# create XPC object
xp_client = XPlaneConnect()

wind_speed = np.random.randint(ws_bins[0], ws_bins[1])
wind_degrees = np.random.randint(wh_bins[0], wh_bins[1])

print("Using (wind speed, wind heading):", wind_speed, wind_degrees)

xp_wind_direction = -1 * wind_degrees + runway_heading # since positive rotation to right
xp_wind_direction += 180 # wind is counter clockwise of true north
friction = 0

xp_client.sendDREFs(XPlaneDefs.condition_drefs, [friction, xp_wind_direction, wind_speed])

start_x, start_y, start_z = xp_client.getDREFs(XPlaneDefs.position_dref)
start_y = start_y[0]
xp_client.sendDREFs(XPlaneDefs.position_dref, [origin_x, start_y, origin_z])

### initialize starting states ###

origin_x = runway_origin_x
origin_z = runway_origin_z

init_heading = xp_client.getDREF(XPlaneDefs.heading_dref)[0]
# add true north heading
init_heading += XPlaneDefs.zero_heading

time.sleep(1)

xp_client.sendCOMM("sim/operation/fix_all_systems")
# release park brake
xp_client.sendDREF("sim/flightmodel/controls/parkbrake", 0)

time.sleep(1)

controls = None
throttle_controller = PID(2.0, 0.0, 1.0, 10.0, sample_time)
rudder_controller = PID(0.3, 0.4, 1.5, 10.0, sample_time)

start = time.time()

cle = 0

for t in range(int(simulation_steps // receding_horizon)):
    read_drefs = XPlaneDefs.control_dref + XPlaneDefs.position_dref
    gs, psi, throttle, x, _, z = xp_client.getDREFs(read_drefs)
    gs, psi, throttle, x, z = gs[0], psi[0], throttle[0], x[0], z[0]
    d_x, d_z = rotate(x - origin_x, z - origin_z, -(runway_heading + XPlaneDefs.zero_heading - 360))
    print("POSITION ON RUNWAY:", d_x, d_z)
    if TAKEOFF and gs > desired_velocity and abs(psi - runway_heading) < 10 and \
        d_x - TAKEOFF_DIST > 0:
        takeoff(xp_client)
        break

    cld = rejection_dist(desired_x, desired_z, x - origin_x, z - origin_z)
    cle = max(cld, cle)
    
    x -= origin_x
    z -= origin_z
    x, z = np.round(rotate(x, z, 360 - (runway_heading + XPlaneDefs.zero_heading)))
    x, z = int(x), int(z)

    psi = confine_bins(psi - runway_heading, heading_bins)
    gs = confine_bins(gs, velocity_bins)
    wind_speed = confine_bins(wind_speed, ws_bins)
    wind_degrees = confine_bins(wind_degrees, wh_bins) 

    # print("(gs, heading, ws, wh)", (gs, psi, wind_speed, wind_degrees))
    # print("point: ", (x,z))
    grid_no = grid[(x, z)]
    controls, cost = table[grid_no][(gs, psi, wind_speed, wind_degrees)]

    if time.time() - start > 1:
        wind_speed = np.random.randint(ws_bins[0], ws_bins[1])
        wind_degrees = np.random.randint(wh_bins[0], wh_bins[1])

        print("Using (wind speed, wind heading):", wind_speed, wind_degrees)

        xp_wind_direction = -1 * wind_degrees + runway_heading # since positive rotation to right
        xp_wind_direction += 180 # wind is counter clockwise of true north
        friction = 0

        xp_client.sendDREFs(XPlaneDefs.condition_drefs, [friction, xp_wind_direction, wind_speed])
        start = time.time()

    apply_lookup_controls(xp_client, throttle_controller, rudder_controller, controls, sample_time, receding_horizon)


print("FINISHED!")
print("Maximum Center Line Error (CLE):", np.sqrt(cle))
print("HEADING:", psi - runway_heading)