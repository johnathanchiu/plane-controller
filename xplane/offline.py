from controller import apply_controls, takeoff, PID
from definitions import XPlaneDefs
from geometry import kn_to_ms, rotate, true_heading,

from xpc import XPlaneConnect

import numpy as np
import random
import math
import time
import sys


# load lookup tables
table = json.load(open("table.json"))
grid = json.load(open("grid.json" ))


TAKEOFF = False

 
### runway config ###

runway_origin_x = -25163.9477
runway_origin_z = 33693.3450

runway_end_x = -22742.57617
runway_end_z = 31956.02344

runway_heading = 54.331

# PID controller recompute time step
sample_time = 0.1
# number of seconds to run the PID controller + solver
simulation_steps = 100
# recompute using solver after t seconds
receding_horizon = 1

### environment config ###

wind_speed = 10
wind_degrees = 20
xp_wind_direction = -1 * wind_degrees + runway_heading # since positive rotation to right
xp_wind_direction += 180 # wind is counter clockwise of true north

friction = 0

xp_client.sendDREFs(XPlaneDefs.condition_drefs, [friction, xp_wind_direction, wind_speed])

# create XPC object
xp_client = XPlaneConnect()

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
for t in range(int(simulation_steps // receding_horizon)):
    read_drefs = XPlaneDefs.control_dref + XPlaneDefs.position_dref
    gs, psi, throttle, x, _, z = xp_client.getDREFs(read_drefs)
    gs, psi, throttle, x, z = gs[0], psi[0], throttle[0], x[0], z[0]
    if TAKEOFF and gs > desired_velocity and abs(psi - runway_heading) < 10:
        takeoff(xp_client)
        break
    
    x, z = np.round(rotate(x, z, -(runway_heading + XPlaneDefs.zero_heading - 360)))
    x, z = int(x), int(z)
    psi = int(np.round(psi))
    gs = fix_heading(int(np.round(gs)))
    
    grid_no = grid[(x, z)]
    controls = table[grid_no][(gs, psi, wind_speed, wind_degrees)]

    apply_controls(xp_client, throttle_controller, rudder_controller, controls, sample_time, time_step, receding_horizon)

