from geometry import compute_heading_error, quaternion_for
from solver import solve_states
from controller import control

from xpc import XPlaneConnect

import math
import time

import sys

### runway setup ###

#runway_origin_x = -25163.9477
#runway_origin_z = 33693.3450

runway_end_x = -22742.57617
runway_end_z = 31956.02344

runway_heading = 54.331

### states setup ###

# true north is 270 degrees in unit circle coordinates
xplane_zero_heading = 270

desired_velocity = 30 # m/s

acceleration_constraint = 7 # m/s^2
turning_constraint = 10 # degrees

time_step = 1
sim_time = 10

guess_range = (0, 3)

# create XPC object
xp_client = XPlaneConnect()

# datarefs #
groundspeed_dref = "sim/flightmodel/position/groundspeed"           # m/s
heading_dref = "sim/flightmodel/position/psi"                       # in degrees
throttle_dref = "sim/flightmodel/engine/ENGN_thro"
position_dref = ["sim/flightmodel/position/local_x",
                 "sim/flightmodel/position/local_y",
                 "sim/flightmodel/position/local_z"]
control_dref = [groundspeed_dref, heading_dref, throttle_dref]

### initialize starting states ###

init_x, _, init_z = xp_client.getDREFs(position_dref)
init_x, init_z = true_init_x[0], true_init_z[0]

init_heading = xp_client.getDREF(heading_dref)
# add true north heading
init_heading += xplane_zero_heading

# want to end here at t = sim_time
desired_x = runway_end_x - init_x
desired_z = runway_end_z - init_z

time.sleep(1)

# release park brake
xp_client.sendDREF("sim/flightmodel/controls/parkbrake", 0)

time.sleep(1)

state = 1
controls = None
for i in range(20):
    gs, psi, throttle, x, _, z = xp_client.getDREFs(control_dref + position_dref)
    gs, psi, throttle, x, z = gs[0], psi[0], throttle[0], x[0], z[0]
    
    if i % 5 == 0:
        init_states = [x - init_x, z - init_z, gs, psi + xplane_zero_heading]
        desired_states = [desired_x, desired_y, desired_velocity]
#        print("computing optimal trajectory/controls")
        controls, success, msg = solve_states(init_states, desired_states, acceleration_constraint, turning_constraint, time_step=time_step, sim_time=sim_time, guess_range=guess_range)
#        print(success, msg)
        state = 1

    # send controls to xplane
    velocity, heading = controls[state]
    # remove solver orientation
    heading -= xplane_zero_heading
    heading_err = compute_heading_error(heading, psi)
    control(xp_client, velocity, gs, throttle, heading_err)
    state += 1

    time.sleep(time_step)
