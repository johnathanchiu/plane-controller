from solver import solve_states
from controller import apply_controls, PID
from definitions import XPlaneDefs

from xpc import XPlaneConnect

import numpy as np
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

desired_x = runway_end_x
desired_z = runway_end_z

desired_velocity = 27 # m/s

acceleration_constraint = 7 # m/s^2
turning_constraint = 10 # degrees

time_step = 1
num_steps = 10

simulation_steps = 1000

receding_horizon = 5

# create XPC object
xp_client = XPlaneConnect()

### initialize starting states ###

init_x, _, init_z = np.squeeze(np.array(xp_client.getDREFs(XPlaneDefs.position_dref)))

init_heading = xp_client.getDREF(XPlaneDefs.heading_dref)[0]
# add true north heading
init_heading += XPlaneDefs.zero_heading

# want to end here at t = sim_time
desired_x -= init_x
desired_z -= init_z

time.sleep(1)

# release park brake
xp_client.sendDREF("sim/flightmodel/controls/parkbrake", 0)

time.sleep(1)

state, controls = 1, None
throttle_controller = PID(1.0, 0.0, 3.0, 20.0, time_step)
for t in range(simulation_steps):
    read_drefs = XPlaneDefs.control_dref + XPlaneDefs.position_dref
    gs, psi, x, _, z = np.squeeze(np.array(xp_client.getDREFs(read_drefs)))

    if t % receding_horizon == 0:
        new_init_states = [x - init_x, z - init_z, gs, psi + XPlaneDefs.zero_heading]
        desired_states = [desired_x, desired_z, desired_velocity]
        controls, _, _ = solve_states(new_init_states, desired_states, acceleration_constraint,
                                        turning_constraint, time_step=time_step, sim_time=num_steps)
        throttle_controller.clear()
        state = 1
    
    heading_control, velocity_control = controls[state]
    heading_control -= XPlaneDefs.zero_heading
    # apply the controls from solver
    apply_controls(xp_client, throttle_controller, [heading_control, velocity_control], [gs, psi])
    state += 1

    time.sleep(time_step)
