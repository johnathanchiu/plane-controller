import sys
sys.path.append('..')

from simple_solver import solve_states
from controller import apply_controls, takeoff, PID
from definitions import XPlaneDefs
from geometry import kn_to_ms, signed_rejection_dist, rotate

from xpc import XPlaneConnect

import numpy as np
import random
import math
import time
import sys

### runway setup ###

runway_origin_x = -25163.9477
runway_origin_z = 33693.3450

runway_end_x = -22742.57617
runway_end_z = 31956.02344

runway_heading = 54.331

### states setup ###

desired_x = runway_end_x
desired_z = runway_end_z

desired_velocity = 50 # m/s

acceleration_constraint = 2 # m/s^2
turning_constraint = 40 # degrees

plane_cs = 27.41 # square meters
plane_mass = 6175 * 0.45359237 # lbs -> kg
plane_half_length = 10.82 / 2 # meters

plane_specs = [plane_cs, plane_mass, plane_half_length]

# solver time step
time_step = 1 # seconds
# solver number of states to solve
num_steps = 4

# PID controller recompute time step
sample_time = 0.1
# number of seconds to run the PID controller + solver
simulation_steps = 50

# recompute using solver after t seconds
receding_horizon = 1

# number of environments to sample
num_environment_samples = 100

TAKEOFF = True
SAMPLE = False

TAKEOFF_DIST = 500

# create XPC object
xp_client = XPlaneConnect()

### initialize starting states ###

origin_x = runway_origin_x
origin_z = runway_origin_z

start_x, start_y, start_z = xp_client.getDREFs(XPlaneDefs.position_dref)
start_y = start_y[0]
xp_client.sendDREFs(XPlaneDefs.position_dref, [origin_x, start_y, origin_z])

# want to end here at t = sim_time
desired_x -= origin_x
desired_z -= origin_z

time.sleep(1)

xp_client.sendCOMM("sim/operation/fix_all_systems")
# release park brake
xp_client.sendDREF("sim/flightmodel/controls/parkbrake", 0)

time.sleep(1)

controls = None
throttle_controller = PID(2.0, 0.0, 1.0, 10.0, sample_time)
rudder_controller = PID(0.3, 0.4, 1.5, 10.0, sample_time)

read_drefs = XPlaneDefs.control_dref + XPlaneDefs.position_dref
winds = [[0, runway_heading]]
cl = np.array([desired_x, desired_z])

for t in range(50):
    gs, psi, throttle, x, _, z = xp_client.getDREFs(read_drefs)
    gs, psi, throttle, x, z = gs[0], psi[0], throttle[0], x[0], z[0]
    
    desired_states = [runway_heading, desired_velocity]
    dist = signed_rejection_dist(cl, x - origin_x, z - origin_z)
    init_states = [dist, gs, psi]

    controls, _ = solve_states(init_states, desired_states, cl, winds, plane_specs, acceleration_constraint,
                                  turning_constraint, time_step=time_step, sim_time=num_steps)
                                  
                                  
    controls = [[c[0], c[1]] for c in controls]
    apply_controls(xp_client, throttle_controller, rudder_controller, controls, sample_time, time_step, receding_horizon)
    print("------------------------------------------------------------")
