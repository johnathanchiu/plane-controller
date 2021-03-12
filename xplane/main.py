from solver import solve_states
from controller import apply_controls, takeoff, PID
from definitions import XPlaneDefs
from geometry import kn_to_ms, rejection_dist, rotate

from xpc import XPlaneConnect
import scenic.syntax.translator as translator

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
turning_constraint = 10 # degrees

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

### Scenic program setup ###
# sampler = translator.scenarioFromFile('conditions.sc')
# scene, _ = sampler.generate()

# params = scene.params

# wind_speed = scene.params['wind_speed']
# wind_direction = scene.params['wind_direction']
# friction = scene.params['friction']
ws_bins = (0, 10, 1)
wh_bins = (-20, 20, 1)

wind_speed = np.random.randint(ws_bins[0], ws_bins[1])
wind_degrees = np.random.randint(wh_bins[0], wh_bins[1])

wind_direction = runway_heading + wind_degrees 

xp_wind_direction = -1 * wind_degrees + runway_heading # since positive rotation to right
xp_wind_direction += 180 # wind is counter clockwise of true north

print("Using (wind speed, wind heading):", wind_speed, wind_degrees)

friction = 0
xp_client.sendDREFs(XPlaneDefs.condition_drefs, [friction, xp_wind_direction, wind_speed])

### initialize starting states ###

origin_x = runway_origin_x
origin_z = runway_origin_z

start_x, start_y, start_z = xp_client.getDREFs(XPlaneDefs.position_dref)
start_y = start_y[0]
xp_client.sendDREFs(XPlaneDefs.position_dref, [origin_x, start_y, origin_z])

init_heading = xp_client.getDREF(XPlaneDefs.heading_dref)[0]
# add true north heading
init_heading += XPlaneDefs.zero_heading

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

cle = 0

start = time.time()
for t in range(int(simulation_steps // receding_horizon)):

    if time.time() - start > 1:
        wind_speed = np.random.randint(ws_bins[0], ws_bins[1])
        wind_degrees = np.random.randint(wh_bins[0], wh_bins[1])

        wind_direction = runway_heading + wind_degrees 

        xp_wind_direction = -1 * wind_degrees + runway_heading # since positive rotation to right
        xp_wind_direction += 180 # wind is counter clockwise of true north

        print("Using (wind speed, wind heading):", wind_speed, wind_degrees)

        friction = 0

        xp_client.sendDREFs(XPlaneDefs.condition_drefs, [friction, xp_wind_direction, wind_speed])
        start = time.time()

    read_drefs = XPlaneDefs.control_dref + XPlaneDefs.position_dref
    gs, psi, throttle, x, _, z = xp_client.getDREFs(read_drefs)
    gs, psi, throttle, x, z = gs[0], psi[0], throttle[0], x[0], z[0]
    d_x, d_z = rotate(x - origin_x, z - origin_z, -(runway_heading + XPlaneDefs.zero_heading - 360))
    print("POSITION ON RUNWAY:", d_x, d_z)
    start_time = time.time()
    if TAKEOFF and gs > desired_velocity and abs(psi - runway_heading) < 10 and \
        d_x - TAKEOFF_DIST > 0:
        takeoff(xp_client)
        break
    # TODO: add zero heading in solver
    new_init_states = [x - origin_x, z - origin_z, gs, psi + XPlaneDefs.zero_heading]
    desired_states = [desired_x, desired_z, desired_velocity]
    winds = [[kn_to_ms(wind_speed), (wind_direction + XPlaneDefs.zero_heading)]]
    if SAMPLE:
        winds = np.concatenate((winds, np.column_stack((np.random.randint(ws_bins[0], ws_bins[1] + 1, size=num_environment_samples), 
                                np.random.randint(wh_bins[0], wh_bins[1] + 1, size=num_environment_samples) + XPlaneDefs.zero_heading))))

    cld = rejection_dist(desired_x, desired_z, x - origin_x, z - origin_z)
    cle = max(cld, cle)

    controls, _, _, _ = solve_states(new_init_states, desired_states, winds, plane_specs, acceleration_constraint,
                                  turning_constraint, time_step=time_step, sim_time=num_steps)
    # print('----', time.time() - start_time, 'seconds (1) ----')
    controls = [[c[0], c[1] - XPlaneDefs.zero_heading] for c in controls]
    apply_controls(xp_client, throttle_controller, rudder_controller, controls, sample_time, time_step, receding_horizon)

print("FINISHED!")
print("Maximum Center Line Error (CLE):", np.sqrt(cle))
print("HEADING:", psi - runway_heading)
