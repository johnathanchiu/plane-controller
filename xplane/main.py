from solver import solve_states
from controller import apply_controls, takeoff, PID
from definitions import XPlaneDefs
from geometry import kn_to_ms

from xpc import XPlaneConnect
import scenic.syntax.translator as translator

import numpy as np
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

acceleration_constraint = 10 # m/s^2
turning_constraint = 40 # degrees

plane_cs = 27.41 # square meters
plane_mass = 6175 * 0.45359237 # lbs -> kg
plane_half_length = 10.82 / 2 # meters

plane_specs = [plane_cs, plane_mass, plane_half_length]

# solver time step
time_step = 1 # seconds
# solver number of states to solve
num_steps = 3

# PID controller recompute time step
sample_time = 0.1
# number of seconds to run the PID controller + solver
simulation_steps = 100

# recompute using solver after t seconds
receding_horizon = 1

TAKEOFF = False
SAMPLE = False

# create XPC object
xp_client = XPlaneConnect()

### Scenic program setup ###
# sampler = translator.scenarioFromFile('conditions.sc')
# scene, _ = sampler.generate()

# params = scene.params

# wind_speed = scene.params['wind_speed']
# wind_direction = scene.params['wind_direction']
# friction = scene.params['friction']
wind_speed = 3
wind_degrees = 2
wind_direction = runway_heading + wind_degrees 
xp_wind_direction = -1 * wind_degrees + runway_heading # since positive rotation to right
xp_wind_direction += 180 # wind is counter clockwise of true north

friction = 0
xp_client.sendDREFs(XPlaneDefs.condition_drefs, [friction, xp_wind_direction, wind_speed])

wind_speed = kn_to_ms(wind_speed)

num_environment_samples = 50
windspeed_lb = wind_speed - 10
windspeed_ub = wind_speed + 10
windheading_lb = wind_direction - 30
windheading_ub = wind_direction + 30

### initialize starting states ###

origin_x = runway_origin_x
origin_z = runway_origin_z

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
for t in range(int(simulation_steps // receding_horizon)):
    read_drefs = XPlaneDefs.control_dref + XPlaneDefs.position_dref
    gs, psi, throttle, x, _, z = xp_client.getDREFs(read_drefs)
    gs, psi, throttle, x, z = gs[0], psi[0], throttle[0], x[0], z[0]
    if TAKEOFF and gs > desired_velocity and abs(psi - runway_heading) < 10:
        takeoff(xp_client)
        break
    # TODO: add zero heading in solver
    new_init_states = [x - origin_x, z - origin_z, gs, psi + XPlaneDefs.zero_heading]
    desired_states = [desired_x, desired_z, desired_velocity]
    winds = [(wind_speed, (wind_direction + XPlaneDefs.zero_heading))]
    if SAMPLE:
        for _ in range(num_environment_samples - 1):
            winds.append((np.random.randint(windspeed_lb, windspeed_ub + 1), 
                          np.random.randint(windheading_lb, windheading_ub + 1) + XPlaneDefs.zero_heading))

    controls, _, _, _ = solve_states(new_init_states, desired_states, winds, plane_specs, acceleration_constraint,
                                  turning_constraint, time_step=time_step, sim_time=num_steps)
    controls = [[c[0], c[1] - XPlaneDefs.zero_heading] for c in controls]
    apply_controls(xp_client, throttle_controller, rudder_controller, controls, sample_time, time_step, receding_horizon)


