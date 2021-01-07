from solver import solve_states
from controller import apply_controls, PID
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

desired_velocity = 27 # m/s

acceleration_constraint = 10 # m/s^2
turning_constraint = 30 # degrees

plane_cs = 27.41 # square meters
plane_mass = 6175 * 0.45359237 # lbs -> kg

plane_specs = [plane_cs, plane_mass]

# solver time step
time_step = 1 # seconds
# solver number of states to solve
num_steps = 8

# controller recompute time step
sample_time = 0.1
# number of seconds to run the controller + solver
simulation_steps = 100

# recompute using solver after t seconds
receding_horizon = 1


# create XPC object
xp_client = XPlaneConnect()

### Scenic program setup ###
sampler = translator.scenarioFromFile('conditions.sc')
scene, _ = sampler.generate()

params = scene.params

wind_speed = scene.params['wind_speed']
wind_direction = scene.params['wind_direction']
# friction = scene.params['friction']
wind_speed = 20
wind_degrees = 40
wind_direction = runway_heading + wind_degrees
wind_direction += 180 # wind is counter clockwise of true north

friction = 0
xp_client.sendDREFs(XPlaneDefs.condition_drefs, [friction, wind_direction, wind_speed])

wind_speed = kn_to_ms(wind_speed)
wind_direction += -2 * wind_degrees # since positive rotation is the right
wind_direction -= 180

### initialize starting states ###

# init_x, _, init_z = np.squeeze(np.array(xp_client.getDREFs(XPlaneDefs.position_dref)))
init_x = runway_origin_x
init_z = runway_origin_z

init_heading = xp_client.getDREF(XPlaneDefs.heading_dref)[0]
# add true north heading
init_heading += XPlaneDefs.zero_heading

# want to end here at t = sim_time
desired_x -= init_x
desired_z -= init_z

time.sleep(1)

xp_client.sendCOMM("sim/operation/fix_all_systems")
# release park brake
xp_client.sendDREF("sim/flightmodel/controls/parkbrake", 0)

time.sleep(1)

controls = None
throttle_controller = PID(2.0, 0.0, 1.0, 10.0, sample_time)
rudder_controller = PID(0.3, 0.5, 0.8, 10.0, sample_time)
for t in range(int(simulation_steps // receding_horizon)):
    read_drefs = XPlaneDefs.control_dref + XPlaneDefs.position_dref
    gs, psi, throttle, x, _, z = xp_client.getDREFs(read_drefs)
    gs, psi, throttle, x, z = gs[0], psi[0], throttle[0], x[0], z[0]
    print(gs, psi)
    # TODO: add zero heading in solver
    new_init_states = [x - init_x, z - init_z, gs, psi + XPlaneDefs.zero_heading]
    desired_states = [desired_x, desired_z, desired_velocity]
    winds = [0, (wind_direction + XPlaneDefs.zero_heading)]

    controls, _, _ = solve_states(new_init_states, desired_states, winds, plane_specs, acceleration_constraint,
                                  turning_constraint, time_step=time_step, sim_time=num_steps)
    controls = [[c[0], c[1] - XPlaneDefs.zero_heading] for c in controls]
    print(controls)
    throttle_controller.clear()
    rudder_controller.clear()
    apply_controls(xp_client, throttle_controller, rudder_controller, controls, sample_time, time_step, receding_horizon)
