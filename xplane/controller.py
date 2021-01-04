from geometry import compute_heading_error
from definitions import XPlaneDefs

import numpy as np
import math

import time

class PID:

    def __init__(self, P=0.2, I=0.0, D=0.0, windup=20.0, sample=1.0):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.windup_guard = windup
        self.sample_time = sample

        self.clear()

    def clear(self):
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0

        self.last_error = 0.0

        self.current_time = time.time()
        self.last_time = self.current_time

        self.output = 0.0

    def update(self, feedback):

        error = feedback

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

        return self.output



def set_control(client, rudder, throttle):
    client.sendCTRL([0.0, 0.0, rudder, throttle])


def compute_rudder(rudder_controller, desired_heading, real_heading):
    heading_err = compute_heading_error(desired_heading, real_heading)
    rudder = rudder_controller.update(heading_err)
    rudder_input = np.clip(rudder, -1.0, 1.0)
    return rudder_input


def compute_throttle(throttle_controller, groundspeed, reference_speed):
    error = reference_speed - groundspeed
    throttle = throttle_controller.update(error)
    throttle_input = np.clip(throttle, -0.5, 0.5)
    return throttle_input


def apply_controls(client, throttle_controller, rudder_controller, controls, sample_time=0.1,
                    time_step=1, num_states=5):
    for control in controls[:num_states]:
        velocity_control, heading_control = control
        t0 = time.time()
        while time.time() - t0 < time_step:
            gs, psi = np.squeeze(np.array(client.getDREFs(XPlaneDefs.control_dref)))
            rudder = compute_rudder(rudder_controller, heading_control, psi)
            throttle = compute_throttle(throttle_controller, gs, velocity_control)
            set_control(client, rudder, throttle)
            time.sleep(sample_time)
