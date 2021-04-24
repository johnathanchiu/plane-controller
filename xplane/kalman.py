from geometry import solver_heading, true_heading

import numpy as np
import copy
import math


class AbstractKFilter:

    def reset(self):
        self.steady_state = False
        self.state = np.zeros(self.s)
        self.P = np.zeros((self.s, self.s))

    def set_state(self, state):
        self.state = state       


class KFilter(AbstractKFilter):
    def __init__(self, A, B, Q, C, R, state=None):
        self.A = A
        self.Q = Q
        self.C = C
        self.R = R
        self.B = B
        self.s = A.shape[0]
        self.m = C.shape[0]
        if state is None:
            self.state = np.zeros(self.s)
        else:
            self.state = state
        self.prev_P = np.zeros((self.s, self.s))
        self.P = np.zeros((self.s, self.s))
        self.steady_state = False
    
    def predict(self, control):
        self.prev_P = copy.deepcopy(self.P)
        self.state = self.A@self.state + self.B@control
        self.P = self.A@self.prev_P@self.A.T + self.Q
        
    def update(self, measurement):
        if not self.steady_state:
            self.K = self.P@self.C.T@np.linalg.inv(self.C@self.P@self.C.T + self.R)
            self.P = (np.identity(self.s) - self.K@self.C)@self.P
            if np.allclose(self.P, self.prev_P):
                self.steady_state = True
        innovation = measurement - self.C@self.state
        self.state = self.state + self.K@innovation

    def get_state(self):
        vx, vz = self.state[-2], self.state[-1] 
        pred_gs = math.sqrt(vx**2 + vz**2)
        pred_heading = math.degrees(math.atan(vz / vx)) % 360
        return self.state[0], self.state[1], pred_gs, true_heading(pred_heading)


def find_kalman_controls(control, heading, winds=None, plane_specs=None):
    control_x = control * np.cos(np.radians(solver_heading(heading)))
    control_z = control * np.sin(np.radians(solver_heading(heading)))
    if winds:
        wind_speed, wind_heading = winds
        plane_cs, plane_mass = plane_specs
        wind_acc = wind_speed * plane_cs / plane_mass
        
        control_x += wind_acc * np.cos(np.radians(solver_heading(wind_heading)))
        control_z += wind_acc * np.sin(np.radians(solver_heading(wind_heading)))
    return control_x, control_z