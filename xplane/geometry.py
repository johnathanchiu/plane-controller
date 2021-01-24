import numpy as np
import math

from definitions import XPlaneDefs


# compute quaternion for main plane pitch, roll, heading
def quaternion_for(theta, phi, psi):
    # (conversion math from http://www.xsquawkbox.net/xpsdk/mediawiki/MovingThePlane)
    sin_theta, cos_theta = math.sin(theta/2), math.cos(theta/2)
    sin_phi, cos_phi = math.sin(phi/2), math.cos(phi/2)
    sin_psi, cos_psi = math.sin(psi/2), math.cos(psi/2)
    quaternion = [
        cos_psi * cos_theta * cos_phi + sin_psi * sin_theta * sin_phi,
        cos_psi * cos_theta * sin_phi - sin_psi * sin_theta * cos_phi,
        cos_psi * sin_theta * cos_phi + sin_psi * cos_theta * sin_phi,
        -cos_psi * sin_theta * sin_phi + sin_psi * cos_theta * cos_phi
    ]
    return quaternion


def kn_to_ms(knots):
    return knots * 0.51444


def compute_heading_error(desired, real):
    '''
    param desired: desired heading
    param real: current heading
    return: heading error
    '''
    error, sign = desired - real, 1
    phi = abs(error) % 360
    if not (0 <= error <= 180 or -360 <= error <= -180):
        sign = -1
    if phi > 180:
        return (360 - phi) * sign
    return phi * sign


def rejection_dist(desired_x, desired_y, curr_x, curr_y):
    # TODO: fix polyline to start from an initial given point
    a = np.array([desired_x, desired_y])
    b = np.array([curr_x, curr_y])
    projection = a@b / np.linalg.norm(a, ord=2) ** 2 * a
    proj_x, proj_y = projection
    return np.linalg.norm([curr_x - proj_x, curr_y - proj_y], ord=2) ** 2


def rotate(x, y, angle):
    return [x * np.cos(np.radians(angle)) - y * np.sin(np.radians(angle)),
            x * np.sin(np.radians(angle)) + y * np.cos(np.radians(angle))]


def solver_heading(angle):
    return angle + XPlaneDefs.zero_heading
    

def true_heading(angle):
    return angle - XPlaneDefs.zero_heading
    

def fix_heading(angle):
    return ((h % 360) + 360) % 360
