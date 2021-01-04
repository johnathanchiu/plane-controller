import numpy as np
import math

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
