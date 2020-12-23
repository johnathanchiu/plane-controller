import math
    

thr_gain = 0.10  # gain for plane throttle
he_gain = 0.10 # gain for heading error

def control(xplane, dv, rv, rt, heading_error):
    '''
    params
    dv: desired ground speed
    rv: observed ground speed
    rt: observed throttle
    heading_error: error in heading
    '''
    rudder = he_gain * heading_error
    throttle = max(0, min(1, rt + ((dv - rv) * thr_gain)))
    xplane.sendCTRL([0.0, 0.0, rudder, throttle])
