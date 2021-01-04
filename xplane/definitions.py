
class XPlaneDefs:

    # True North in XPlane is 270 degrees in unit circle coordinates
    zero_heading = 270

    groundspeed_dref = "sim/flightmodel/position/groundspeed"           # m/s
    heading_dref = "sim/flightmodel/position/psi"                       # in degrees
    throttle_dref = "sim/flightmodel/engine/ENGN_thro"
    position_dref = ["sim/flightmodel/position/local_x",
                     "sim/flightmodel/position/local_y",
                     "sim/flightmodel/position/local_z"]
    control_dref = [groundspeed_dref, heading_dref, throttle_dref]
