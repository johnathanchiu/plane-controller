
class XPlaneDefs:

    # True North in XPlane is 270 degrees in unit circle coordinates
    zero_heading = 270

    groundspeed_dref = "sim/flightmodel/position/groundspeed"           # m/s
    heading_dref = "sim/flightmodel/position/psi"                       # in degrees
    throttle_dref = "sim/flightmodel/engine/ENGN_thro"
    position_dref = ["sim/flightmodel/position/local_x",
                     "sim/flightmodel/position/local_y",
                     "sim/flightmodel/position/local_z"]
    condition_drefs = ["sim/weather/runway_friction",
                   	   "sim/weather/wind_direction_degt[0]",
                   	   "sim/weather/wind_speed_kt[0]"]
    velocity_drefs = ["sim/flightmodel/position/local_vx",
                      "sim/flightmodel/position/local_vy",
                      "sim/flightmodel/position/local_vz",
                      "sim/flightmodel/position/local_ax",
                      "sim/flightmodel/position/local_ay",
                      "sim/flightmodel/position/local_az",
                      "sim/flightmodel/position/P",
                      "sim/flightmodel/position/Q",
                      "sim/flightmodel/position/R"]
    fuel_dref = "sim/flightmodel/weight/m_fuel"
    park_dref = "sim/flightmodel/controls/parkbrake"
    fix_systems = "sim/operation/fix_all_systems"
    quaternion_dref = "sim/flightmodel/position/q"

    control_dref = [groundspeed_dref, heading_dref, throttle_dref]
