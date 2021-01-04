# set wind conditions
param wind_speed = Range(0, 20)
param wind_direction = Range(0, 360)

# set runway friction
param friction = Uniform(0, 1, 2)

constructor Plane:
    pass

ego = Plane
