# The min/max distance that reported objects must be from the sensor
float64 object_min_sensor_range
float64 object_max_sensor_range

# The min/max size that reported objects must be, in terms of
# spherical radius from their averaged center point.
float64 object_min_spherical_radius
float64 object_max_spherical_radius

# When any of the above ranges is below zero,
# it is interpreted as +/- infinity (aka. don't care).
float64 INFINITE=-1

