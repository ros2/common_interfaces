# Measurement of the Magnetic Field vector at a specific location.

std_msgs/HeaderBounded48 header         # timestamp is the time the
                                           # field was measured
                                           # frame_id is the location and orientation
                                           # of the field measurement

geometry_msgs/Vector3F32 magnetic_field # x, y, and z components of the
                                           # field vector in MicroTesla
                                           # If your sensor does not output 3 axes,
                                           # put NaNs in the components not reported.
