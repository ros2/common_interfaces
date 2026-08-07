# Single pressure reading.  This message is appropriate for measuring
# static atmospheric or differential pressure. Requires temperature.
#
# This message is not appropriate for force/pressure contact sensors.

std_msgs/HeaderBounded48 header # timestamp of the measurement
                                # frame_id is the location of the pressure sensor

float32 pressure                # Pressure reading in hectopascals or millibar.

float32 temperature             # Measurement of the Temperature in Degrees Celsius.
