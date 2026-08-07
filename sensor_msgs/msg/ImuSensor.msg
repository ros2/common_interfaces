# This is a message to hold data from an IMU (Inertial Measurement Unit) sensor.
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec

std_msgs/HeaderBounded48 header

geometry_msgs/Vector3F32 angular_velocity

geometry_msgs/Vector3F32 linear_acceleration
