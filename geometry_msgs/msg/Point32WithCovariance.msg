# This represents a point in free space with uncertainty (with 32 bits of precision).
# It is recommended to use PointWithCovariance wherever possible instead of
# Point32WithCovariance.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.

Point32 point

# Row-major representation of the 3x3 covariance matrix
# In order, the parameters are:
# (x, y, z)
float32[9] covariance
