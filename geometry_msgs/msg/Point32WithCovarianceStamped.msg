# This expresses an estimated point with a reference coordinate frame and timestamp
# (with 32 bits of precision).
# It is recommended to use PointWithCovarianceStamped wherever possible instead of
# Point32WithCovarianceStamped.
#
# This recommendation is to promote interoperability.

std_msgs/Header header
Point32WithCovariance point
