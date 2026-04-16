# This represents a Point with reference coordinate frame and timestamp
# (with 32 bits of precision).
# It is recommended to use PointStamped wherever possible instead of
# Point32Stamped.
#
# This recommendation is to promote interoperability.

std_msgs/Header header
Point32 point
