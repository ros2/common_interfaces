# This represents a Polygon with reference coordinate frame and timestamp
# Includes and unique identification field for disambiguating multiple instances

std_msgs/Header header
Point32[] points
int64 id
