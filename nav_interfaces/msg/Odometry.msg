# This represents an estimate of a position and velocity in free space.
# The pose in this message should be specified in the coordinate frame given by header.frame_id
# The twist in this message should be specified in the coordinate frame given by the child_frame_id

std_interfaces/Header header
string child_frame_id
geometry_interfaces/PoseWithCovariance pose
geometry_interfaces/TwistWithCovariance twist
