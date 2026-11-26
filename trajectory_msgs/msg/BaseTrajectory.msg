# The header is used to specify the coordinate frame and the reference time for
# the trajectory durations
std_msgs/Header header

# The names of the active waypoint in each trajectory point. These names are
# ordered and must correspond to the values in each trajectory point.
string[] waypoint_name

# Array of trajectory points, which describe the poses, velocities,
# and/or accelerations of the base at each time point.
BaseTrajectoryPoint[] points
