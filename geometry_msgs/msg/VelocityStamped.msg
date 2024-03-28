# This expresses the timestamped velocity of a frame 'body_frame_id' in the reference frame 'header.frame_id' expressed from arbitrary observation pose 'observation_pose'.
# The message is often used in simplified versions where:
# - observation pose is not defined, i.e., it is assumed that the observation pose is the origin of the 'header.frame_id';
# - if the observation pose is an established frame, you may set its frame_id only in the pose (since the relative pose to that frame is Identity);
# - 'body_frame_id' and 'header.frame_id' are identical, i.e., the velocity is observed and defined in the local coordinates system of the body 
#   which is the usual use-case in mobile robotics.

std_msgs/Header header
string body_frame_id
PoseStamped observation_pose
Twist velocity
