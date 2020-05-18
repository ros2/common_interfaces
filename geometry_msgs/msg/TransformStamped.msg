# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id at the time of header.stamp
#
# This message is mostly used by the
# <a href="http://www.ros.org/wiki/tf2">tf2</a> package.
# See its documentation for more information.
#
# The child_frame_id is necessary in addition to the frame_id
# in the Header to communication the full reference for the transform
# in a self contained message. 

std_msgs/Header header
string child_frame_id # the frame id of the child frame
Transform transform
