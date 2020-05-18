# This represents a Vector3 with reference coordinate frame and timestamp

# Note that this follows vector semantics with it alwasy being at the origin,
# so the rotational elements of a transform are the only parts applied when transforming.

std_msgs/Header header
Vector3 vector
