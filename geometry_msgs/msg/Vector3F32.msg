# This represents a vector in free space (with 32 bits of precision).
#
# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.
# It is recommended to use Vector3 wherever possible instead of Vector3F32.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending lots of vectors.

float32 x
float32 y
float32 z
