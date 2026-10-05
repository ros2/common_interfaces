# Bounds which are useful in several contexts, e.g. to avoid spawning with other object overlap,
# or parking in a spot that is too small.
# Certain goals or points might be valid for a small object, but not suitable for a large one,
# or a differently shaped one.
# Bounds can be also checked to ensure certain scenario conditions are met.
# For entities, these limits are relative to entity's canonical link transform, following ROS rep-103 convention.

# As bounds are optional in most interfaces, TYPE_EMPTY signifies empty bounds, to be understood as "unbounded".
# Otherwise, the fields are expected to define a valid volume.
# For spawning with a named pose, you should check whether the entity simulation model fits within the bounds
# before calling SpawnEntity, to avoid overlaps and unstable behavior.

# bounds type
uint8 TYPE_EMPTY         = 0      # No bounds. The points vector will be empty.
uint8 TYPE_BOX           = 1      # Axis-aligned bounding box, points field should have two values,
                                  # which are upper right and lower left corners of the box.
uint8 TYPE_CONVEX_HULL   = 2      # Points define a convex hull (at least 3 non-collinear points).
uint8 TYPE_SPHERE        = 3      # A sphere with center and radius. First element of points vector is the center.
                                  # The x field of the second point of the vector is the radius (y and z are ignored).


uint8 type
geometry_msgs/Vector3[] points           # Points defining the bounded area. Check type field to determine semantics.
                                         # Valid sizes are zero (no bounds), 2 (sphere or box, depending on type field),
                                         # and 3 or more (convex hull).
