# Single vertical position (altitude/depth) measurement.
#
# Units & sign (REP-103): meters, +Z up. Negative altitude indicates depth
# below the chosen reference.
#
# Semantics (REP-105):
# - 'altitude' is the vertical position along the +Z axis of header.frame_id.
# - header.frame_id is the frame in which the measurement is expressed
#   (e.g., base_link, imu_link, map, earth).
#
# Reference:
# The 'reference' field indicates what this measurement is relative to:
#   REFERENCE_UNKNOWN (0): unspecified or mixed semantics.
#   REFERENCE_DATUM   (1): relative to a vertical datum / ellipsoid / geoid.
#                         When used, 'datum' should name the model, e.g.
#                         "WGS84", "EGM96", "MSL", or a local datum identifier.
#   REFERENCE_SURFACE (2): relative to a local physical surface (ground, seabed,
#                         water surface). Typical for AGL/height-above-surface sensors.
#
# Uncertainty:
# - 'variance' is the measurement variance in m^2; 0.0 indicates unknown.

# Reference enumeration
uint8 REFERENCE_UNKNOWN=0
uint8 REFERENCE_DATUM=1
uint8 REFERENCE_SURFACE=2

# Timestamp and frame of the measurement
std_msgs/Header header        # timestamp is when the altitude was measured;
                              # frame_id is the frame whose +Z defines "up"

# Scalar vertical position
float64 altitude              # Vertical position in meters (+Z up); negative for depth

# Measurement uncertainty
float64 variance              # 0 is interpreted as variance unknown

# Optional name of the datum/surface
string datum                  # Name of vertical datum or surface if applicable
                              # (e.g., "WGS84", "EGM96", "MSL", "GROUND", "SEA_SURFACE");
                              # empty if unknown or not applicable
