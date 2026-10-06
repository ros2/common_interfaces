# Single vertical position (altitude/depth) measurement.
#
# Units & sign (REP-103): meters, +Z up. Negative altitude indicates depth.
#
# Semantics (REP-105):
# - 'altitude' is the height (or depth if negative) of the origin of header.frame_id,
#   measured vertically in the selected 'reference'.
# - header.frame_id identifies the point this measurement applies to.
#
# Reference:
# The 'reference' field indicates what this measurement is relative to.
#
# Uncertainty:
# - 'variance' is the measurement variance in m^2; 0.0 indicates unknown.

# Reference enumeration
uint8 REFERENCE_UNKNOWN=0
uint8 REFERENCE_MSL=1            # altitude above mean sea level (as provided by the sensor/system)
uint8 REFERENCE_WGS84=2          # altitude above WGS84 ellipsoid (matches NavSatFix semantics)
uint8 REFERENCE_EGM96=3          # altitude relative to EGM96 geoid model
uint8 REFERENCE_LOCAL_DATUM=4    # altitude relative to a publisher-defined local vertical datum
uint8 REFERENCE_SURFACE=5        # altitude relative to a local physical surface (ground, seabed, water surface, etc.)

# Timestamp and point of the measurement
std_msgs/Header header  # stamp: measurement time; frame_id: measurement point

# Scalar vertical position
float64 altitude        # Vertical position in meters (+Z up); negative for depth

# Measurement uncertainty
float64 variance        # 0 is interpreted as variance unknown

# Which reference 'altitude' is relative to (see REFERENCE_* constants above)
uint8 reference
