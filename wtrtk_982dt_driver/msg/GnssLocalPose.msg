# Local two-dimensional GNSS pose in an east-north coordinate frame.

std_msgs/Header header

# Main antenna position relative to the configured origin, in metres.
# Positive x points east and positive y points north.
float64 x
float64 y

# Vehicle heading after heading_offset_deg is applied.
# Zero is true north and the angle increases clockwise, in [0, 360) degrees.
float64 heading_deg

# True only when position and heading are both valid and time-synchronised.
bool data_valid
