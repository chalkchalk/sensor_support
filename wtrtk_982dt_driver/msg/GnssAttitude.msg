# Parsed WTRTK-982DT GPHPR output.

uint8 QUALITY_INVALID=0
uint8 QUALITY_SINGLE=1
uint8 QUALITY_CODE_DIFFERENTIAL=2
uint8 QUALITY_RTK_FIXED=4
uint8 QUALITY_RTK_FLOAT=5
uint8 QUALITY_DEAD_RECKONING=6
uint8 QUALITY_MANUAL=7
uint8 QUALITY_WIDE_LANE=8
uint8 QUALITY_SBAS=9

std_msgs/Header header

# Actual NMEA sentence identifier, for example GNHPR.
string source_sentence
# UTC field copied from HPR (hhmmss.ss).
string utc_time

# Receiver heading: clockwise from true north, before mounting correction.
float64 raw_heading_deg
# Vehicle heading after heading_offset_deg, clockwise from true north.
float64 heading_deg
float64 pitch_deg
float64 roll_deg
# ROS ENU yaw derived from heading_deg, in [-pi, pi) radians.
float64 yaw_enu_rad

# Receiver QF field. It describes the position solution, not an independent
# heading-accuracy estimate.
uint8 solution_quality
uint16 satellites
float64 correction_age_sec
string station_id

# True only when the configured checksum policy, fields, QF and freshness pass.
bool data_valid
