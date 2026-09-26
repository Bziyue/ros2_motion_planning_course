# Publication time, fixed execution frame "odom". This is not the start time.
std_msgs/Header header
# Absolute simulation time at which piece zero starts. No zero-time sentinel.
builtin_interfaces/Time start_time
# Nonempty; positive finite durations and finite, C2-connected coefficients.
motion2d_interfaces/QuinticPiece2D[] pieces
# Independent, constant yaw reference (rad); not inferred from the tangent.
float64 yaw
