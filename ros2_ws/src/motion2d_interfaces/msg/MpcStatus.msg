# Observation time used for the solve; frame_id=odom.
std_msgs/Header header
# Only solved authorizes the optimized first force; other states use fresh braking.
string status
uint32 iterations
# Wall time for QP assembly + solve + warm-start update; not the whole ROS pipeline.
float64 compute_seconds
float64 primal_residual
float64 dual_residual
# NaN when the result is not solved; never present failure as zero violation.
float64 max_violation
