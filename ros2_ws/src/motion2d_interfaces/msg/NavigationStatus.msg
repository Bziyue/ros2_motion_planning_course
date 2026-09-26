# Current observation timestamp, frame odom. Durations are measured wall seconds.
std_msgs/Header header
string state
string planner_status
string optimization_status
float64 planning_seconds
uint64 accepted_plans
uint64 stop_count
