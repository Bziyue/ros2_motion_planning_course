# Acquisition stamp, frame laser. All durations are seconds, not simulation time.
std_msgs/Header header
string backend
int32 beams
# NaN on CPU: these phases are not applicable.
float64 kernel_seconds
float64 download_seconds
float64 scan_seconds
float64 noise_seconds
float64 message_seconds
float64 publish_seconds
# From scan start through publish return; excludes debug markers/diagnostic itself.
# Does NOT measure delivery to a subscriber or RViz rendering.
float64 total_seconds
