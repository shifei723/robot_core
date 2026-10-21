# State time (depth exposure time), not inference completion time.
std_msgs/Header header
uint64 epoch
bool healthy
string status
# A healthy empty list means successful perception with no tracked threats.
# Unhealthy arrays NEVER authorize motion or deletion of previous threats.
DynamicObstacle[] obstacles
