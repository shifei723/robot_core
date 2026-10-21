uint8 TENTATIVE=0
uint8 DYNAMIC=1
uint8 COASTING=2
string id
uint8 state
bool observed
builtin_interfaces/Time last_observed
# Position and velocity are BOTH expressed in array.header.frame_id.
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 radius
# Row-major covariance of [x,y,vx,vy], expressed in array.header.frame_id.
float64[16] covariance
