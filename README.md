# drone_explorer
Exploration using UAV in ROS


## Msgs and topics

hagen_msgs/PoseCommand
```
uint8 TRAJECTORY_STATUS_EMPTY=0
uint8 TRAJECTORY_STATUS_READY=1
uint8 TRAJECTORY_STATUS_COMPLETED=3
uint8 TRAJECTROY_STATUS_ABORT=4
uint8 TRAJECTORY_STATUS_ILLEGAL_START=5
uint8 TRAJECTORY_STATUS_ILLEGAL_FINAL=6
uint8 TRAJECTORY_STATUS_IMPOSSIBLE=7
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Point position
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 velocity
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 state_vector
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 acceleration
  float64 x
  float64 y
  float64 z
float64 yaw
float64 yaw_dot
float64[3] kx
float64[3] kv
uint32 trajectory_id
uint8 trajectory_flag
```