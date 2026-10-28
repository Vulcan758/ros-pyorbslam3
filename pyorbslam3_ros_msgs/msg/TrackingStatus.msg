# Per-frame ORB-SLAM3 tracking status, published by pyorbslam3_ros/slam_node.

std_msgs/Header header        # stamp = image stamp, frame_id = map frame

int8 SYSTEM_NOT_READY = -1
int8 NO_IMAGES_YET = 0
int8 NOT_INITIALIZED = 1
int8 OK = 2
int8 RECENTLY_LOST = 3
int8 LOST = 4
int8 OK_KLT = 5
int8 state                    # mirrors pyorbslam3.TrackingState / ORB-SLAM3 eTrackingState

bool pose_valid
bool imu_initialized          # always false in non-inertial modes
bool localization_mode
uint32 num_tracked_points
uint64 frames_total           # since start or last reset
uint64 frames_ok
uint64 frames_skipped         # dropped before tracking (IMU gaps, bad encoding, backward time)
