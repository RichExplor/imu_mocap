# IMU姿态数组消息，包含一帧20个IMU的姿态数据
builtin_interfaces/Time stamp
string frame_id
int32 imu_count
ImuOrientation[] orientations
