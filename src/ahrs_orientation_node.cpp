/**
 * @file ahrs_orientation_node.cpp
 * @brief AHRS 姿态解算节点入口
 *
 * 订阅 /mocap_imu/calibrated (ImuDataArray)，
 * 使用 Madgwick 滤波器实时解算每个 IMU 的姿态，
 * 发布 /mocap_imu/orientation (ImuOrientationArray)。
 */

#include "imu_mocap/ahrs_orientation.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HanWei::Mocap::AhrsOrientationNode>());
  rclcpp::shutdown();
  return 0;
}
