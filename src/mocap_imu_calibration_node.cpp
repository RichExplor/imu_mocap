#include "imu_mocap/mocap_imu_calibration.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HanWei::Mocap::MocapImuCalibration>());
  rclcpp::shutdown();
  return 0;
}
