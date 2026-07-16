#pragma once

#include "imu_mocap/json_parser.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <imu_mocap/msg/imu_data.hpp>
#include <imu_mocap/msg/imu_data_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

/**
 * @brief IMU 动捕标定节点
 *
 * 订阅原始 IMU JSON 字符串数据，解析并校准后发布 ImuDataArray。
 * 所有可配置参数通过 ROS2 参数系统从 YAML 文件加载。
 */
class MocapImuCalibration : public rclcpp::Node {
public:
  MocapImuCalibration();

private:
  /** @brief 原始 IMU 数据回调函数 */
  void imuRawCallback(const std_msgs::msg::String::SharedPtr msg);

  /** @brief 生成 IMU 名称列表 */
  void generateImuNames();

  // ---- 可配置参数 ----
  std::string imu_raw_topic_;    ///< 输入话题（原始 JSON 字符串）
  std::string calibrated_topic_; ///< 输出话题（标定后 IMU 数据数组）
  int         max_imu_count_;    ///< 最大 IMU 数量
  std::string default_frame_id_; ///< 默认 frame_id
  std::string imu_name_prefix_;  ///< IMU 名称前缀
  int         imu_name_width_;   ///< IMU 名称数字补零宽度

  // ---- 运行时数据 ----
  std::vector<std::string>             imu_names_; ///< IMU 名称列表
  std::vector<imu_mocap::msg::ImuData> imu_data_;  ///< IMU 数据缓存

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     imu_raw_sub_;
  rclcpp::Publisher<imu_mocap::msg::ImuDataArray>::SharedPtr imu_array_pub_;
};