#include "imu_mocap/mocap_imu_calibration.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

MocapImuCalibration::MocapImuCalibration() : Node("mocap_imu_calibration_node") {
  // 声明并读取可配置参数
  imu_raw_topic_    = this->declare_parameter<std::string>("imu_raw_topic", "/imu20/raw");
  calibrated_topic_ = this->declare_parameter<std::string>("calibrated_topic", "/mocap_imu/calibrated");
  max_imu_count_    = this->declare_parameter<int>("max_imu_count", 20);
  default_frame_id_ = this->declare_parameter<std::string>("default_frame_id", "imu20");
  imu_name_prefix_  = this->declare_parameter<std::string>("imu_name_prefix", "imu");
  imu_name_width_   = this->declare_parameter<int>("imu_name_width", 2);

  // 生成 IMU 名称列表
  generateImuNames();

  // 初始化 IMU 数据缓存
  imu_data_.resize(max_imu_count_);
  for (auto& imu : imu_data_) {
    imu.acc.x          = 0.0;
    imu.acc.y          = 0.0;
    imu.acc.z          = 0.0;
    imu.gyro.x         = 0.0;
    imu.gyro.y         = 0.0;
    imu.gyro.z         = 0.0;
    imu.magn.x         = 0.0;
    imu.magn.y         = 0.0;
    imu.magn.z         = 0.0;
    imu.valid          = false;
    imu.invalid_reason = "";
  }

  // 创建订阅器和发布器
  imu_raw_sub_ = this->create_subscription<std_msgs::msg::String>(
      imu_raw_topic_, 10, std::bind(&MocapImuCalibration::imuRawCallback, this, std::placeholders::_1));

  imu_array_pub_ = this->create_publisher<imu_mocap::msg::ImuDataArray>(calibrated_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "MocapImuCalibration Node started");
  RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", imu_raw_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Publishing to:  %s", calibrated_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Max IMU count:  %d", max_imu_count_);
  RCLCPP_INFO(this->get_logger(), "  Default frame:  %s", default_frame_id_.c_str());
}

void MocapImuCalibration::generateImuNames() {
  imu_names_.clear();
  imu_names_.reserve(max_imu_count_);
  for (int i = 1; i <= max_imu_count_; ++i) {
    std::stringstream ss;
    ss << imu_name_prefix_ << std::setw(imu_name_width_) << std::setfill('0') << i;
    imu_names_.push_back(ss.str());
  }
}

void MocapImuCalibration::imuRawCallback(const std_msgs::msg::String::SharedPtr msg) {
  try {
    SimpleJsonParser parser(msg->data);

    builtin_interfaces::msg::Time stamp;
    double                        stamp_sec = parser.getDouble("stamp", 0.0);
    if (stamp_sec > 0) {
      stamp.sec     = static_cast<int32_t>(stamp_sec);
      stamp.nanosec = static_cast<uint32_t>((stamp_sec - stamp.sec) * 1e9);
    } else {
      stamp = this->now();
    }

    std::string frame_id  = parser.getString("frame_id", default_frame_id_);
    int         imu_count = std::min(parser.getInt("imu_count", max_imu_count_), max_imu_count_);

    std::vector<std::vector<double>> acc_data        = parser.getDouble2DArray("acc");
    std::vector<std::vector<double>> gyro_data       = parser.getDouble2DArray("gyro");
    std::vector<std::vector<double>> magn_data       = parser.getDouble2DArray("magn");
    std::vector<bool>                valid_data      = parser.getBoolArray("valid");
    std::vector<std::string>         invalid_reasons = parser.getStringArray("invalid_reason");

    if (valid_data.empty()) {
      valid_data.assign(max_imu_count_, false);
    }
    if (invalid_reasons.empty()) {
      invalid_reasons.assign(max_imu_count_, "");
    }

    for (int i = 0; i < imu_count; ++i) {
      imu_data_[i].stamp    = stamp;
      imu_data_[i].frame_id = frame_id;
      imu_data_[i].imu_name = imu_names_[i];

      if (i < static_cast<int>(acc_data.size()) && acc_data[i].size() >= 3) {
        imu_data_[i].acc.x = acc_data[i][0];
        imu_data_[i].acc.y = acc_data[i][1];
        imu_data_[i].acc.z = acc_data[i][2];
      }
      if (i < static_cast<int>(gyro_data.size()) && gyro_data[i].size() >= 3) {
        imu_data_[i].gyro.x = gyro_data[i][0];
        imu_data_[i].gyro.y = gyro_data[i][1];
        imu_data_[i].gyro.z = gyro_data[i][2];
      }
      if (i < static_cast<int>(magn_data.size()) && magn_data[i].size() >= 3) {
        imu_data_[i].magn.x = magn_data[i][0];
        imu_data_[i].magn.y = magn_data[i][1];
        imu_data_[i].magn.z = magn_data[i][2];
      }
      if (i < static_cast<int>(valid_data.size())) {
        imu_data_[i].valid = valid_data[i];
      }
      if (i < static_cast<int>(invalid_reasons.size())) {
        imu_data_[i].invalid_reason = invalid_reasons[i];
      }
    }

    // // 标定校准 gyro and acc bias、mag软铁矫正、scale factor、misalignment
    // calibrateImuData(imu_data_, imu_count);

    // === 发布 ImuDataArray 消息（完整一帧） ===
    imu_mocap::msg::ImuDataArray array_msg;
    array_msg.stamp     = stamp;
    array_msg.frame_id  = frame_id;
    array_msg.imu_count = imu_count;
    array_msg.imus.assign(imu_data_.begin(), imu_data_.begin() + imu_count);

    imu_array_pub_->publish(array_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published %d IMUs data at stamp: %d.%09d", imu_count, stamp.sec, stamp.nanosec);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing IMU data: %s", e.what());
  }
}