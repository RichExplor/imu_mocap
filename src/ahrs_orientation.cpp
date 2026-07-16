#include "imu_mocap/ahrs_orientation.hpp"

#include <algorithm>

constexpr double Deg2Rad = M_PI / 180.0;
constexpr double Rad2Deg = 180.0 / M_PI;

namespace HanWei {
namespace Mocap {

MadgwickFilter::MadgwickFilter(double beta, double mag_weight)
    : beta_(beta), mag_weight_(mag_weight), q_(1.0, 0.0, 0.0, 0.0), initialized_(false) {
}

// 纯 IMU 更新（不带磁力计）
void MadgwickFilter::update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc, double dt) {
  updateWithMag(gyro, acc, Eigen::Vector3d::Zero(), dt);
}

// MARG 统一更新（带磁力计）
void MadgwickFilter::updateWithMag(const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc, const Eigen::Vector3d& mag,
                                   double dt) {
  // 1. 首次接收数据时，使用加速度计初始化姿态
  if (!initialized_) {
    Eigen::Vector3d a    = acc;
    double          norm = a.norm();
    if (norm < 1e-6) {
      return;
    }
    a /= norm;

    // 从加速度计计算 roll 和 pitch
    double roll  = std::atan2(a.y(), a.z());
    double pitch = -std::asin(std::max(-1.0, std::min(1.0, a.x()))); // 增加防 NaN 截断

    // 从 roll 和 pitch 构建初始四元数（假设 Yaw = 0）
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);

    q_.w() = cr * cp;
    q_.x() = sr * cp;
    q_.y() = cr * sp;
    q_.z() = sr * sp;

    initialized_ = true;
    return;
  }

  // 2. 归一化加速度计数据
  Eigen::Vector3d a      = acc;
  double          a_norm = a.norm();
  if (a_norm < 1e-6) {
    return;
  }
  a /= a_norm;

  // 3. 检查并归一化磁力计数据
  Eigen::Vector3d m       = mag;
  double          m_norm  = m.norm();
  bool            use_mag = (m_norm > 1e-6);
  if (use_mag) {
    m /= m_norm;
  }

  // 4. 计算加速度计梯度
  Eigen::Quaterniond grad_acc = computeAccGradient(q_, a);
  Eigen::Quaterniond grad     = grad_acc; // 基础梯度

  // 5. 如果有有效的磁力计数据，进行梯度融合（标准 Madgwick MARG 做法）
  if (use_mag) {
    Eigen::Quaterniond grad_mag = computeMagGradient(q_, m);
    // 融合梯度：grad = grad_acc + mag_weight_ * grad_mag
    grad.coeffs() += mag_weight_ * grad_mag.coeffs();
  }

  // 6. 统一归一化梯度
  double grad_norm = grad.coeffs().norm();
  if (grad_norm > 1e-6) {
    grad.coeffs() /= grad_norm;
  }

  // 7. 计算陀螺仪导数: q_dot = 0.5 * q * [0, ωx, ωy, ωz]
  Eigen::Quaterniond q_gyro(0.0, gyro.x(), gyro.y(), gyro.z());
  Eigen::Quaterniond q_dot = q_ * q_gyro;
  q_dot.coeffs() *= 0.5;

  // 8. 融合更新：q = q + (q_dot - beta * grad) * dt
  q_.coeffs() += (q_dot.coeffs() - beta_ * grad.coeffs()) * dt;

  // 9. 归一化四元数
  q_.normalize();
}

Eigen::Quaterniond MadgwickFilter::getQuaternion() const {
  return q_;
}

// 转换欧拉角（增加防 NaN 截断）
Eigen::Vector3d MadgwickFilter::getEulerAngles() const {
  double q0 = q_.w();
  double q1 = q_.x();
  double q2 = q_.y();
  double q3 = q_.z();

  double roll  = std::atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
  double pitch = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (q0 * q2 - q3 * q1)))); // 防 NaN 截断
  double yaw   = std::atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));

  return Eigen::Vector3d(roll, pitch, yaw);
}

void MadgwickFilter::reset() {
  q_           = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  initialized_ = false;
}

void MadgwickFilter::setBeta(double beta) {
  beta_ = beta;
}

void MadgwickFilter::setMagWeight(double mag_weight) {
  mag_weight_ = mag_weight;
}

// 计算加速度计梯度（返回 Quaterniond 规避 Eigen 内存布局问题）
Eigen::Quaterniond MadgwickFilter::computeAccGradient(const Eigen::Quaterniond& q, const Eigen::Vector3d& a) {
  double q0 = q.w();
  double q1 = q.x();
  double q2 = q.y();
  double q3 = q.z();

  // 目标函数 f(q, a) = q* ⊗ a ⊗ q - g
  double f1 = 2.0 * (q1 * q3 - q0 * q2) - a.x();
  double f2 = 2.0 * (q0 * q1 + q2 * q3) - a.y();
  double f3 = 2.0 * (0.5 - q1 * q1 - q2 * q2) - a.z();

  // 梯度 = J^T * f
  double grad_w = -2.0 * q2 * f1 + 2.0 * q1 * f2;
  double grad_x = 2.0 * q3 * f1 + 2.0 * q0 * f2 - 4.0 * q1 * f3;
  double grad_y = -2.0 * q0 * f1 + 2.0 * q3 * f2 - 4.0 * q2 * f3;
  double grad_z = 2.0 * q1 * f1 + 2.0 * q2 * f2;

  return Eigen::Quaterniond(grad_w, grad_x, grad_y, grad_z);
}

// 计算磁力计梯度（已修正数学公式，返回 Quaterniond）
Eigen::Quaterniond MadgwickFilter::computeMagGradient(const Eigen::Quaterniond& q, const Eigen::Vector3d& m) {
  double q0 = q.w();
  double q1 = q.x();
  double q2 = q.y();
  double q3 = q.z();

  // 将磁力计测量值旋转到参考坐标系: h = q ⊗ [0, m] ⊗ q*
  Eigen::Quaterniond m_quat(0.0, m.x(), m.y(), m.z());
  Eigen::Quaterniond q_conj = q.conjugate();
  Eigen::Quaterniond h      = q * m_quat * q_conj;

  double hx = h.x();
  double hy = h.y();
  double hz = h.z();

  // 参考磁场在水平面上的分量
  double bx = std::sqrt(hx * hx + hy * hy);
  double bz = hz;

  // 目标函数 f(q, b, m)
  double f1 = 2.0 * bx * (0.5 - q2 * q2 - q3 * q3) + 2.0 * bz * (q1 * q3 - q0 * q2) - m.x();
  double f2 = 2.0 * bx * (q1 * q2 - q0 * q3) + 2.0 * bz * (q0 * q1 + q2 * q3) - m.y();
  double f3 = 2.0 * bx * (q0 * q2 + q1 * q3) + 2.0 * bz * (0.5 - q1 * q1 - q2 * q2) - m.z();

  // 梯度 = J^T * f (精准推导版)
  double grad_w = -2.0 * bz * q2 * f1 + (-2.0 * bx * q3 + 2.0 * bz * q1) * f2 + 2.0 * bx * q2 * f3;
  double grad_x = 2.0 * bz * q3 * f1 + (2.0 * bx * q2 + 2.0 * bz * q0) * f2 + (2.0 * bx * q3 - 4.0 * bz * q1) * f3;
  double grad_y = (-4.0 * bx * q2 - 2.0 * bz * q0) * f1 + (2.0 * bx * q1 + 2.0 * bz * q3) * f2 +
                  (2.0 * bx * q0 - 4.0 * bz * q2) * f3;
  double grad_z = (-4.0 * bx * q3 + 2.0 * bz * q1) * f1 + (-2.0 * bx * q0 + 2.0 * bz * q2) * f2 + 2.0 * bx * q1 * f3;

  return Eigen::Quaterniond(grad_w, grad_x, grad_y, grad_z);
}

// ============================================================================
// AhrsOrientationNode 实现
// ============================================================================

AhrsOrientationNode::AhrsOrientationNode() : Node("ahrs_orientation_node") {
  // 声明并读取可配置参数
  calibrated_topic_  = this->declare_parameter<std::string>("calibrated_topic", "/mocap_imu/calibrated");
  orientation_topic_ = this->declare_parameter<std::string>("orientation_topic", "/mocap_imu/orientation");
  max_imu_count_     = this->declare_parameter<int>("max_imu_count", 20);
  beta_              = this->declare_parameter<double>("beta", 0.1);
  mag_weight_        = this->declare_parameter<double>("mag_weight", 0.5);
  default_dt_        = this->declare_parameter<double>("default_dt", 0.01);
  max_dt_            = this->declare_parameter<double>("max_dt", 0.1);

  // 初始化滤波器、时间戳和初始化状态
  filters_.resize(max_imu_count_, MadgwickFilter(beta_, mag_weight_));
  last_timestamps_.resize(max_imu_count_);
  initialized_.resize(max_imu_count_, false);

  // 创建订阅器和发布器
  imu_sub_ = this->create_subscription<imu_mocap::msg::ImuDataArray>(
      calibrated_topic_, 10, std::bind(&AhrsOrientationNode::imuArrayCallback, this, std::placeholders::_1));

  orientation_pub_ = this->create_publisher<imu_mocap::msg::ImuOrientationArray>(orientation_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "AHRS Orientation Node started");
  RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", calibrated_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Publishing to:  %s", orientation_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Max IMU count:  %d", max_imu_count_);
  RCLCPP_INFO(this->get_logger(), "  Filter beta:    %.3f", beta_);
  RCLCPP_INFO(this->get_logger(), "  Mag weight:     %.3f", mag_weight_);
}

void AhrsOrientationNode::imuArrayCallback(const imu_mocap::msg::ImuDataArray::SharedPtr msg) {
  try {
    int imu_count = std::min(msg->imu_count, max_imu_count_);

    if (imu_count <= 0 || msg->imus.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty IMU data array");
      return;
    }

    // 构建姿态数组消息
    imu_mocap::msg::ImuOrientationArray orientation_array;
    orientation_array.stamp     = msg->stamp;
    orientation_array.frame_id  = msg->frame_id;
    orientation_array.imu_count = imu_count;
    orientation_array.orientations.resize(imu_count);

    for (int i = 0; i < imu_count; ++i) {
      const auto& imu = msg->imus[i];

      // 检查数据有效性
      if (!imu.valid) {
        RCLCPP_DEBUG(this->get_logger(), "IMU %s data invalid, skipping orientation estimation", imu.imu_name.c_str());
        continue;
      }

      // 获取 IMU 数据
      Eigen::Vector3d acc(Eigen::Vector3d(imu.acc.x, imu.acc.y, imu.acc.z));
      Eigen::Vector3d gyro(Eigen::Vector3d(imu.gyro.x, imu.gyro.y, imu.gyro.z) * Deg2Rad);
      Eigen::Vector3d mag(Eigen::Vector3d(imu.magn.x, imu.magn.y, imu.magn.z));

      // 计算时间间隔
      rclcpp::Time current_time(imu.stamp.sec, imu.stamp.nanosec);
      double       dt = 0.0;

      if (initialized_[i]) {
        dt = (current_time - last_timestamps_[i]).seconds();
        if (dt <= 0.0 || dt > max_dt_) {
          dt = default_dt_;
        }
      } else {
        dt              = default_dt_;
        initialized_[i] = true;
      }

      // 检查磁力计数据是否有效（非零）
      bool mag_valid = (mag.norm() > 1e-6);

      if (mag_valid) {
        filters_[i].updateWithMag(gyro, acc, mag, dt);
      } else {
        filters_[i].update(gyro, acc, dt);
      }

      // 更新时间戳
      last_timestamps_[i] = current_time;

      // 获取姿态结果
      Eigen::Quaterniond quat  = filters_[i].getQuaternion();
      Eigen::Vector3d    euler = filters_[i].getEulerAngles();

      // 填充消息
      auto& orientation_msg    = orientation_array.orientations[i];
      orientation_msg.stamp    = imu.stamp;
      orientation_msg.frame_id = msg->frame_id;
      orientation_msg.imu_name = imu.imu_name;

      orientation_msg.quaternion.w = quat.w();
      orientation_msg.quaternion.x = quat.x();
      orientation_msg.quaternion.y = quat.y();
      orientation_msg.quaternion.z = quat.z();

      orientation_msg.euler.x = euler.x(); // roll
      orientation_msg.euler.y = euler.y(); // pitch
      orientation_msg.euler.z = euler.z(); // yaw

      RCLCPP_DEBUG(this->get_logger(), "[%s]  Roll: %.2f  Pitch: %.2f  Yaw: %.2f", imu.imu_name.c_str(), euler.x(),
                   euler.y(), euler.z());
    }

    // 发布姿态数据
    orientation_pub_->publish(orientation_array);

    RCLCPP_DEBUG(this->get_logger(), "Published %d IMU orientations at stamp: %d.%09d", imu_count, msg->stamp.sec,
                 msg->stamp.nanosec);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing IMU orientation: %s", e.what());
  }
}

Eigen::Vector3d AhrsOrientationNode::quaternionToEuler(const geometry_msgs::msg::Quaternion& q) {
  double q0 = q.w;
  double q1 = q.x;
  double q2 = q.y;
  double q3 = q.z;

  double roll  = std::atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
  double pitch = std::asin(2.0 * (q0 * q2 - q3 * q1));
  double yaw   = std::atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));

  return Eigen::Vector3d(roll, pitch, yaw);
}

} // namespace Mocap
} // namespace HanWei