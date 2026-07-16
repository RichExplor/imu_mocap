#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <imu_mocap/msg/imu_data_array.hpp>
#include <imu_mocap/msg/imu_orientation_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

/**
 * @brief Madgwick AHRS 滤波器
 *
 * 使用陀螺仪、加速度计和磁力计数据估计姿态。
 * 基于 Madgwick 2010 论文提出的梯度下降算法。
 *
 * 参考: Madgwick, S. O. H., Harrison, A. J. L., & Vaidyanathan, R. (2011).
 * "Estimation of IMU and MARG orientation using a gradient descent algorithm."
 * IEEE International Conference on Rehabilitation Robotics.
 */
class MadgwickFilter {
public:
  /**
   * @brief 构造函数
   * @param beta       滤波器增益，表示陀螺仪测量误差（默认 0.16，典型范围 0.01~0.5）
   * @param mag_weight 磁力计修正权重（默认 1.0）
   */
  explicit MadgwickFilter(double beta = 0.16, double mag_weight = 1.0);

  /**
   * @brief 使用陀螺仪和加速度计更新姿态（无磁力计）
   * @param gyro 陀螺仪数据 (rad/s) [x, y, z]
   * @param acc  加速度计数据 (m/s^2) [x, y, z]
   * @param dt   时间间隔 (s)
   */
  void update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc, double dt);

  /**
   * @brief 使用陀螺仪、加速度计和磁力计更新姿态
   * @param gyro 陀螺仪数据 (rad/s) [x, y, z]
   * @param acc  加速度计数据 (m/s^2) [x, y, z]
   * @param mag  磁力计数据 (μT) [x, y, z]
   * @param dt   时间间隔 (s)
   */
  void updateWithMag(const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc, const Eigen::Vector3d& mag, double dt);

  /** @brief 获取当前姿态四元数 */
  Eigen::Quaterniond getQuaternion() const;

  /** @brief 获取欧拉角 (roll, pitch, yaw) 单位: 弧度 */
  Eigen::Vector3d getEulerAngles() const;

  /** @brief 重置滤波器为初始状态 */
  void reset();

  /** @brief 设置滤波器增益 */
  void setBeta(double beta);

  /** @brief 设置磁力计修正权重 */
  void setMagWeight(double mag_weight);

private:
  /**
   * @brief 计算加速度计目标函数的梯度
   * @param q 当前姿态四元数
   * @param a 归一化加速度计测量值
   * @return 梯度向量
   */
  static Eigen::Quaterniond computeAccGradient(const Eigen::Quaterniond& q, const Eigen::Vector3d& a);

  /**
   * @brief 计算磁力计目标函数的梯度
   * @param q 当前姿态四元数
   * @param m 归一化磁力计测量值
   * @return 梯度向量
   */
  static Eigen::Quaterniond computeMagGradient(const Eigen::Quaterniond& q, const Eigen::Vector3d& m);

  double             beta_;        ///< 滤波器增益
  double             mag_weight_;  ///< 磁力计修正权重
  Eigen::Quaterniond q_;           ///< 当前姿态四元数
  bool               initialized_; ///< 是否已初始化
};

/**
 * @brief AHRS 姿态解算节点
 *
 * 订阅标定后的 IMU 数据数组，对每个 IMU 运行 Madgwick 滤波器解算姿态，
 * 发布姿态解算结果数组。所有可配置参数通过 ROS2 参数系统从 YAML 文件加载。
 */
class AhrsOrientationNode : public rclcpp::Node {
public:
  AhrsOrientationNode();

private:
  /** @brief IMU 数据回调函数 */
  void imuArrayCallback(const imu_mocap::msg::ImuDataArray::SharedPtr msg);

  /** @brief 四元数转欧拉角 (roll, pitch, yaw) */
  static Eigen::Vector3d quaternionToEuler(const geometry_msgs::msg::Quaternion& q);

  // ---- 可配置参数 ----
  std::string calibrated_topic_;  ///< 输入话题（标定后 IMU 数据数组）
  std::string orientation_topic_; ///< 输出话题（姿态解算结果数组）
  int         max_imu_count_;     ///< 最大 IMU 数量
  double      beta_;              ///< Madgwick 滤波器增益
  double      mag_weight_;        ///< 磁力计修正权重
  double      default_dt_;        ///< 默认时间间隔 (s)
  double      max_dt_;            ///< 最大允许时间间隔 (s)

  // ---- 运行时数据 ----
  std::vector<MadgwickFilter> filters_;         ///< 每个 IMU 对应的滤波器实例
  std::vector<rclcpp::Time>   last_timestamps_; ///< 每个 IMU 上次更新时间戳
  std::vector<bool>           initialized_;     ///< 每个 IMU 是否已初始化

  rclcpp::Subscription<imu_mocap::msg::ImuDataArray>::SharedPtr     imu_sub_;
  rclcpp::Publisher<imu_mocap::msg::ImuOrientationArray>::SharedPtr orientation_pub_;
};
