/**
 * @file madgwick_sim_test.cpp
 * @brief Madgwick 滤波器仿真测试
 *
 * 通过仿真生成已知真值姿态轨迹，由真值四元数反推陀螺仪、加速度计、磁力计
 * 测量值（可叠加噪声），送入 MadgwickFilter 解算，再将估计姿态与真值对比，
 * 输出逐帧误差与统计指标（RMSE / 最大误差），验证滤波器正确性。
 *
 * 用法（编译后）:
 *   ros2 run imu_mocap madgwick_sim_test
 *   # 可选参数: <duration_s> <dt_s> <beta> <noise_std>
 *   ros2 run imu_mocap madgwick_sim_test 20.0 0.01 0.1 0.0
 */

#include "imu_mocap/ahrs_orientation.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// 数学工具
// ---------------------------------------------------------------------------

/// 欧拉角 (roll, pitch, yaw) -> 四元数 (ZYX 顺序，与滤波器一致)
Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

/// 四元数 -> 欧拉角 (roll, pitch, yaw)，与 MadgwickFilter::getEulerAngles 一致
Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q) {
  double q0 = q.w();
  double q1 = q.x();
  double q2 = q.y();
  double q3 = q.z();
  double roll  = std::atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
  double pitch = std::asin(std::clamp(2.0 * (q0 * q2 - q3 * q1), -1.0, 1.0));
  double yaw   = std::atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
  return Eigen::Vector3d(roll, pitch, yaw);
}

/// 角度归一化到 [-pi, pi]
double wrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

/// 两个四元数之间的测地距离 (弧度)
double quaternionAngleError(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
  // |q1 ⊗ q2*| 的标量部分 = cos(theta/2)
  double dot = std::abs(q1.dot(q2));
  dot        = std::clamp(dot, -1.0, 1.0);
  return 2.0 * std::acos(dot);
}

// ---------------------------------------------------------------------------
// 仿真轨迹：真值姿态随时间变化
// ---------------------------------------------------------------------------

/// 返回 t 时刻的真值欧拉角 (rad)
Eigen::Vector3d trueEulerAt(double t) {
  double roll  = 0.3 * std::sin(0.5 * t);
  double pitch = 0.2 * std::sin(0.7 * t + 0.5);
  double yaw   = 0.1 * t;  // 持续偏航
  return Eigen::Vector3d(roll, pitch, yaw);
}

// ---------------------------------------------------------------------------
// 仿真传感器测量值生成
// ---------------------------------------------------------------------------

struct ImuMeasurement {
  Eigen::Vector3d gyro;  // rad/s
  Eigen::Vector3d acc;   // 归一化重力方向 (体坐标系)
  Eigen::Vector3d mag;   // 归一化磁场方向 (体坐标系)
};

/**
 * @brief 由相邻真值四元数生成一帧 IMU 测量
 * @param q_prev 上一时刻真值四元数
 * @param q_curr 当前时刻真值四元数
 * @param dt     时间间隔 (s)
 * @param noise  噪声标准差 (0 表示无噪声)
 * @param rng    随机数引擎
 *
 * 约定:
 *   - 重力参考方向 (导航系) g = [0, 0, 1]，体坐标系测量 acc = R^T * g
 *   - 磁场参考方向 (导航系) b = [bx, 0, bz]，体坐标系测量 mag = R^T * b
 *   - 陀螺仪为体坐标系角速度: omega = 2 * vec(q_curr ⊗ q_prev*) / dt
 */
ImuMeasurement generateMeasurement(const Eigen::Quaterniond& q_prev, const Eigen::Quaterniond& q_curr, double dt,
                                    double noise, std::mt19937& rng) {
  ImuMeasurement m;

  // 陀螺仪: 由四元数差分反推体坐标系角速度
  Eigen::Quaterniond dq = q_curr * q_prev.conjugate();
  // dq ≈ [1, 0.5*omega*dt]，取矢量部分
  Eigen::Vector3d omega_body = 2.0 * Eigen::Vector3d(dq.x(), dq.y(), dq.z()) / dt;
  m.gyro = omega_body;

  // 加速度计: 体坐标系下的重力方向 (归一化)
  Eigen::Vector3d g_world(0.0, 0.0, 1.0);
  m.acc = q_curr.conjugate() * g_world;  // R^T * g

  // 磁力计: 导航系参考磁场 b = [bx, 0, bz]
  Eigen::Vector3d b_world(0.5, 0.0, 0.8660254);  // bx, bz (归一化)
  m.mag = q_curr.conjugate() * b_world;

  // 叠加高斯噪声
  if (noise > 0.0) {
    std::normal_distribution<double> dist(0.0, noise);
    m.gyro += Eigen::Vector3d(dist(rng), dist(rng), dist(rng));
    m.acc  += Eigen::Vector3d(dist(rng), dist(rng), dist(rng)) * 0.01;
    m.mag  += Eigen::Vector3d(dist(rng), dist(rng), dist(rng)) * 0.01;
    m.acc.normalize();
    m.mag.normalize();
  }

  return m;
}

// ---------------------------------------------------------------------------
// 主测试流程
// ---------------------------------------------------------------------------

struct TestConfig {
  double duration_s = 20.0;  // 仿真时长 (s)
  double dt         = 0.01;   // 采样间隔 (s)
  double beta       = 0.1;    // Madgwick 增益
  double mag_weight = 1.0;    // 磁力计权重
  double noise_std  = 0.0;    // 噪声标准差 (0=无噪声)
};

struct TestResult {
  double rmse_quat_deg = 0.0;   // 四元数测地距离 RMSE (度)
  double max_quat_deg  = 0.0;   // 四元数最大误差 (度)
  double rmse_roll_deg = 0.0;
  double rmse_pitch_deg = 0.0;
  double rmse_yaw_deg   = 0.0;
  int    num_steps      = 0;
};

TestResult runSimulation(const TestConfig& cfg) {
  std::mt19937 rng(42);  // 固定种子，保证可复现

  MadgwickFilter filter(cfg.beta, cfg.mag_weight);

  int    steps    = static_cast<int>(cfg.duration_s / cfg.dt);
  double sum_q2   = 0.0;
  double max_q    = 0.0;
  double sum_r2   = 0.0, sum_p2 = 0.0, sum_y2 = 0.0;

  // 打印表头
  std::printf("\n%-6s %-9s %-9s %-9s | %-9s %-9s %-9s | %-9s %-9s %-9s | %s\n", "step", "tR(deg)", "tP(deg)", "tY(deg)",
              "eR(deg)", "eP(deg)", "eY(deg)", "dR(deg)", "dP(deg)", "dY(deg)", "qErr(deg)");
  std::printf("%s\n", std::string(110, '-').c_str());

  Eigen::Quaterniond q_prev = eulerToQuaternion(0.0, 0.0, 0.0);

  for (int i = 0; i < steps; ++i) {
    double t = i * cfg.dt;

    // 真值
    Eigen::Vector3d    euler_true = trueEulerAt(t);
    Eigen::Quaterniond q_true     = eulerToQuaternion(euler_true.x(), euler_true.y(), euler_true.z());

    // 生成测量 (用上一真值与当前真值差分)
    ImuMeasurement meas = generateMeasurement(q_prev, q_true, cfg.dt, cfg.noise_std, rng);

    // 滤波器更新 (使用磁力计)
    filter.updateWithMag(meas.gyro, meas.acc, meas.mag, cfg.dt);

    // 估计姿态
    Eigen::Quaterniond q_est  = filter.getQuaternion();
    Eigen::Vector3d    euler_est = quaternionToEuler(q_est);

    // 误差
    double q_err_rad = quaternionAngleError(q_est, q_true);
    double q_err_deg = q_err_rad * 180.0 / M_PI;

    double d_roll  = wrapAngle(euler_est.x() - euler_true.x()) * 180.0 / M_PI;
    double d_pitch = wrapAngle(euler_est.y() - euler_true.y()) * 180.0 / M_PI;
    double d_yaw   = wrapAngle(euler_est.z() - euler_true.z()) * 180.0 / M_PI;

    // 累计统计
    sum_q2 += q_err_deg * q_err_deg;
    max_q   = std::max(max_q, q_err_deg);
    sum_r2 += d_roll * d_roll;
    sum_p2 += d_pitch * d_pitch;
    sum_y2 += d_yaw * d_yaw;

    // 每 100 步打印一行
    if (i % 100 == 0 || i == steps - 1) {
      std::printf("%-6d %-9.2f %-9.2f %-9.2f | %-9.2f %-9.2f %-9.2f | %-9.2f %-9.2f %-9.2f | %.3f\n", i,
                  euler_true.x() * 180.0 / M_PI, euler_true.y() * 180.0 / M_PI, euler_true.z() * 180.0 / M_PI,
                  euler_est.x() * 180.0 / M_PI, euler_est.y() * 180.0 / M_PI, euler_est.z() * 180.0 / M_PI, d_roll,
                  d_pitch, d_yaw, q_err_deg);
    }

    q_prev = q_true;
  }

  TestResult res;
  res.num_steps      = steps;
  res.rmse_quat_deg = std::sqrt(sum_q2 / steps);
  res.max_quat_deg  = max_q;
  res.rmse_roll_deg = std::sqrt(sum_r2 / steps);
  res.rmse_pitch_deg = std::sqrt(sum_p2 / steps);
  res.rmse_yaw_deg   = std::sqrt(sum_y2 / steps);
  return res;
}

}  // namespace

int main(int argc, char** argv) {
  TestConfig cfg;

  // 解析可选命令行参数
  if (argc > 1) cfg.duration_s = std::atof(argv[1]);
  if (argc > 2) cfg.dt         = std::atof(argv[2]);
  if (argc > 3) cfg.beta       = std::atof(argv[3]);
  if (argc > 4) cfg.noise_std  = std::atof(argv[4]);

  std::printf("=== Madgwick 滤波器仿真测试 ===\n");
  std::printf("配置: duration=%.1fs  dt=%.4fs  beta=%.3f  mag_weight=%.3f  noise_std=%.4f\n", cfg.duration_s, cfg.dt,
              cfg.beta, cfg.mag_weight, cfg.noise_std);

  TestResult res = runSimulation(cfg);

  std::printf("\n=== 统计结果 (共 %d 步) ===\n", res.num_steps);
  std::printf("四元数测地距离 RMSE : %.4f deg\n", res.rmse_quat_deg);
  std::printf("四元数最大误差      : %.4f deg\n", res.max_quat_deg);
  std::printf("Roll  RMSE          : %.4f deg\n", res.rmse_roll_deg);
  std::printf("Pitch RMSE          : %.4f deg\n", res.rmse_pitch_deg);
  std::printf("Yaw   RMSE          : %.4f deg\n", res.rmse_yaw_deg);

  // 判定通过阈值: 无噪声情况下稳态误差应较小
  bool pass = (res.rmse_quat_deg < 5.0);
  std::printf("\n判定: %s (阈值 RMSE < 5.0 deg)\n", pass ? "PASS ✓" : "FAIL ✗");

  return pass ? 0 : 1;
}