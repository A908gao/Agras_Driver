//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef LIVOX_ROS_DRIVER2_BUILTIN_IMU_PROCESSOR_H_
#define LIVOX_ROS_DRIVER2_BUILTIN_IMU_PROCESSOR_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace livox_ros {

/** 重力加速度 (m/s²), 与 ArduPilot GRAVITY_MSS 一致 */
constexpr float kGravityMss = 9.80665f;

/**
 * @brief 内置 Livox IMU 处理配置 —— 全部参照 ArduPilot AP_InertialSensor 参数
 *
 * 对应关系:
 *   calib_enable    ~ INS_GYR_CAL (开机静置自校准)
 *   still_threshold ~ INS_STILL_THRESH (2.5 m/s², Copter 默认)
 *   gyro_cal_diff   ~ GYRO_INIT_MAX_DIFF_DPS (0.1 deg/s)
 *   notch_*         ~ INS_HNTCH_* (谐波陷波, 默认关闭)
 *   gyro_lpf_hz     ~ INS_GYRO_FILTER  (Copter 默认 20 Hz)
 *   accel_lpf_hz    ~ INS_ACCEL_FILTER (Copter 默认 20 Hz)
 *
 * 注意: ArduPilot 在 1 kHz 采样下使用 20 Hz 低通并由 EKF 的 INS_DELAY 补偿群时延;
 * 内置 Livox IMU 仅 200 Hz, 用于 FAST-LIO 时若映射精度受滤波滞后影响,
 * 可适当调高 gyro/accel_lpf_hz (如 80/40)。
 */
struct BuiltinImuConfig {
  // ── 开机静置自校准 (仿 _init_gyro 收敛策略) ──
  bool   calib_enable    = true;
  double calib_duration  = 3.0;    /**< 校准最长收集时间 [s] */
  double still_threshold = 2.5f;   /**< 静置判定: 窗口内加计三轴标准差上限 [m/s²] */
  double gyro_cal_diff   = 0.1;    /**< 相邻 0.5s 窗口陀螺均值差收敛判据 [deg/s] */

  // ── 谐波陷波 (仿 HarmonicNotchFilter, 固定频率模式) ──
  bool   notch_enable    = false;  /**< 仿 INS_HNTCH_ENABLE (默认 0) */
  double notch_freq      = 80.0;   /**< 基频 [Hz] (仿 INS_HNTCH_FREQ) */
  double notch_bw        = 40.0;   /**< 带宽 [Hz] (仿 INS_HNTCH_BW) */
  double notch_att       = 40.0;   /**< 衰减 [dB] (仿 INS_HNTCH_ATT) */
  int    notch_harmonics = 3;      /**< 谐波位掩码 (仿 INS_HNTCH_HMNCS, 3=1/2 次) */

  // ── 低通滤波 (仿 INS_GYRO_FILTER / INS_ACCEL_FILTER) ──
  double gyro_lpf_hz     = 20.0;
  double accel_lpf_hz    = 20.0;

  double nominal_rate_hz = 200.0;  /**< 内置 IMU 名义采样率 (MID-360: 200 Hz) */
  bool   publish_raw     = false;  /**< 额外发布 /livox/imu_raw 供诊断对照 */

  // ── 轴映射/取反 (仿 ext_imu 桥; Agras 固件内置 IMU 全轴反向 → 默认 '-x-y-z') ──
  // 单轴取反是镜像变换, extrinsic_R 无法表示, 必须在数据层修正
  std::string gyro_axis_map  = "-x-y-z";
  std::string accel_axis_map = "-x-y-z";
};

/**
 * @brief ArduPilot 风格的内置 IMU 处理器
 *
 * 处理链 (与 AP_InertialSensor_Backend 一致):
 *   1. 采样率跟踪 (_update_sensor_rate 的 EMA 策略) 并据此计算 dt;
 *   2. 启动静置自校准: 0.5s 窗口均值两两比较收敛 (_init_gyro 策略) —
 *      陀螺零偏估计 + 加速度计幅值归一化 (仿 INS_ACC_BODYFIX, 无姿态假设);
 *   3. 校正: gyro -= bias; accel *= scale (先偏置后滤波, 仿 _rotate_and_correct_*);
 *   4. 陀螺: 谐波陷波 → 低通 (低通必须最后, 压掉陷波引入的噪声);
 *   5. 加计: 低通;
 *   6. NaN/Inf 保护: 复位滤波器并保持上一次输出 (仿 apply_gyro_filters)。
 *
 * 校准完成前按原始数据直通 (仿 ArduPilot _calibrating_gyro 期间不施加偏置)。
 */
class BuiltinImuProcessor {
 public:
  explicit BuiltinImuProcessor(const BuiltinImuConfig &cfg);

  /**
   * @brief 处理一个原始 IMU 采样
   * @param ts_ns      采样时间戳 [ns] (内置 IMU 原始打戳)
   * @param gyro_raw   原始陀螺 [rad/s]
   * @param accel_raw  原始加计 [g]
   * @param gyro_out   处理后陀螺 [rad/s]
   * @param accel_out  处理后加计 [m/s²]
   * @return 始终为 true (NaN 时返回上一有效值)
   */
  bool Process(uint64_t ts_ns,
               const float gyro_raw[3], const float accel_raw[3],
               float gyro_out[3], float accel_out[3]);

  bool  calibrated() const { return calibrated_; }
  float sample_rate_hz() const { return sample_rate_hz_; }
  float gyro_bias(size_t axis) const { return gyro_bias_[axis]; }
  float accel_scale() const { return accel_scale_; }
  bool  raw_passthrough() const { return !calibrated_ && cfg_.calib_enable; }

 private:
  /** 单轴 biquad 陷波器 (移植自 ArduPilot Filter/NotchFilter, Leonard Hall 设计) */
  struct NotchFilter3 {
    bool  initialised = false;
    bool  need_reset = false;
    float b0 = 0, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    float fc = 0, fs = 0, A = 0;
    float ntch1[3] = {0, 0, 0}, ntch2[3] = {0, 0, 0};
    float sig1[3] = {0, 0, 0}, sig2[3] = {0, 0, 0};

    void Init(float sample_freq, float center_freq, float bandwidth, float atten_dB);
    void Reset();
    void Apply(float v[3]);
  };

  /** 变 dt 一阶低通 (仿 ArduPilot Filter/LowPassFilter: alpha = dt/(dt+1/(2πf))) */
  struct Lpf3 {
    float val[3] = {0, 0, 0};
    bool  init[3] = {false, false, false};

    void Reset();
    void Apply(const float in[3], float dt, float cutoff_hz, float out[3]);
  };

  void InitNotches(float sample_rate);
  void UpdateSampleRate(uint64_t ts_ns);
  float ComputeDt(uint64_t ts_ns);
  void CalibrationCollect(uint64_t ts_ns, const float gyro[3], const float accel[3]);
  void FinalizeCalibWindow(uint64_t ts_ns);
  bool FinishCalibration(bool timeout);
  bool OutputsValid(const float gyro[3], const float accel[3]) const;

  BuiltinImuConfig cfg_;
  std::vector<NotchFilter3> notches_;
  float notch_sample_rate_ = 0;   /**< 陷波器当前按此采样率设计, 变化>5% 时重建 */
  Lpf3 gyro_lpf_;
  Lpf3 accel_lpf_;

  // ── 采样率跟踪 (仿 _update_sensor_rate) ──
  float    sample_rate_hz_ = 200.0f;
  uint32_t rate_count_ = 0;
  uint64_t rate_start_ts_ns_ = 0;

  // ── 校准状态 ──
  bool     calibrating_ = true;
  bool     calibrated_ = false;
  uint64_t calib_start_ts_ns_ = 0;

  // 0.5s 校准窗口统计 (仿 _init_gyro "平均 50 点 / 0.5 秒" 策略)
  double win_gyro_sum_[3] = {0, 0, 0};
  double win_accel_sum_[3] = {0, 0, 0};
  double win_accel_sq_[3] = {0, 0, 0};
  uint32_t win_count_ = 0;
  uint64_t win_start_ts_ns_ = 0;
  bool     prev_win_valid_ = false;
  double  prev_win_gyro_mean_[3] = {0, 0, 0};

  // 最近一次静置窗口的候选标定值 (收敛/超时兜底用)
  bool   cand_valid_ = false;
  double cand_gyro_mean_[3] = {0, 0, 0};
  double cand_accel_mean_[3] = {0, 0, 0};

  // ── 校准结果 ──
  float gyro_bias_[3] = {0, 0, 0};
  float accel_scale_ = 1.0f;

  // ── 滤波输出保持 (NaN 保护) ──
  uint64_t last_ts_ns_ = 0;
  bool     first_sample_ = true;
  float    last_gyro_out_[3] = {0, 0, 0};
  float    last_accel_out_[3] = {0, 0, 0};
};

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER2_BUILTIN_IMU_PROCESSOR_H_
