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

/**
 * @file builtin_imu_processor.cpp
 * @brief ArduPilot 风格的内置 Livox IMU 处理器
 *
 * 参照实现:
 *   - AP_InertialSensor_Backend::apply_gyro_filters()  陷波 → 低通, 低通必须最后
 *   - AP_InertialSensor_Backend::_update_sensor_rate() 采样率 EMA 跟踪
 *   - AP_InertialSensor::_init_gyro()                  0.5s 窗口均值收敛的零偏校准
 *   - AP_InertialSensor::is_still()                    加计标准差静置判定
 *   - Filter/NotchFilter.{h,cpp}                       双二阶陷波 (Leonard Hall 设计)
 *   - Filter/LowPassFilter.{h,cpp}                     alpha = dt/(dt + 1/(2πf))
 */

#include "builtin_imu_processor.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace livox_ros {

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr double kCalibWindowSec = 0.5;         // 仿 _init_gyro: 0.5s 一个均值窗口
constexpr uint32_t kMinWindowSamples = 10;      // 窗口内最少采样数
constexpr uint64_t kNsPerSecond = 1000000000ULL;
constexpr uint64_t kMinDtNs = 250000ULL;        // 4 kHz 上限
constexpr uint64_t kMaxDtNs = 100000000ULL;     // 10 Hz 下限
constexpr float kNotchNyquistRatio = 0.48f;     // 仿 HARMONIC_NYQUIST_CUTOFF
constexpr float kMaxGyroBiasRad = 0.3f;         // ~17°/s, 超过则疑似校准时未静止
constexpr float kAccelScaleMin = 0.8f;
constexpr float kAccelScaleMax = 1.2f;

float Clamp(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}

/** 仿 ArduPilot calc_lowpass_alpha_dt */
float LowpassAlpha(float dt, float cutoff_hz) {
  if (cutoff_hz <= 0.0f) {
    return 1.0f;  // 直通
  }
  return dt / (dt + 1.0f / (2.0f * kPi * cutoff_hz));
}

float VectorMag(const double v[3]) {
  return sqrtf(static_cast<float>(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]));
}

float VectorMagF(const float v[3]) {
  return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}
}  // namespace

// ════════════════════════════════════════════════════════════════════════
// NotchFilter3 —— 移植自 ArduPilot Filter/NotchFilter
// ════════════════════════════════════════════════════════════════════════

void BuiltinImuProcessor::NotchFilter3::Init(float sample_freq,
                                             float center_freq,
                                             float bandwidth,
                                             float atten_dB) {
  initialised = false;
  if ((center_freq <= 0.5f * bandwidth) || (center_freq >= 0.5f * sample_freq)) {
    return;  // 仿 NotchFilter::init 的范围检查
  }

  // 仿 calculate_A_and_Q
  A = powf(10.0f, -atten_dB / 40.0f);
  const float octaves = log2f(center_freq / (center_freq - bandwidth / 2.0f)) * 2.0f;
  const float Q = sqrtf(powf(2.0f, octaves)) / (powf(2.0f, octaves) - 1.0f);

  // 仿 init_with_A_and_Q
  if (Q <= 0.0f || !std::isfinite(Q)) {
    return;
  }
  const float omega = 2.0f * kPi * center_freq / sample_freq;
  const float alpha = sinf(omega) / (2.0f * Q);
  b0 = 1.0f + alpha * A * A;
  b1 = -2.0f * cosf(omega);
  b2 = 1.0f - alpha * A * A;
  a1 = b1;
  a2 = 1.0f - alpha;

  const float a0_inv = 1.0f / (1.0f + alpha);
  b0 *= a0_inv;
  b1 *= a0_inv;
  b2 *= a0_inv;
  a1 *= a0_inv;
  a2 *= a0_inv;

  fc = center_freq;
  fs = sample_freq;
  initialised = true;
}

void BuiltinImuProcessor::NotchFilter3::Reset() {
  need_reset = true;
}

void BuiltinImuProcessor::NotchFilter3::Apply(float v[3]) {
  if (!initialised || need_reset) {
    // 未初始化时直通并更新延迟样本 (仿 NotchFilter::apply)
    for (int i = 0; i < 3; ++i) {
      sig1[i] = sig2[i] = ntch1[i] = ntch2[i] = v[i];
    }
    need_reset = false;
    return;
  }

  for (int i = 0; i < 3; ++i) {
    const float out = v[i] * b0 + ntch1[i] * b1 + ntch2[i] * b2 -
                      sig1[i] * a1 - sig2[i] * a2;
    ntch2[i] = ntch1[i];
    ntch1[i] = v[i];
    sig2[i] = sig1[i];
    sig1[i] = out;
    v[i] = out;
  }
}

// ════════════════════════════════════════════════════════════════════════
// Lpf3 —— 移植自 ArduPilot Filter/LowPassFilter (变 dt 版)
// ════════════════════════════════════════════════════════════════════════

void BuiltinImuProcessor::Lpf3::Reset() {
  for (int i = 0; i < 3; ++i) {
    init[i] = false;
  }
}

void BuiltinImuProcessor::Lpf3::Apply(const float in[3], float dt,
                                      float cutoff_hz, float out[3]) {
  const float alpha = LowpassAlpha(dt, cutoff_hz);
  for (int i = 0; i < 3; ++i) {
    if (!init[i]) {
      // 首样本直接初始化输出 (仿 DigitalLPF::_apply)
      val[i] = in[i];
      init[i] = true;
      out[i] = in[i];
    } else {
      val[i] += (in[i] - val[i]) * alpha;
      out[i] = val[i];
    }
  }
}

// ════════════════════════════════════════════════════════════════════════
// BuiltinImuProcessor
// ════════════════════════════════════════════════════════════════════════

BuiltinImuProcessor::BuiltinImuProcessor(const BuiltinImuConfig &cfg) : cfg_(cfg) {
  sample_rate_hz_ = static_cast<float>(cfg_.nominal_rate_hz);
  calibrating_ = cfg_.calib_enable;
}

void BuiltinImuProcessor::InitNotches(float sample_rate) {
  notches_.clear();
  if (!cfg_.notch_enable || sample_rate <= 0.0f || cfg_.notch_freq <= 0.0f) {
    notch_sample_rate_ = 0;
    return;
  }

  const float nyquist = sample_rate * kNotchNyquistRatio;
  int harmonics = std::max(1, cfg_.notch_harmonics);
  for (int k = 1; k <= 16 && k <= harmonics; ++k) {
    if (!(cfg_.notch_harmonics & (1 << (k - 1)))) {
      continue;  // 位掩码未启用该次谐波
    }
    const float fc = static_cast<float>(cfg_.notch_freq) * k;
    if (fc >= nyquist) {
      printf("[BuiltinIMU] 陷波: 跳过 %d 次谐波 %.0f Hz (≥ 0.48*fs=%.0f Hz)\n",
             k, fc, nyquist);
      continue;  // 仿 HARMONIC_NYQUIST_CUTOFF
    }
    NotchFilter3 nf;
    nf.Init(sample_rate, fc, static_cast<float>(cfg_.notch_bw),
            static_cast<float>(cfg_.notch_att));
    if (nf.initialised) {
      notches_.push_back(nf);
    }
  }
  notch_sample_rate_ = sample_rate;
}

void BuiltinImuProcessor::UpdateSampleRate(uint64_t ts_ns) {
  // 仿 AP_InertialSensor_Backend::_update_sensor_rate
  if (rate_start_ts_ns_ == 0) {
    rate_count_ = 0;
    rate_start_ts_ns_ = ts_ns;
    return;
  }
  rate_count_++;
  const uint64_t elapsed = (ts_ns > rate_start_ts_ns_) ? (ts_ns - rate_start_ts_ns_) : 0;
  if (elapsed >= kNsPerSecond) {
    float observed = (elapsed > 0) ? rate_count_ * 1.0e9f / elapsed : sample_rate_hz_;

    // 收敛期 (校准未完成) 快速跟踪 ±100%, 之后 ±5% 缓慢修正
    float filter_const = 0.98f;
    float upper = 1.05f;
    float lower = 0.95f;
    if (calibrating_) {
      filter_const = 0.8f;
      upper = 2.0f;
      lower = 0.5f;
    }
    observed = Clamp(observed, sample_rate_hz_ * lower, sample_rate_hz_ * upper);
    sample_rate_hz_ = filter_const * sample_rate_hz_ + (1.0f - filter_const) * observed;

    // 采样率变化 >5% 时重建陷波 (仿收敛期反复 re-init 陷波的策略)
    if (notch_sample_rate_ > 0.0f &&
        fabsf(sample_rate_hz_ - notch_sample_rate_) > 0.05f * notch_sample_rate_) {
      InitNotches(sample_rate_hz_);
    }

    rate_count_ = 0;
    rate_start_ts_ns_ = ts_ns;
  }
}

float BuiltinImuProcessor::ComputeDt(uint64_t ts_ns) {
  float dt = 1.0f / sample_rate_hz_;
  if (last_ts_ns_ != 0 && ts_ns > last_ts_ns_) {
    const uint64_t d = ts_ns - last_ts_ns_;
    if (d >= kMinDtNs && d <= kMaxDtNs) {
      dt = d * 1.0e-9f;
    }
  }
  last_ts_ns_ = ts_ns;
  return dt;
}

void BuiltinImuProcessor::CalibrationCollect(uint64_t ts_ns,
                                             const float gyro[3],
                                             const float accel[3]) {
  if (calib_start_ts_ns_ == 0) {
    calib_start_ts_ns_ = ts_ns;
  }
  if (win_start_ts_ns_ == 0) {
    win_start_ts_ns_ = ts_ns;
  }

  for (int i = 0; i < 3; ++i) {
    win_gyro_sum_[i] += gyro[i];
    win_accel_sum_[i] += accel[i];
    win_accel_sq_[i] += static_cast<double>(accel[i]) * accel[i];
  }
  win_count_++;

  // 窗口满 (0.5s 且样本足够) → 结算
  const uint64_t win_elapsed =
      (ts_ns > win_start_ts_ns_) ? (ts_ns - win_start_ts_ns_) : 0;
  if (win_elapsed >= static_cast<uint64_t>(kCalibWindowSec * kNsPerSecond) &&
      win_count_ >= kMinWindowSamples) {
    FinalizeCalibWindow(ts_ns);
  }

  // 超时兜底
  const uint64_t total_elapsed =
      (ts_ns > calib_start_ts_ns_) ? (ts_ns - calib_start_ts_ns_) : 0;
  if (total_elapsed >= static_cast<uint64_t>(cfg_.calib_duration * kNsPerSecond)) {
    FinishCalibration(true);
  }
}

void BuiltinImuProcessor::FinalizeCalibWindow(uint64_t ts_ns) {
  if (win_count_ == 0) {
    return;
  }

  const double n = static_cast<double>(win_count_);
  float gyro_mean[3];
  float accel_mean[3];
  float accel_std[3];
  for (int i = 0; i < 3; ++i) {
    gyro_mean[i] = static_cast<float>(win_gyro_sum_[i] / n);
    accel_mean[i] = static_cast<float>(win_accel_sum_[i] / n);
    const double var = win_accel_sq_[i] / n -
                       static_cast<double>(accel_mean[i]) * accel_mean[i];
    accel_std[i] = sqrtf(static_cast<float>(std::max(var, 0.0)));
  }

  // 仿 AP_InertialSensor::is_still(): 三轴加计标准差均低于阈值
  const float thresh = static_cast<float>(cfg_.still_threshold);
  const bool still = (accel_std[0] < thresh) && (accel_std[1] < thresh) &&
                     (accel_std[2] < thresh);

  if (still) {
    for (int i = 0; i < 3; ++i) {
      cand_gyro_mean_[i] = gyro_mean[i];
      cand_accel_mean_[i] = accel_mean[i];
    }
    cand_valid_ = true;

    if (prev_win_valid_) {
      // 仿 _init_gyro: 相邻两个 0.5s 窗口均值差 < 阈值即收敛
      float max_diff = 0.0f;
      for (int i = 0; i < 3; ++i) {
        max_diff = std::max(max_diff,
            fabsf(gyro_mean[i] - static_cast<float>(prev_win_gyro_mean_[i])));
      }
      const float diff_limit =
          static_cast<float>(cfg_.gyro_cal_diff) * kPi / 180.0f;  // deg/s → rad/s
      if (max_diff < diff_limit) {
        FinishCalibration(false);
        return;
      }
    }
    for (int i = 0; i < 3; ++i) {
      prev_win_gyro_mean_[i] = gyro_mean[i];
    }
    prev_win_valid_ = true;
  } else {
    prev_win_valid_ = false;  // 窗口内有移动, 收敛链断裂
  }

  // 清空窗口统计
  for (int i = 0; i < 3; ++i) {
    win_gyro_sum_[i] = 0.0;
    win_accel_sum_[i] = 0.0;
    win_accel_sq_[i] = 0.0;
  }
  win_count_ = 0;
  win_start_ts_ns_ = ts_ns;
}

bool BuiltinImuProcessor::FinishCalibration(bool timeout) {
  if (calibrated_) {
    return true;
  }

  if (cand_valid_) {
    for (int i = 0; i < 3; ++i) {
      gyro_bias_[i] = static_cast<float>(cand_gyro_mean_[i]);
    }
    // 仿 INS_ACC_BODYFIX: 静态加速度幅值归一化到 g (无姿态假设的幅值修正)
    const float mag = VectorMag(cand_accel_mean_);
    if (mag > 0.5f * kGravityMss && mag < 2.0f * kGravityMss) {
      accel_scale_ = Clamp(kGravityMss / mag, kAccelScaleMin, kAccelScaleMax);
    } else {
      printf("[BuiltinIMU] 校准: 静态加速度幅值 %.3f m/s² 异常, 跳过幅值修正\n", mag);
    }

    const float bias_mag = VectorMagF(gyro_bias_);
    printf("[BuiltinIMU] 校准完成%s: gyro_bias=(%+.5f, %+.5f, %+.5f) rad/s (|bias|=%.5f), "
           "accel_scale=%.5f (静态幅值 %.3f → %.3f m/s²)\n",
           timeout ? "(超时, 取最近静置窗口)" : "",
           gyro_bias_[0], gyro_bias_[1], gyro_bias_[2], bias_mag,
           accel_scale_, mag, mag * accel_scale_);
    if (bias_mag > kMaxGyroBiasRad) {
      printf("[BuiltinIMU] 警告: 陀螺零偏过大 (%.1f °/s), 校准时可能未完全静止\n",
             bias_mag * 180.0f / kPi);
    }
  } else {
    printf("[BuiltinIMU] 校准失败: %.1f s 内未检测到静置 (加计三轴标准差 < %.1f m/s²), "
           "保持零偏置直通\n", cfg_.calib_duration, cfg_.still_threshold);
  }

  // 校正量阶跃引入滤波暂态 → 复位滤波器 (首样本直接输出校正值)
  gyro_lpf_.Reset();
  accel_lpf_.Reset();
  for (auto &nf : notches_) {
    nf.Reset();
  }

  calibrated_ = true;
  calibrating_ = false;
  return true;
}

bool BuiltinImuProcessor::OutputsValid(const float gyro[3],
                                       const float accel[3]) const {
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(gyro[i]) || !std::isfinite(accel[i])) {
      return false;
    }
  }
  return true;
}

bool BuiltinImuProcessor::Process(uint64_t ts_ns,
                                  const float gyro_raw[3],
                                  const float accel_raw[3],
                                  float gyro_out[3],
                                  float accel_out[3]) {
  if (first_sample_) {
    first_sample_ = false;
    sample_rate_hz_ = static_cast<float>(cfg_.nominal_rate_hz);
    InitNotches(sample_rate_hz_);
    printf("[BuiltinIMU] ArduPilot 风格处理链启用: "
           "notch=%s(f0=%.0f Hz, bw=%.0f, att=%.0f dB, hmncs=%d) "
           "gyro_lpf=%.0f Hz accel_lpf=%.0f Hz calib=%s(%.1f s, still=%.1f m/s²)\n",
           cfg_.notch_enable ? "ON" : "OFF",
           cfg_.notch_freq, cfg_.notch_bw, cfg_.notch_att, cfg_.notch_harmonics,
           cfg_.gyro_lpf_hz, cfg_.accel_lpf_hz,
           cfg_.calib_enable ? "ON" : "OFF",
           cfg_.calib_duration, cfg_.still_threshold);
  }

  UpdateSampleRate(ts_ns);

  // ── 单位换算: g → m/s² (仿 ArduPilot GRAVITY_MSS) ──
  float accel[3];
  for (int i = 0; i < 3; ++i) {
    accel[i] = accel_raw[i] * kGravityMss;
  }

  // ── 启动静置校准 (仿 _init_gyro; 校准期间不施加偏置) ──
  if (calibrating_) {
    CalibrationCollect(ts_ns, gyro_raw, accel);
  }

  // ── 校正: 先减偏置/幅值修正, 再滤波 (仿 _rotate_and_correct_*) ──
  float gyro[3];
  for (int i = 0; i < 3; ++i) {
    gyro[i] = gyro_raw[i] - gyro_bias_[i];
    accel[i] *= accel_scale_;
  }

  // ── 陀螺: 谐波陷波 → 低通 (低通必须最后, 压掉陷波引入的噪声) ──
  for (auto &nf : notches_) {
    nf.Apply(gyro);
  }

  const float dt = ComputeDt(ts_ns);
  float gyro_f[3];
  float accel_f[3];
  gyro_lpf_.Apply(gyro, dt, static_cast<float>(cfg_.gyro_lpf_hz), gyro_f);
  accel_lpf_.Apply(accel, dt, static_cast<float>(cfg_.accel_lpf_hz), accel_f);

  // ── NaN/Inf 保护: 复位滤波器并保持上一有效输出 (仿 apply_gyro_filters) ──
  if (!OutputsValid(gyro_f, accel_f)) {
    gyro_lpf_.Reset();
    accel_lpf_.Reset();
    for (auto &nf : notches_) {
      nf.Reset();
    }
    for (int i = 0; i < 3; ++i) {
      gyro_out[i] = last_gyro_out_[i];
      accel_out[i] = last_accel_out_[i];
    }
    return true;
  }

  for (int i = 0; i < 3; ++i) {
    gyro_out[i] = gyro_f[i];
    accel_out[i] = accel_f[i];
    last_gyro_out_[i] = gyro_f[i];
    last_accel_out_[i] = accel_f[i];
  }
  return true;
}

}  // namespace livox_ros
