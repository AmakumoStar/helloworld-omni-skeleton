/**
 *******************************************************************************
 * @file      : imu_bias.hpp
 * @brief     : 陀螺仪零漂估计器（纯算法，可脱离硬件单测）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 初版：滑动窗均值 + 标准差收敛判定
 *******************************************************************************
 * @attention :
 *  1. 本文件只做「开机静止采样 → 求角速度零漂」这一件事，不碰 SPI / BMI088 / Mahony，
 *     因此可以在 PC 上喂假数据验证收敛逻辑。
 *  2. 判定标准：窗口内每轴样本标准差 <= tol，认为静止可信，零漂可用。
 *     窗口结束仍不达标则丢弃本轮重来，最多重试 max_retry 次。
 *  3. 零漂未收敛（converged() == false）时，状态机不得进入 Working。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_IMU_IMU_BIAS_HPP_
#define HW_COMPONENTS_DEVICES_IMU_IMU_BIAS_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <cstdint>

namespace hello_world
{
namespace imu
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class GyroBiasEstimator
{
 public:
  GyroBiasEstimator(void) = default;
  ~GyroBiasEstimator(void) = default;

  /**
   * @brief       初始化
   * @param       sample_freq_hz: 采样频率，单位 Hz
   * @param       window_s:       采样窗口时长，单位 s（典型 1~3 s）
   * @param       tol_rad_s:      收敛阈值（窗口内标准差上限），单位 rad/s
   * @param       max_retry:      窗口不达标时的最大重试次数
   * @retval      None
   */
  void Init(float sample_freq_hz, float window_s, float tol_rad_s, uint8_t max_retry = 3)
  {
    sample_freq_hz_ = (sample_freq_hz > 1.0f) ? sample_freq_hz : 1000.0f;
    tol_ = tol_rad_s;
    max_retry_ = max_retry;
    need_ = static_cast<uint32_t>(sample_freq_hz_ * window_s);
    if (need_ == 0u) {
      need_ = 1u;
    }
    Reset();
  }

  /** 清空采样，重新开始（重新上电 / 人为重标定） */
  void Reset(void)
  {
    for (uint8_t i = 0; i < 3u; ++i) {
      sum_[i] = 0.0;
      sumsq_[i] = 0.0;
      bias_[i] = 0.0f;
    }
    count_ = 0u;
    retry_ = 0u;
    converged_ = false;
    failed_ = false;
  }

  /**
   * @brief       送入一帧陀螺原始数据（未扣零漂）
   * @param       gyro: [wx wy wz]，单位 rad/s
   * @retval      是否已收敛
   * @note        未收敛前应持续调用；收敛后调用无副作用（直接返回 true）
   */
  bool Update(const float gyro[3])
  {
    if (converged_) {
      return converged_;
    }
    for (uint8_t i = 0; i < 3u; ++i) {
      sum_[i] += static_cast<double>(gyro[i]);
      sumsq_[i] += static_cast<double>(gyro[i]) * static_cast<double>(gyro[i]);
    }
    ++count_;

    if (count_ < need_) {
      return false;
    }

    /* 窗口结束：算均值与标准差 */
    const double inv = 1.0 / static_cast<double>(count_);
    bool ok = true;
    for (uint8_t i = 0; i < 3u; ++i) {
      const double mean = sum_[i] * inv;
      double var = sumsq_[i] * inv - mean * mean;
      if (var < 0.0) {
        var = 0.0;  /* 浮点误差保护 */
      }
      if (std::sqrt(var) > static_cast<double>(tol_)) {
        ok = false;
      }
      bias_[i] = static_cast<float>(mean);
    }

    if (ok) {
      converged_ = true;
      failed_ = false;
      return true;
    }

    /* 不达标：丢弃本轮重来。
     * 注意这里【无限重试】：上电时车可能在动，如果 failed_ 一旦置位就永久放弃，
     * 状态机会永远进不了 Working（这是现场踩过的坑）。
     * 重试次数超限只把 failed_ 置位供调试观察，然后重新计数继续采样。 */
    ++retry_;
    if (retry_ > max_retry_) {
      failed_ = true;
      retry_ = 0u;
    }
    for (uint8_t i = 0; i < 3u; ++i) {
      sum_[i] = 0.0;
      sumsq_[i] = 0.0;
    }
    count_ = 0u;
    return false;
  }

  /* 数据获取 */
  bool converged(void) const { return converged_; }
  /** 重试次数超限仍不收敛（说明上电时车在动，或 IMU 异常） */
  bool failed(void) const { return failed_; }
  /** 进度 0~1，可用于开机自检指示 */
  float progress(void) const
  {
    if (need_ == 0u) {
      return 1.0f;
    }
    return static_cast<float>(count_) / static_cast<float>(need_);
  }
  float bias(uint8_t axis) const { return (axis < 3u) ? bias_[axis] : 0.0f; }

 private:
  float sample_freq_hz_ = 1000.0f;
  float tol_ = 0.02f;
  uint32_t need_ = 1000u;
  uint32_t count_ = 0u;
  uint8_t max_retry_ = 3u;
  uint8_t retry_ = 0u;
  bool converged_ = false;
  bool failed_ = false;
  double sum_[3] = {0.0, 0.0, 0.0};
  double sumsq_[3] = {0.0, 0.0, 0.0};
  float bias_[3] = {0.0f, 0.0f, 0.0f};
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace imu
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_IMU_IMU_BIAS_HPP_ */
