/**
 *******************************************************************************
 * @file      : imu.hpp
 * @brief     : IMU 封装类：BMI088 原始读取 + Mahony 姿态解算 + 开机零漂
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 组合 BMI088 + Mahony，补零漂与姿态角输出
 *******************************************************************************
 * @attention :
 *  1. 姿态角为「相对开机姿态」的欧拉角（Z-Y-X），单位 rad，
 *     坐标系 x 朝前、y 朝左、z 朝上（见 ahrs_base.hpp）。
 *  2. 零漂未收敛时 isCalibrated() 为 false，状态机必须停在 Dead / Resurrection。
 *  3. Update() 只做「读一帧 + 推进一次解算」，不含任何延时，可在 1ms 中断里调用；
 *     Mahony 的采样频率必须与调用频率一致（默认 1000 Hz）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
#define HW_COMPONENTS_DEVICES_IMU_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include "BMI088.hpp"
#include "imu_bias.hpp"
#include "mahony.hpp"

namespace hello_world
{
namespace imu
{
/* Exported constants --------------------------------------------------------*/

/** 姿态解算采样频率，必须与调用 Update() 的频率一致 */
inline constexpr float kImuSampleFreqHz = 1000.0f;
/** Mahony 比例增益（与 C_project 样例一致：kp = 0.5, ki = 0） */
inline constexpr float kMahonyKp = 0.5f;
/** Mahony 积分增益 */
inline constexpr float kMahonyKi = 0.0f;
/** 零漂采样窗口，单位 s */
inline constexpr float kBiasWindowS = 1.5f;
/** 零漂收敛阈值（窗口内标准差上限），单位 rad/s */
inline constexpr float kBiasTolRadS = 0.05f;
/** yaw 角速度的低通系数（1 kHz 下 α=0.1 约等于 10 ms 时间常数）。
 *  只给「小陀螺坐标系超前补偿」用：压掉振动噪声，又不至于让补偿量抖动。 */
inline constexpr float kGyroZFilterAlpha = 0.1f;

/* Exported types ------------------------------------------------------------*/

class Imu
{
 public:
  Imu(void) : ahrs_(kImuSampleFreqHz, kMahonyKp, kMahonyKi)
  {
    bias_.Init(kImuSampleFreqHz, kBiasWindowS, kBiasTolRadS);
  }
  ~Imu(void) = default;

  /**
   * @brief       绑定芯片对象（不负责 SPI / 片选配置，由 Instance 层负责）
   * @param       bmi088: 已完成 init() 与 imuInit() 的 BMI088 对象指针
   * @retval      None
   * @note        调用后零漂重新开始采样
   */
  void Init(BMI088* bmi088)
  {
    bmi088_ = bmi088;
    bias_.Reset();
    for (uint8_t i = 0; i < 3u; ++i) {
      euler_[i] = 0.0f;
    }
    gyro_z_ = 0.0f;
  }

  /**
   * @brief       推进一次：读芯片 → 扣零漂 → 姿态解算 → 更新欧拉角
   * @retval      None
   * @note        零漂未收敛前也在推进解算，只是不对外宣称「已标定」
   */
  void Update(void)
  {
    if (bmi088_ == nullptr) {
      return;
    }

    float acc[3] = {0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    bmi088_->getData(acc, gyro, nullptr);

    if (!bias_.converged()) {
      /* 开机静止段：只采样零漂，不把原始数据喂给解算，避免把抖动积分进去 */
      bias_.Update(gyro);
    }

    float gyro_corrected[3] = {gyro[0], gyro[1], gyro[2]};
    if (bias_.converged()) {
      for (uint8_t i = 0; i < 3u; ++i) {
        gyro_corrected[i] = gyro[i] - bias_.bias(i);
      }
    }

    /* 小陀螺坐标系补偿用的 yaw 角速度：与 euler_[2] 同一符号约定（逆时针为正）。
     * 一阶低通只是压噪声，不参与姿态解算（解算用的是上面那组原始修正值）。 */
    gyro_z_ += (gyro_corrected[2] - gyro_z_) * kGyroZFilterAlpha;

    ahrs_.update(acc, gyro_corrected);
    ahrs_.getEulerAngle(euler_);
  }

  /* 数据获取 */

  /** 零漂是否已收敛（未收敛不得进 Working） */
  bool isCalibrated(void) const { return bias_.converged(); }
  /** 零漂采样进度 0~1，可用于开机自检指示 */
  float calibrationProgress(void) const { return bias_.progress(); }

  float roll(void) const { return euler_[0]; }
  float pitch(void) const { return euler_[1]; }
  float yaw(void) const { return euler_[2]; }
  /**
   * @brief       yaw 角速度（已扣零漂 + 10 ms 低通），rad/s，逆时针为正
   * @note        用于小陀螺模式的坐标系超前补偿（见 Robot::kFrameDelta）。
   *              不要拿它替代 euler_[2] 做姿态，它只是给补偿用的平滑值。
   */
  float yawRate(void) const { return gyro_z_; }

  /** 一次取回 [roll pitch yaw]，单位 rad */
  void getEulerAngle(float euler[3]) const
  {
    if (euler == nullptr) {
      return;
    }
    for (uint8_t i = 0; i < 3u; ++i) {
      euler[i] = euler_[i];
    }
  }

  /** 零漂值访问（标定完成后有效），单位 rad/s */
  float bias(uint8_t axis) const { return bias_.bias(axis); }

 private:
  BMI088* bmi088_ = nullptr;  ///< 芯片对象（不拥有其生命周期）
  ahrs::Mahony ahrs_;         ///< Mahony 姿态解算
  GyroBiasEstimator bias_;    ///< 开机零漂估计
  float euler_[3] = {0.0f, 0.0f, 0.0f};  ///< [roll pitch yaw]，rad
  float gyro_z_ = 0.0f;       ///< yaw 角速度（低通后），rad/s，给小陀螺补偿用
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace imu
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_IMU_IMU_HPP_ */
