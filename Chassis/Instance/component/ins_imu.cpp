/**
 *******************************************************************************
 * @file      : ins_imu.cpp
 * @brief     : 底盘 IMU 实例化（BMI088 + Mahony）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 静态实例 + SPI / 片选配置
 *******************************************************************************
 * @attention :
 *  ⚠️ 下面两个片选引脚是**按 SPI1 + 上电默认电平推断**的，必须对照原理图核对：
 *     - 加速度计 CS：PA4（SPI1_NSS）
 *     - 陀螺仪   CS：PB0
 *  引脚填错的表现是 imuInit() 返回「未找到设备」，不会损坏硬件。
 *  旋转矩阵先按单位阵给：芯片坐标与机体坐标（x 前 y 左 z 上）一致时无需改，
 *  若芯片贴装方向不同，改 kRotMat 后重新上电即可。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_imu.hpp"

#include "BMI088.hpp"
#include "imu.hpp"
#include "main.h"
#include "spi.h"

namespace
{
/* ====== 硬件配置：已确认 ======
 * 来源：C_project/Tasks/imu_task.cpp 的 ImuInit()（同款 STM32F407 板）：
 *   .hspi = &hspi1,  acc_cs = PA4 (GPIOA/GPIO_PIN_4),  gyro_cs = PB0 (GPIOB/GPIO_PIN_0)
 *   旋转矩阵取单位阵（芯片坐标与机体系 x 前 / y 左 / z 上 同向）。 */
/* 注意：GPIOA/GPIOB 是 reinterpret_cast 出来的地址，不能用于 constexpr，
 * 所以这里是「const 指针」而不是 constexpr 指针。 */
GPIO_TypeDef* const kAccCsPort = GPIOA;
constexpr uint32_t kAccCsPin = GPIO_PIN_4;
GPIO_TypeDef* const kGyroCsPort = GPIOB;
constexpr uint32_t kGyroCsPin = GPIO_PIN_0;

/* 芯片 → 机体系 的旋转矩阵（展平），单位阵 = 同向 */
const float kRotMat[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

}  // namespace

/* Exported function definitions ---------------------------------------------*/

hello_world::imu::Imu* GetImu()
{
  static hello_world::imu::BMI088 chip;
  static hello_world::imu::Imu imu;
  static bool inited = false;

  if (!inited) {
    hello_world::imu::BMI088HWConfig hw_config;
    hw_config.hspi = &hspi1;
    hw_config.acc_cs_port = kAccCsPort;
    hw_config.acc_cs_pin = kAccCsPin;
    hw_config.gyro_cs_port = kGyroCsPort;
    hw_config.gyro_cs_pin = kGyroCsPin;

    chip.init(hw_config, kRotMat);
    /* 芯片配置会阻塞一小段时间，只允许在 MainTaskInit 里调用一次 */
    (void)chip.imuInit(false);

    imu.Init(&chip);
    inited = true;
  }

  return &imu;
}

/* Private function definitions ----------------------------------------------*/
