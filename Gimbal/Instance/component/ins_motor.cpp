/**
 *******************************************************************************
 * @file      : ins_motor.cpp
 * @brief     : 云台关节电机实例化（yaw / pitch，DM-J4310）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 静态实例 + 接线配置项
 *******************************************************************************
 * @attention :
 *  1. 达妙电机上电处于失能状态，且「不发指令就不反馈」，因此这里把
 *     auto_enable 置 true，让驱动类自动发使能指令并等到反馈使能成功。
 *  2. 方向 / 角度偏移是待填项，标定方法见 motor_base.hpp 的 attention：
 *     先让 dir = kDirFwd，手转关节看 angle() 增减是否与期望一致。
 *  3. yaw 的 angle_range 用 kNegPiToPosPi：DM-J4310 内部不做多圈累计，
 *     连续旋转靠上层用「相对增量 + 角度差归一化」实现，而不是靠电机计圈。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_motor.hpp"

#include "motor.hpp"

namespace
{
using hello_world::motor::DM_J4310;
using hello_world::motor::OptionalParams;

/* ====== 实机接线配置 ======
 * 来源：电路拓扑.png —— 「云台Pitch电机 id:1」「云台Yaw DM4310电机 id:2」。
 * 注意 yaw / pitch 的 ID 与直觉相反：pitch 是 1，yaw 是 2。 */
constexpr uint8_t kYawId = 2u;    /* Yaw  → id 2 */
constexpr uint8_t kPitchId = 1u;  /* Pitch → id 1 */
/* 已补全版考核说明 6.2：yaw「正向 / -1.36920」，pitch「反向 / 0.02480」 */
constexpr float kYawAngleOffset = -1.36920f;
constexpr float kPitchAngleOffset = 0.02480f;

OptionalParams MakeYawParams()
{
  OptionalParams opt;
  opt.input_type = hello_world::motor::InputType::kTorq;
  opt.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi;
  opt.dir = hello_world::motor::kDirFwd; /* 已补全：正向 */
  opt.angle_offset = kYawAngleOffset;
  return opt;
}

OptionalParams MakePitchParams()
{
  OptionalParams opt;
  opt.input_type = hello_world::motor::InputType::kTorq;
  opt.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi;
  opt.dir = hello_world::motor::kDirRev; /* 已补全：反向 */
  opt.angle_offset = kPitchAngleOffset;
  return opt;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/

hello_world::motor::Motor* GetMotorYaw()
{
  /* auto_enable = true：上电后自动使能；MIT 模式由调用方通过 setMitInput 指定 */
  static DM_J4310 yaw(kYawId, MakeYawParams(), true, false);
  return &yaw;
}

hello_world::motor::Motor* GetMotorPitch()
{
  static DM_J4310 pitch(kPitchId, MakePitchParams(), true, false);
  return &pitch;
}

/* Private function definitions ----------------------------------------------*/
