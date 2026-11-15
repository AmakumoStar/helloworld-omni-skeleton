/**
 *******************************************************************************
 * @file      : ins_pid.cpp
 * @brief     : 云台 yaw / pitch 位置环 PID 实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 两个关节位置环
 *******************************************************************************
 * @attention :
 *  单位约定：输入 = 角度差（rad，已归一化到 [-π, π)），输出 = 转矩（N·m）。
 *  输出限幅留了余量（DM-J4310 最大 7.5 N·m，这里先用 3 N·m），
 *  避免整定初期直接把云台甩出去。整定到稳定后再逐步放开。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_pid.hpp"

#include "pid.hpp"

namespace
{
/** 位置环参数（整定起点：先 P 后 D，最后补少量 I） */
hello_world::pid::Pid::Config MakeYawConfig()
{
  hello_world::pid::Pid::Config cfg;
  cfg.kp = 6.0f;    ///< N·m/rad
  cfg.ki = 0.0f;    ///< 位置环一般不需要积分
  cfg.kd = 0.15f;   ///< 阻尼，抑制过零附近的振荡
  cfg.out_min = -3.0f;
  cfg.out_max = 3.0f;
  cfg.int_min = -1.0f;
  cfg.int_max = 1.0f;
  return cfg;
}

hello_world::pid::Pid::Config MakePitchConfig()
{
  hello_world::pid::Pid::Config cfg;
  cfg.kp = 5.0f;
  cfg.ki = 0.0f;
  cfg.kd = 0.12f;
  cfg.out_min = -2.0f;  ///< pitch 更保守：撞限位会堵转
  cfg.out_max = 2.0f;
  cfg.int_min = -1.0f;
  cfg.int_max = 1.0f;
  return cfg;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/

hello_world::pid::Pid* GetPidMotorYaw()
{
  static hello_world::pid::Pid pid(MakeYawConfig());
  return &pid;
}

hello_world::pid::Pid* GetPidMotorPitch()
{
  static hello_world::pid::Pid pid(MakePitchConfig());
  return &pid;
}

/* Private function definitions ----------------------------------------------*/
