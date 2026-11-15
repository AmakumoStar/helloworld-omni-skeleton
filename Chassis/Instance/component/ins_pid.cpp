/**
 *******************************************************************************
 * @file      : ins_pid.cpp
 * @brief     : 底盘轮速环 PID 实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 4 个轮速环 PID
 *******************************************************************************
 * @attention :
 *  单位约定：输入 = 轮子角速度（rad/s，电机输出端），输出 = 报文原始量（±16384）。
 *  下面的增益只是「能转起来」的起点，必须按考核要求整定到「稳、快、无超调」。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_pid.hpp"

#include "chassis_ik_solver.hpp"
#include "pid.hpp"

namespace
{
constexpr int kWheelCount = static_cast<int>(hello_world::chassis_ik_solver::kWheelCount);

/** 轮速环参数（整定起点） */
hello_world::pid::Pid::Config MakeWheelConfig()
{
  hello_world::pid::Pid::Config cfg;
  cfg.kp = 200.0f;      ///< 输出/（rad/s）
  cfg.ki = 20.0f;       ///< 消除稳态误差
  cfg.kd = 0.0f;        ///< 速度环一般不用 D
  cfg.out_min = -16384.0f;
  cfg.out_max = 16384.0f;
  cfg.int_min = -8000.0f;
  cfg.int_max = 8000.0f;
  return cfg;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/

hello_world::pid::Pid* GetPidMotorWheel(int idx)
{
  static hello_world::pid::Pid pids[kWheelCount];
  static bool inited = false;

  if (!inited) {
    const hello_world::pid::Pid::Config cfg = MakeWheelConfig();
    for (int i = 0; i < kWheelCount; ++i) {
      pids[i].Init(cfg);
    }
    inited = true;
  }

  if ((idx < 0) || (idx >= kWheelCount)) {
    return nullptr;
  }
  return &pids[idx];
}

/* Private function definitions ----------------------------------------------*/
