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

}  // namespace

/* ===== 轮速环在线调参（J-Link 直接写，不用重烧）=====
 * 为什么必须可调：0.1 m/s ≈ 轮子 1.3 rad/s，这个速度下 PID 要给出约 4~6% 的电流才
 * 推得动静摩擦。2026-11-17 开环实测：raw 655（4%）轮子不动、raw 819（5%）才起转。
 * 而 kp=50 时，误差 1.3 rad/s 只能给出 65 raw —— 推满杆轮子也不动。
 * 所以：kp 提高 + 加一份按方向固定的静摩擦前馈 ff，然后边测边调。
 *
 *   g_wheel_kp  输出/(rad/s)      起步 150（原 50 太小）
 *   g_wheel_ki  输出/(rad/s·s)    起步 150（负责补上稳态所需的电流）
 *   g_wheel_kd  不用              0
 *   g_wheel_ff  静摩擦前馈 raw    起步 400（固定加在输出上，压过死区）
 *
 * ⚠️ ff 只在 |目标速度| > 0.05 rad/s 时加，静止时不加，避免中位抖动被放大。
 * ⚠️ 积分限幅 ±1000，且反馈一断就 Reset（IsFeedbackFresh 看门狗），不会累积成"猛冲"。
 */
volatile float g_wheel_kp = 150.0f;
volatile float g_wheel_ki = 150.0f;
volatile float g_wheel_kd = 0.0f;
/* 静摩擦前馈：2026-11-17 实测起转门槛 ≈ raw 655~819（开环 4% 不动、5% 才转）。
 * 原来给 400 不够 —— 表现为"推杆后要等 0.5~1 s，等积分把输出顶上去轮子才动"。
 * 提到 700 后一推杆就有接近门槛的电流，起步几乎无延迟。 */
volatile float g_wheel_ff = 700.0f;

namespace
{
/** 轮速环参数（每周期由 g_wheel_* 覆盖，这里的值只作初值） */
hello_world::pid::Pid::Config MakeWheelConfig()
{
  hello_world::pid::Pid::Config cfg;
  cfg.kp = g_wheel_kp;      ///< 输出/（rad/s）
  cfg.ki = g_wheel_ki;      ///< 输出/（rad/s·s）
  cfg.kd = g_wheel_kd;      ///< 速度环一般不用 D
  cfg.out_min = -3000.0f;
  cfg.out_max = 3000.0f;    ///< 单轮最大输出 ≈ 18% 满量程，低速测试阶段的硬上限
  cfg.int_min = -1000.0f;
  cfg.int_max = 1000.0f;
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
