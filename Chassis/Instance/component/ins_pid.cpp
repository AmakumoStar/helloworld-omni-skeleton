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
 *
 * ===== 2026-11-18 架空阶跃实测（拨轮阶跃 → 四轮目标 1.77 rad/s，100 Hz 录制）=====
 *   150 / 150 / ff=700（旧值）: t90 ≈ 40 ms  超调 **106%**  2 s 后还差 36%
 *   150 / 150 / ff=0          : t90 1150 ms  超调  28%
 *   400 /  50 / ff=0          : t90  ~50 ms  超调  **19%**   ← 拐点
 *   500 /  50 / ff=0          : t90  ~16 ms  超调  19%
 *   600 /  50 / ff=0          : t90   9 ms  超调  25%  ← 过头
 *   400 /   0 / ff=0          : t90  ~24 ms  超调  47%  ← 无积分稳态有静差，不可取
 *
 * 第二轮（ff 衰减实现后，测的是"稳态准不准"）：
 *   400 / 150 / ff=700 常数 : 稳态 2.15~2.31（目标 1.77，**+25%**）超调 38.7%
 *   400 / 300 / ff=700 常数 : 稳态 1.95~1.97（+11%）              超调 60.5%
 *   400 / 150 / ff=700 衰减 : 稳态 1.49~1.63（−10%）              超调 37.2%
 *   400 / 300 / ff=700 衰减 : 稳态 **1.62~1.72（−3~5%）**         超调 **24.7%** ← 定稿
 *   （超调数字含 ±0.2 rad/s 的轮速纹波 ≈12%，真实超调约 12~15%）
 * ⚠️ ki=50 时稳态差 26%：维持转速的电流要靠积分补，ki 太小补不上。
 *    但 ki 不能无限加（300→更大时常数 ff 那组超调从 38% 涨到 60%）。 */
volatile float g_wheel_kp = 400.0f;
volatile float g_wheel_ki = 300.0f;
volatile float g_wheel_kd = 0.0f;
/* 静摩擦前馈：2026-11-17 实测起转门槛 ≈ raw 655~819（开环 4% 不动、5% 才转）。
 * 2026-11-18 起它只作用于**低速段**，由 g_wheel_ff_knee 决定衰减速度。 */
volatile float g_wheel_ff = 700.0f;
/* 前馈衰减拐点（轮子角速度 rad/s）：|vel| 达到这个值时 ff 衰减到 0。
 * 为什么要衰减：常数 ff 在轮子动起来后就是**过量推力**（动摩擦 ≪ 静摩擦），
 * 实测造成 106% 超调 + 数秒才回落，即"一开始快然后变慢"。
 * 1.0 rad/s ≈ 0.078 m/s 轮面速度，比正常行驶速度低一个量级，
 * 所以只影响起步那几十毫秒。调小 → 更快撒手（更不冲，但破静摩擦更弱）。 */
volatile float g_wheel_ff_knee = 1.0f;

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
