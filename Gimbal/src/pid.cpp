/**
 *******************************************************************************
 * @file      : pid.cpp
 * @brief     : PID 控制器实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 位置式 + 微分先行 + 条件积分抗饱和
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "pid.hpp"

namespace hello_world
{
namespace pid
{
/* Private function definitions ----------------------------------------------*/

/** 局部限幅（不引入对 robot::Clamp 的依赖，保持本模块可独立编译） */
static inline float ClampLocal(float value, float lower, float upper)
{
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

/* Exported function definitions ---------------------------------------------*/

void Pid::Init(const Config& config)
{
  config_ = config;
  Reset();
}

void Pid::Reset(void)
{
  integral_ = 0.0f;
  prev_meas_ = 0.0f;
  prev_error_ = 0.0f;
  out_ = 0.0f;
  first_ = true;
  saturated_ = false;
}

float Pid::Step(float error, float d_term, float dt)
{
  if (dt <= 0.0f) {
    return out_;
  }

  const float p_term = config_.kp * error;

  /* 试探性积分 */
  float integral_try = integral_ + config_.ki * error * dt;
  integral_try = ClampLocal(integral_try, config_.int_min, config_.int_max);

  float raw = p_term + integral_try + d_term;
  float out = ClampLocal(raw, config_.out_min, config_.out_max);

  /* 条件积分：输出已顶到边界、且误差仍在把输出往同一方向推时，撤销本周期积分，
   * 避免积分饱和导致反向恢复缓慢（积分饱和是「超调」的主要来源之一）。 */
  const bool pushing_up = (raw > config_.out_max) && (error > 0.0f);
  const bool pushing_down = (raw < config_.out_min) && (error < 0.0f);
  if (pushing_up || pushing_down) {
    integral_try = integral_;
    raw = p_term + integral_try + d_term;
    out = ClampLocal(raw, config_.out_min, config_.out_max);
  }

  integral_ = integral_try;
  saturated_ = (raw != out);
  out_ = out;
  return out_;
}

float Pid::Update(float target, float actual, float dt)
{
  if (dt <= 0.0f) {
    return out_;
  }

  const float error = target - actual;

  /* 微分先行：对测量值求微分，目标阶跃时不产生冲击 */
  float d_term = 0.0f;
  if (!first_) {
    d_term = -config_.kd * (actual - prev_meas_) / dt;
  }

  prev_meas_ = actual;
  prev_error_ = error;
  first_ = false;

  return Step(error, d_term, dt);
}

float Pid::UpdateErr(float error, float dt)
{
  if (dt <= 0.0f) {
    return out_;
  }

  float d_term = 0.0f;
  if (!first_) {
    d_term = config_.kd * (error - prev_error_) / dt;
  }

  prev_error_ = error;
  first_ = false;

  return Step(error, d_term, dt);
}

float Pid::UpdateWithRate(float error, float rate, float dt)
{
  if (dt <= 0.0f) {
    return out_;
  }

  /* D 项直接由外部给的（已滤波）速率构成，**不对误差做差分**，避免把噪声放大。
   * 符号：速率正 ⇒ 输出负（反向阻尼）。 */
  prev_error_ = error;
  first_ = false;

  return Step(error, -config_.kd * rate, dt);
}

/* Private function definitions ----------------------------------------------*/
}  // namespace pid
}  // namespace hello_world
