/**
 *******************************************************************************
 * @file      : chassis_ik_solver.hpp
 * @brief     : 麦克纳姆轮底盘运动学逆解 + 云台系/车体系变换（纯数学）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 初版：4 轮逆解 + 等比限幅
 *******************************************************************************
 * @attention :
 *  1. 本文件是纯数学，不含任何 HAL / 电机 / 时间依赖，可在 PC 上直接单测。
 *  2. 轮序与考核说明 6.2 的表格一致：0=LF(左前) 1=LB(左后) 2=RB(右后) 3=RF(右前)。
 *  3. 麦轮滚子角 45°，即逆解矩阵里 tan45° = 1，四轮的 ±1 系数本身已经把滚子角
 *     包含进去了，不需要再乘一个 tan。
 *  4. 逆解公式（车体系输入，输出轮子线速度 m/s）：
 *        v_LF = vx − vy − k·w        v_LB = vx + vy − k·w
 *        v_RB = vx − vy + k·w        v_RF = vx + vy + k·w
 *     其中 k = 半轴距 + 半轮距。若实车自旋方向相反，把 kSpinSign 取反。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_ROBOT_MODULES_CHASSIS_IK_SOLVER_HPP_
#define CHASSIS_ROBOT_MODULES_CHASSIS_IK_SOLVER_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <cstdint>

#include "module_state.hpp"

namespace hello_world
{
namespace chassis_ik_solver
{
/* Exported constants --------------------------------------------------------*/

/** 轮子数量 */
inline constexpr uint8_t kWheelCount = 4u;
/** 轮子名称，下标与轮序一致 */
inline const char* const kWheelName[kWheelCount] = {"LF", "LB", "RB", "RF"};

/**
 * 自旋项符号。标准麦轮布置为 +1；实车若发现「给正 w 却反向自旋」，改成 -1。
 */
inline constexpr float kSpinSign = 1.0f;

/* Exported types ------------------------------------------------------------*/

/** 底盘几何参数（数值以实机测量为准，见考核说明 6.2） */
struct ChassisGeometry {
  float wheel_radius = 0.0f;     ///< 轮半径 R，单位 m
  float half_wheelbase = 0.0f;   ///< 半轴距（前后轮到中心距离），单位 m
  float half_track = 0.0f;       ///< 半轮距（左右轮到中心距离），单位 m
  float max_wheel_vel = 0.0f;    ///< 单轮线速度上限，单位 m/s（用于解算后限幅）
};

/** 四轮解算结果（轮子线速度，单位 m/s） */
struct WheelVel {
  float v[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};

  void setZero(void)
  {
    for (uint8_t i = 0; i < kWheelCount; ++i) {
      v[i] = 0.0f;
    }
  }
  float maxAbs(void) const
  {
    float m = 0.0f;
    for (uint8_t i = 0; i < kWheelCount; ++i) {
      const float a = std::fabs(v[i]);
      if (a > m) {
        m = a;
      }
    }
    return m;
  }
};

/* Exported function prototypes ----------------------------------------------*/

class ChassisIkSolver
{
 public:
  ChassisIkSolver(void) = default;
  ~ChassisIkSolver(void) = default;

  /** 写入几何参数（上板前用实机测量值替换） */
  void Init(const ChassisGeometry& geometry)
  {
    geometry_ = geometry;
    if (geometry_.max_wheel_vel <= 0.0f) {
      geometry_.max_wheel_vel = 1e9f; /* 未配置限幅 → 不限制 */
    }
  }

  const ChassisGeometry& geometry(void) const { return geometry_; }
  void set_geometry(const ChassisGeometry& geometry) { Init(geometry); }

  /** 等效力臂 k = 半轴距 + 半轮距 */
  float lever(void) const { return geometry_.half_wheelbase + geometry_.half_track; }

  /**
   * @brief       车体系速度 → 四轮线速度（不含限幅）
   * @param       vel: 车体系 (vx, vy, w)
   * @retval      WheelVel
   */
  WheelVel SolveChassisFrame(const robot::ChassisVel& vel) const
  {
    const float k = lever() * kSpinSign;
    const float w_term = k * vel.w;
    WheelVel out;
    out.v[0] = vel.vx - vel.vy - w_term;  ///< LF
    out.v[1] = vel.vx + vel.vy - w_term;  ///< LB
    out.v[2] = vel.vx - vel.vy + w_term;  ///< RB
    out.v[3] = vel.vx + vel.vy + w_term;  ///< RF
    return out;
  }

  /**
   * @brief       云台系速度 → 四轮线速度（先旋转到车体系，再逆解）
   * @param       vel_gimbal: 云台系 (vx, vy, w)
   * @param       delta: δ = 云台朝向 − 车头朝向，rad
   * @retval      WheelVel（未限幅）
   * @note        这是「前进正方向始终是云台朝向」的实现入口
   */
  WheelVel Solve(const robot::ChassisVel& vel_gimbal, float delta) const
  {
    return SolveChassisFrame(robot::GimbalToChassis(vel_gimbal, delta));
  }

  /**
   * @brief       等比限幅
   * @param       in: 未限幅的解算结果
   * @retval      限幅后的结果
   * @note        超速时四轮按同一比例缩放，保持运动方向不变；独立裁剪会让车「跑偏」。
   */
  WheelVel ApplyLimit(const WheelVel& in) const
  {
    const float max_abs = in.maxAbs();
    const float limit = geometry_.max_wheel_vel;
    if (max_abs <= limit || max_abs <= 0.0f) {
      return in;
    }
    const float scale = limit / max_abs;
    WheelVel out;
    for (uint8_t i = 0; i < kWheelCount; ++i) {
      out.v[i] = in.v[i] * scale;
    }
    return out;
  }

  /**
   * @brief       解算 + 限幅（常用组合）
   */
  WheelVel SolveAndLimit(const robot::ChassisVel& vel_gimbal, float delta) const
  {
    return ApplyLimit(Solve(vel_gimbal, delta));
  }

  /**
   * @brief       轮子线速度 → 轮子角速度（rad/s）
   * @param       wheel_vel: 线速度，m/s
   * @retval      角速度，rad/s；轮半径未配置时返回 0
   */
  float WheelToAngular(const WheelVel& wheel_vel, uint8_t idx) const
  {
    if (idx >= kWheelCount || geometry_.wheel_radius <= 0.0f) {
      return 0.0f;
    }
    return wheel_vel.v[idx] / geometry_.wheel_radius;
  }

 private:
  ChassisGeometry geometry_{};
};

/* Exported variables --------------------------------------------------------*/
}  // namespace chassis_ik_solver
}  // namespace hello_world

#endif /* CHASSIS_ROBOT_MODULES_CHASSIS_IK_SOLVER_HPP_ */
