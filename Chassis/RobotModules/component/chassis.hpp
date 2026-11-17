/**
 *******************************************************************************
 * @file      : chassis.hpp
 * @brief     : 底盘运动模块：云台系速度 → 麦轮逆解 → 四轮限幅轮速
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 初版：组合逆解器 + 指令缓存 + 限幅
 *******************************************************************************
 * @attention :
 *  1. 本类只做「运动学」，不含模式判断、不含状态机、不直接操作电机——
 *     模式由 robot::Robot 决定，电机下发由 Robot 的 OutputWheels() 完成。
 *  2. 输入速度指令定义在「云台坐标系」，由 Robot 传入 δ 一起解算。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_ROBOT_MODULES_CHASSIS_HPP_
#define CHASSIS_ROBOT_MODULES_CHASSIS_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "chassis_ik_solver.hpp"
#include "module_state.hpp"

namespace robot
{
/* Exported constants --------------------------------------------------------*/

/** 底盘指令限幅（数值待实机标定，先给保守值）
 *
 * 速度恢复阶梯（每档都先架空确认方向、再落地）：
 *   0.10 → **0.15（当前）** → 0.20 → 0.25 → 0.6 → 验收值 3.0 m/s
 * 0.15 m/s ≈ 轮子 1.9 rad/s ≈ 18 rpm，比人手走路（1.2 m/s）慢 8 倍。
 * 空载开环 5% 就能到 20 rad/s（1.7 m/s），所以这里仍然远低于"会失控"的量级。 */
struct ChassisLimits {
  float max_vx = 0.15f;      ///< 前后最大线速度，m/s（首次落地测试档；验收前改回 3.0）
  float max_vy = 0.15f;      ///< 左右最大线速度，m/s（同上）
  float max_w = 0.80f;       ///< 分离 / 跟随模式最大角速度，rad/s（2026-11-17 由 0.40 提高；验收改回 6.0）
  float max_spin_w = 0.80f;  ///< 小陀螺最大自旋角速度，rad/s（同上，验收改回 8.0）
};

/* Exported types ------------------------------------------------------------*/

class Chassis
{
 public:
  Chassis(void) = default;
  ~Chassis(void) = default;

  /**
   * @brief       初始化几何参数
   * @param       geometry: 轮半径 / 半轴距 / 半轮距 / 单轮速度上限
   * @retval      None
   * @note        数值以实机测量为准（考核说明 6.2），未填时解算结果不可信
   */
  void Init(const hello_world::chassis_ik_solver::ChassisGeometry& geometry)
  {
    ik_solver_.Init(geometry);
    Stop();
  }

  void set_limits(const ChassisLimits& limits) { limits_ = limits; }
  const ChassisLimits& limits(void) const { return limits_; }

  /**
   * @brief       下发本周期速度指令（云台系）+ 当前 δ
   * @param       vel_gimbal: 云台系 (vx, vy, w)，单位 m/s、rad/s
   * @param       delta: δ = 云台朝向 − 车头朝向，rad
   * @retval      None
   */
  void SetCommand(const ChassisVel& vel_gimbal, float delta)
  {
    command_ = vel_gimbal;
    delta_ = delta;
  }

  /**
   * @brief       1ms 周期：云台系 → 车体系 → 逆解 → 等比限幅
   * @param       dt: 周期，单位 s（本类暂不使用，保留接口一致性）
   * @retval      None
   */
  void Update(float dt)
  {
    (void)dt;
    wheel_vel_ = ik_solver_.SolveAndLimit(command_, delta_);
  }

  /** 零指令（急停 / 非 Working 时调用） */
  void Stop(void)
  {
    command_.setZero();
    wheel_vel_.setZero();
  }

  /* 数据获取 */
  const hello_world::chassis_ik_solver::WheelVel& wheel_vel(void) const { return wheel_vel_; }
  const ChassisVel& command(void) const { return command_; }
  float delta(void) const { return delta_; }
  const hello_world::chassis_ik_solver::ChassisIkSolver& ik_solver(void) const
  {
    return ik_solver_;
  }
  hello_world::chassis_ik_solver::ChassisIkSolver& ik_solver(void) { return ik_solver_; }

  /** 车体系下的指令（观测用） */
  ChassisVel chassis_frame_command(void) const
  {
    return GimbalToChassis(command_, delta_);
  }

 private:
  hello_world::chassis_ik_solver::ChassisIkSolver ik_solver_{};  ///< 逆解器
  hello_world::chassis_ik_solver::WheelVel wheel_vel_{};         ///< 限幅后的轮速
  ChassisVel command_{};                                         ///< 云台系指令
  ChassisLimits limits_{};                                       ///< 指令限幅
  float delta_ = 0.0f;                                           ///< 云台相对车头的偏航角
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_CHASSIS_HPP_ */
