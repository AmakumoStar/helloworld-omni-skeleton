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
 *   0.10 → 0.15 → **0.20（当前）** → 0.25 → 0.6 → 验收值 3.0 m/s
 * 0.20 m/s ≈ 轮子 2.6 rad/s ≈ 24 rpm，仍远低于人手走路（1.2 m/s）。
 * 空载开环 5% 就能到 20 rad/s（1.7 m/s），所以这里还是"会失控"量级的 1/8。
 *
 * ⚠️ 改这四个数必须同时看 Chassis/Task/component/main_task.cpp 的 kMaxWheelVelMps：
 *    闸门必须 ≥ max_vx + max_vy + (半轴距+半轮距)·max_spin_w，
 *    否则自旋+平移会被**等比削减**（表现：一推杆平移和自旋同时变慢）。
 *    那里有 static_assert 卡着，忘了改会直接编译不过。 */
struct ChassisLimits {
  float max_vx = 0.20f;      ///< 前后最大线速度，m/s（2026-11-17 由 0.15 提高；验收改回 3.0）
  float max_vy = 0.20f;      ///< 左右最大线速度，m/s（同上）
  /* ⚠️ 想"跟随再快一点"应该调这个，**不要调 g_follow_kp**：
   *   kp 改变的是环路阻尼（实测 2.0 就进极限环、车轮明显震荡）；
   *   max_w 只截断大角度段的转速，环路动态不变，所以能加速而不失稳。
   * 2026-11-17 由 1.00 提到 1.50（90° 转身从 ~1.6 s 缩到 ~1.1 s）。 */
  float max_w = 1.50f;       ///< 分离 / 跟随模式最大角速度，rad/s（验收改回 6.0）
  /* 2026-11-17 由 1.00 提到 1.50（用户要求"自转再快一点"）。
   * ⚠️ 改它会连带闸门：单轮自旋项 = 0.3068×max_spin_w，再叠加对角平移 0.4，
   *    所以 main_task.cpp 的 kMaxWheelVelMps 必须同步（有 static_assert 卡着）。 */
  float max_spin_w = 1.50f;  ///< 小陀螺最大自旋角速度，rad/s（验收改回 8.0）
};

/**
 * 小陀螺模式的**平移倍率上限**（默认值也取它）。
 *
 * 为什么要单独放大 spin 的平移：自旋时每轮都有一部分轮速被"自旋项"占掉
 * （0.3068×max_spin_w ≈ 0.46 m/s），留给平移的余量本来就少；操作手反馈
 * "小陀螺档平移明显不够快"。分离/跟随不放大，保持已验证的 0.20 m/s。
 *
 * ⚠️ 这个常量同时是**闸门 static_assert 的输入**（见 main_task.cpp）：
 *    闸门 ≥ boost×(max_vx+max_vy) + (半轴距+半轮距)·max_spin_w
 *    所以不要单独改它而不改闸门，否则自旋+平移会被等比削减。
 * 运行期可用 g_spin_trans_boost 在 [1.0, 本值] 内调，改完记得写回这里。
 */
inline constexpr float kSpinTransBoostMax = 1.5f;

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
