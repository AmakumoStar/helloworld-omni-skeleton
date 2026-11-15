/**
 *******************************************************************************
 * @file      : robot.hpp
 * @brief     : 云台板顶层状态机：1ms 入口，Dead / Resurrection / Working 三态
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 初版：三态 + 摇杆积分目标 + 急停最外层
 *******************************************************************************
 * @attention :
 *  1. 本类是「本板唯一的下发出口」，所有电机指令都经由 update() 产生，
 *     保证急停判断覆盖全部分支。
 *  2. 与底盘板只能通过 GimbalChassisComm 的 CAN 帧交换信息。
 *  3. 遥控器接在云台板上，因此 yaw / pitch 目标由本板摇杆积分得到；
 *     底盘板发来的 yaw_des / pitch_des 字段保留为「底盘指定目标」的扩展位。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_ROBOT_MODULES_ROBOT_HPP_
#define GIMBAL_ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "DT7.hpp"
#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "module_state.hpp"
#include "motor.hpp"
#include "offline_checker.hpp"
#include "pid.hpp"

namespace robot
{
/* Exported constants --------------------------------------------------------*/

/** 遥控器在线判断阈值，单位 ms */
inline constexpr uint32_t kRcOfflineThresMs = 100u;
/** 电机使能等待超时，单位 ms */
inline constexpr uint32_t kMotorReadyTimeoutMs = 2000u;

/* Exported types ------------------------------------------------------------*/

class Robot
{
 public:
  Robot(void) = default;
  ~Robot(void) = default;

  /* ------------------------------ 注册接口 ------------------------------ */

  void RegisterGimbal(Gimbal* gimbal) { gimbal_ = gimbal; }
  void RegisterImu(hello_world::imu::Imu* imu) { imu_ = imu; }
  void RegisterRemoteControl(hello_world::remote_control::DT7* rc) { rc_ = rc; }
  void RegisterComm(GimbalChassisComm* comm) { comm_ = comm; }
  void RegisterYawMotor(hello_world::motor::Motor* motor) { yaw_motor_ = motor; }
  void RegisterPitchMotor(hello_world::motor::Motor* motor) { pitch_motor_ = motor; }

  /** DBUS 收到一帧有效数据时由 CommTask 调用 */
  void NotifyRemoteFrame(void) { rc_checker_.update(); }

  /* ------------------------------ 周期入口 ------------------------------ */

  void Update(float dt);
  void Reset(void);

  /* ------------------------------ 数据获取 ------------------------------ */

  LifeState life_state(void) const { return life_state_; }
  GimbalMode mode(void) const { return mode_; }
  bool estop(void) const { return estop_; }
  const LifeInputs& inputs(void) const { return inputs_; }
  const GimbalPos& target(void) const { return target_; }

 private:
  void UpdateInputs(void);
  void UpdateState(void);
  /** Working 态：摇杆积分目标 + 位置环 */
  void GenerateCommands(float dt);
  /** 把转矩经 MIT 接口下发 */
  void OutputMotors(void);
  /** 全零输出 */
  void OutputZero(void);

  /* 模块指针 */
  Gimbal* gimbal_ = nullptr;
  hello_world::imu::Imu* imu_ = nullptr;
  hello_world::remote_control::DT7* rc_ = nullptr;
  GimbalChassisComm* comm_ = nullptr;
  hello_world::motor::Motor* yaw_motor_ = nullptr;
  hello_world::motor::Motor* pitch_motor_ = nullptr;

  /* 状态 */
  LifeState life_state_ = LifeState::kDead;
  GimbalMode mode_ = GimbalMode::kJointSpace;
  LifeInputs inputs_{};
  GimbalPos target_{};     ///< 本周期目标关节角
  bool estop_ = false;
  uint32_t ready_tick_ = 0;
  uint8_t comm_seq_ = 0u;
  hello_world::OfflineChecker rc_checker_{kRcOfflineThresMs};
  /** 上电时刻 tick(ms)，用于电机就绪的超时兜底（第一次 Update 时记录） */
  uint32_t boot_tick_ = 0u;
  /** 台架自测（不依赖遥控器）：基准绝对角、开始时刻、是否已启动 */
  bool bench_armed_ = false;
  uint32_t bench_t0_ = 0u;
  float bench_base_ = 0.0f;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_ROBOT_HPP_ */
