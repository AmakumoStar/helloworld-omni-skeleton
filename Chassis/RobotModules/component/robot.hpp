/**
 *******************************************************************************
 * @file      : robot.hpp
 * @brief     : 底盘板顶层状态机：1ms 入口，Dead / Resurrection / Working 三态
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 初版：三态 + 三模式 + 急停最外层
 *******************************************************************************
 * @attention :
 *  1. 本类是「本板唯一的下发出口」：所有电机指令都必须经由 update() 产生，
 *     这样急停判断才能覆盖全部分支（硬性验收项）。
 *  2. 与云台板只能通过 GimbalChassisComm 的 CAN 帧交换信息，禁止直接调用。
 *  3. 本类不开中断、不做延时；由 Task 层的 htim6 回调每 1ms 调用一次。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_ROBOT_MODULES_ROBOT_HPP_
#define CHASSIS_ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "DT7.hpp"
#include "chassis.hpp"
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
/** 判定「电机已使能」的超时，单位 ms（超时后不再阻塞进入 Working） */
inline constexpr uint32_t kMotorReadyTimeoutMs = 2000u;

/* Exported types ------------------------------------------------------------*/

class Robot
{
 public:
  Robot(void) = default;
  ~Robot(void) = default;

  /* ------------------------------ 注册接口 ------------------------------ */

  void RegisterChassis(Chassis* chassis) { chassis_ = chassis; }
  void RegisterImu(hello_world::imu::Imu* imu) { imu_ = imu; }
  void RegisterRemoteControl(hello_world::remote_control::DT7* rc) { rc_ = rc; }
  void RegisterComm(GimbalChassisComm* comm) { comm_ = comm; }

  /**
   * @brief       注册四个轮电机
   * @param       motors: 轮序 0=LF 1=LB 2=RB 3=RF
   */
  void RegisterMotors(hello_world::motor::Motor* const motors[hello_world::chassis_ik_solver::kWheelCount])
  {
    for (uint8_t i = 0; i < hello_world::chassis_ik_solver::kWheelCount; ++i) {
      motors_[i] = motors[i];
    }
  }

  /** 配置跟随环 PID（MainTaskInit 里调用，整定时也可在 Ozone 里热改） */
  void ConfigureFollowPid(const hello_world::pid::Pid::Config& config)
  {
    follow_pid_.Init(config);
  }

  /** 注册四个轮速环 PID，轮序同上 */
  void RegisterWheelPids(
      hello_world::pid::Pid* const pids[hello_world::chassis_ik_solver::kWheelCount])
  {
    for (uint8_t i = 0; i < hello_world::chassis_ik_solver::kWheelCount; ++i) {
      wheel_pids_[i] = pids[i];
    }
  }

  /**
   * @brief       DBUS 收到一帧有效数据时由 CommTask 调用
   * @note        用来维护遥控器在线状态，避免 Robot 依赖串口细节
   */
  void NotifyRemoteFrame(void) { rc_checker_.update(); }

  /* ------------------------------ 周期入口 ------------------------------ */

  /**
   * @brief       1ms 周期入口：更新数据 → 状态迁移 → 生成指令 → 下发
   * @param       dt: 周期，单位 s
   * @retval      None
   */
  void Update(float dt);

  /** 复位：清残留指令、清 PID 积分（状态切换 / 上电时调用） */
  void Reset(void);

  /* ------------------------------ 数据获取 ------------------------------ */

  LifeState life_state(void) const { return life_state_; }
  ChassisMode mode(void) const { return mode_; }
  bool estop(void) const { return estop_; }
  const ChassisVel& command(void) const { return command_; }
  const LifeInputs& inputs(void) const { return inputs_; }
  /** 云台相对车头的偏航角（δ），rad */
  float delta(void) const { return delta_; }
  /** delta 来源是否有效（双板通讯在线） */
  bool deltaValid(void) const { return inputs_.peer_online; }

 private:
  /** 平移解算用的坐标系角度：g_ignore_delta=1 时为 0（车体系），否则为真实 δ */
  float kFrameDelta(void) const;
  /** 采集本周期输入，填充 inputs_ */
  void UpdateInputs(void);
  /** 执行状态迁移并处理进入动作 */
  void UpdateState(void);
  /** Working 态下按模式生成云台系速度指令 */
  void GenerateCommands(float dt);
  /** 把轮速指令下发到电机；dt 用于速度环 PID */
  void OutputWheels(float dt);
  /** 全零输出（Dead / 急停） */
  void OutputZero(void);

  /* 模块指针（由 Instance 层注册） */
  Chassis* chassis_ = nullptr;
  hello_world::imu::Imu* imu_ = nullptr;
  hello_world::remote_control::DT7* rc_ = nullptr;
  GimbalChassisComm* comm_ = nullptr;
  hello_world::motor::Motor* motors_[hello_world::chassis_ik_solver::kWheelCount] = {
      nullptr, nullptr, nullptr, nullptr};
  hello_world::pid::Pid* wheel_pids_[hello_world::chassis_ik_solver::kWheelCount] = {
      nullptr, nullptr, nullptr, nullptr};

  /* 状态 */
  LifeState life_state_ = LifeState::kDead;
  ChassisMode mode_ = ChassisMode::kSeparate;
  /* 上电互锁（2026-11-17）：三挡拨杆必须先在"分离"档出现一次，之后才按拨杆选模式。
   * 防止上电时拨杆停在小陀螺档、一武装就自旋。复位后自动清零（RAM）。 */
  bool mode_sw_latched_ = false;
  LifeInputs inputs_{};
  ChassisVel command_{};   ///< 本周期云台系指令
  float delta_ = 0.0f;     ///< 云台相对车头的偏航角
  bool estop_ = false;     ///< 急停档（最外层判定）
  uint32_t ready_tick_ = 0;  ///< 进入 Resurrection 的时刻，用于电机使能超时
  /* 上电武装计时（ms）：三轴摇杆回中并保持 kArmHoldMs 后才允许输出。
   * 目的：防止「带着摇杆上电」时底盘立刻窜车——实机踩过这个坑。 */
  float arm_ms_ = 0.0f;

  /* 分离模式航向保持（2026-11-17）：目标 = 进分离那一刻的车头绝对朝向。
   * sep_hold_valid_ = false 时，GenerateCommands 会先取一次当前朝向当目标；
   * 进分离模式、或从 Dead 回到 Working 时置 false。 */
  float sep_yaw_target_ = 0.0f;
  bool sep_hold_valid_ = false;
  /* 上一拍的模式，用来检测"刚进分离"这个沿 */
  ChassisMode last_mode_ = ChassisMode::kSeparate;

  /* 跟随模式 PID（w 由「云台朝向 − 车头朝向」闭环给出） */
  hello_world::pid::Pid follow_pid_{};
  /* 遥控器在线检测（由 NotifyRemoteFrame 刷新） */
  hello_world::OfflineChecker rc_checker_{kRcOfflineThresMs};
  /* 双板帧序号（自增，用于对面判断丢帧） */
  uint8_t comm_seq_ = 0u;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_ROBOT_HPP_ */
