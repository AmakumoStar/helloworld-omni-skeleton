/**
 *******************************************************************************
 * @file      : main_task.cpp
 * @brief     : 云台板初始化与 1ms 周期入口
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. MainTaskInit / MainTask / TIM6 分发
 *******************************************************************************
 * @attention :
 *  1. MainTask 只调 robot->update()，业务逻辑都在 RobotModules 里；
 *  2. HAL 回调是 C 链接，必须 extern "C"；
 *  3. main.c 的 USER CODE 区已留 MainTaskInit() 的调用位置。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"

#include "comm_task.hpp"
#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "ins_all.hpp"
#include "motor.hpp"
#include "pid.hpp"
#include "pid_tune.hpp"
#include "robot.hpp"
#include "tim.h"

namespace
{
constexpr float kControlPeriodS = 0.001f;

/* -------------------------------------------------------------------------
 * 运行期可调的 yaw 增益（定义放在全局作用域，见文件末尾）
 * ------------------------------------------------------------------------- */

/** 云台限位与速率参数（pitch 限位必须按实机结构测，别照抄） */
robot::GimbalLimits MakeGimbalLimits()
{
  robot::GimbalLimits lim;
  lim.pitch_min = -0.35f;     /* 待测：向下俯仰的机械极限（留 5° 余量） */
  lim.pitch_max = 0.60f;      /* 待测：向上抬升的机械极限（留 5° 余量） */
  lim.max_yaw_rate = 6.0f;    /* 摇杆打满时的 yaw 角速度，rad/s */
  lim.max_pitch_rate = 3.0f;  /* 摇杆打满时的 pitch 角速度，rad/s */
  lim.max_torque = 1.0f;      /* 位置环转矩上限（首次试转临时限幅，改回 3.0；电机上限 7.5 N·m） */
  return lim;
}

/** yaw 位置环参数
 * 实测 kp = 6 会明显震荡：输出转矩上限只有 1 N·m，误差超过 0.17 rad 就饱和，
 * 变成"打满—回弹"的极限环。所以把 kp 降下来（饱和点推到 0.5 rad），kd 加大做阻尼。 */
hello_world::pid::Pid::Config MakeYawConfig()
{
  hello_world::pid::Pid::Config cfg;
  cfg.kp = hello_world::tune::g_yaw_kp;
  cfg.ki = hello_world::tune::g_yaw_ki;
  cfg.kd = hello_world::tune::g_yaw_kd;
  cfg.out_min = -3.0f;
  cfg.out_max = 3.0f;
  cfg.int_min = -1.0f;
  cfg.int_max = 1.0f;
  return cfg;
}

/** pitch 位置环参数（整定起点） */
hello_world::pid::Pid::Config MakePitchConfig()
{
  hello_world::pid::Pid::Config cfg;
  cfg.kp = 5.0f;
  cfg.ki = 0.0f;
  cfg.kd = 0.12f;
  cfg.out_min = -2.0f;
  cfg.out_max = 2.0f;
  cfg.int_min = -1.0f;
  cfg.int_max = 1.0f;
  return cfg;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/

void MainTaskInit(void)
{
  /* ---------- 1. 通讯层 ---------- */
  CommTaskInit();

  /* ---------- 2. 云台限位 ---------- */
  if (GetGimbal() != nullptr) {
    GetGimbal()->Init(MakeGimbalLimits());
  }

  /* ---------- 3. 两个位置环的 PID 参数 ---------- */
  if (GetPidMotorYaw() != nullptr) {
    GetPidMotorYaw()->Init(MakeYawConfig());
  }
  if (GetPidMotorPitch() != nullptr) {
    GetPidMotorPitch()->Init(MakePitchConfig());
  }
  /* Gimbal 内部用的是自己的 PID 实例，这里同步同一套参数 */
  if (GetGimbal() != nullptr) {
    GetGimbal()->yaw_pid().Init(MakeYawConfig());
    GetGimbal()->pitch_pid().Init(MakePitchConfig());
  }

  /* ---------- 4. 注册 ---------- */
  robot::Robot* robot = GetRobot();
  if (robot != nullptr) {
    robot->RegisterGimbal(GetGimbal());
    robot->RegisterImu(GetImu());
    robot->RegisterRemoteControl(GetRemoteControl());
    robot->RegisterComm(GetGimbalChassisComm());
    robot->RegisterYawMotor(GetMotorYaw());
    robot->RegisterPitchMotor(GetMotorPitch());
    robot->Reset();
  }
}

void MainTask(void)
{
  robot::Robot* robot = GetRobot();
  if (robot != nullptr) {
    robot->Update(kControlPeriodS);
  }
}

/* Private function definitions ----------------------------------------------*/

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if ((htim != nullptr) && (htim->Instance == TIM6)) {
    MainTask();
    CommTask();
  }
}

/* ===== 运行期可调增益的定义（初值 = 当前整定值） =====
 * 调参时用 J-Link 直接写这三个变量的内存即可，不用重新编译：
 *   mem32 <addr> 1   （内容是 IEEE-754 位模式）
 * 验收前请把最终值同步写回 MakeYawConfig() 的默认值。 */
volatile float hello_world::tune::g_yaw_kp = 2.0f;
volatile float hello_world::tune::g_yaw_ki = 0.0f;
volatile float hello_world::tune::g_yaw_kd = 0.35f;
