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
  /* ---------- pitch 软件限位：2026-11-17 实机实测（不再是猜的） ----------
   * 左手摇杆拉到底（急停档，全车零转矩把 pitch 变软）后手推关节到两端，读
   * pitch_motor_->angle()（经 0x211 帧转发到云端板的 gimbal_data.pitch）：
   *   机械下限  -0.111 rad（-6.4°）   机械上限  +0.687 rad（+39.4°）
   *   总行程     0.798 rad（45.7°）   + = 抬头（往上推读数值变大）
   * 各留 5°（0.0873 rad）余量：
   *   下限 = -0.111 + 0.0873 = -0.024 → 取 -0.02
   *   上限 = +0.687 - 0.0873 = +0.600 → 取  0.60（原值恰好正确）
   * ⚠️ 原值是 -0.35，比机械下限还低 0.24 rad（13.7°）：下半程一直在以 1 N·m 死顶
   *    机械限位，既磨结构又骗积分。这也是"pitch 目标被钳在区间外"类问题的来源。
   * ⚠️ 因为下限现在高于静止位，进 Working 时目标会被钳到 -0.02，云台会自己抬 5°；
   *    这是"留余量"的正常代价。若想要一点下俯行程，把余量降到 3°（pitch_min=-0.06）。 */
  lim.pitch_min = -0.02f;
  lim.pitch_max = 0.60f;
  /* ⚠️ 安全配置：整定前先用小速率、小转矩跑通，确认手感后再逐步放开。
   *    2026-11-16 曾因 ki 过大 + 转矩上限 1.0 N·m 导致 yaw 失控飞转。 */
  lim.max_yaw_rate = 2.0f;    /* 摇杆打满时的 yaw 角速度，rad/s（安全值；手感合适后可提到 6.0） */
  lim.max_pitch_rate = 1.0f;  /* 摇杆打满时的 pitch 角速度，rad/s */
  /* 2026-11-17 由 0.25 提高到 1.0：0.25 N·m 时饱和点只有 err=0.167 rad（9.6°），
   * 底盘小陀螺自旋会把云台拖着走 —— 云台持续输出 0.25 N·m 反向力矩，既拖慢底盘
   * （底盘转动惯量约 0.22 kg·m²，0.25 N·m 约 1.1 rad/s² 角减速度，0.4 s 就能拖停），
   * 又让 δ 不随底盘旋转变化，导致"前进方向随底盘朝向漂移"。
   * 1.0 N·m 是"能拽住自己"的最低档；抑制 11-16 飞转的三道保护仍然有效：
   *   ki = 0、积分限幅 ±0.15、|err| > 34° 清积分。 */
  lim.max_torque = 1.0f;
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
  /* 积分限幅收紧到 ±0.15：即使以后把 ki 调大，积分也顶不出一股持续大转矩
   * （2026-11-16 的失控就是积分把输出顶到饱和造成的） */
  cfg.int_min = -0.15f;
  cfg.int_max = 0.15f;
  return cfg;
}

/** pitch 位置环参数（初值 = 运行期可调值，见文件末尾的 g_pitch_*） */
hello_world::pid::Pid::Config MakePitchConfig()
{
  hello_world::pid::Pid::Config cfg;
  cfg.kp = hello_world::tune::g_pitch_kp;
  cfg.ki = 0.0f;   /* 防飞转红线：必须 0 */
  cfg.kd = hello_world::tune::g_pitch_kd;
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
volatile float hello_world::tune::g_yaw_kp = 1.5f;
volatile float hello_world::tune::g_yaw_ki = 0.0f;
volatile float hello_world::tune::g_yaw_kd = 0.10f;

/* pitch 位置环：实车"手拨不够硬"，2026-11-17 由 5.0/0.12 提到 10.0/0.25。
 * ki 仍然为 0（防飞转红线，别加）。 */
volatile float hello_world::tune::g_pitch_kp = 10.0f;
volatile float hello_world::tune::g_pitch_kd = 0.25f;
