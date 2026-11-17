/**
 *******************************************************************************
 * @file      : main_task.cpp
 * @brief     : 底盘板初始化与 1ms 周期入口
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. MainTaskInit / MainTask / TIM6 分发
 *******************************************************************************
 * @attention :
 *  1. MainTask 必须保持「薄薄一层」，只调 robot->update()；
 *  2. HAL 回调是 C 链接，必须用 extern "C" 包裹，否则中断里找不到符号；
 *  3. TIM6 的 1ms 已在 main.c 里 HAL_TIM_Base_Start_IT 启动，这里只补分发。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"

#include "chassis.hpp"
#include "chassis_ik_solver.hpp"
#include "comm_task.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "ins_all.hpp"
#include "motor.hpp"
#include "pid.hpp"
#include "robot.hpp"
#include "tim.h"

namespace
{
/* 控制周期，单位 s */
constexpr float kControlPeriodS = 0.001f;

/* ====== 底盘几何参数（考核说明 6.2 的「待填」项，必须以实机测量为准） ======
 * 轮半径 R、半轴距 W/2（前后轮心到中心）、半轮距 L/2（左右轮心到中心）。
 * 未测之前先给出厂近似值，解算结果会有比例误差，别拿它上实车。 */
/* 已补全版考核说明 6.2 给出的实测值 */
constexpr float kWheelRadiusM = 0.07786f;    /* 轮半径 R */
constexpr float kHalfWheelbaseM = 0.15338f;  /* 半轴距 W/2 = 前后距离/2 */
constexpr float kHalfTrackM = 0.15338f;      /* 半轮距 L/2 = 左右距离/2 */
/* 单轮线速度上限（逆解后的最终闸门）。
 * 2026-11-17 第二次提速档 0.3 m/s ≈ 轮子 3.9 rad/s（≈37 rpm）。
 * 这是"最后一道限幅"：任何指令（平移、自转、跟随环）解到轮子上都会被它截住。
 * 恢复顺序：0.3（当前）→ 0.5 → 1.0 → 验收值 4.5。 */
constexpr float kMaxWheelVelMps = 0.3f;

/* 跟随环 PID（w 由「云台朝向 − 车头朝向」闭环给出） */
hello_world::pid::Pid::Config MakeFollowConfig()
{
  hello_world::pid::Pid::Config cfg;
  cfg.kp = 4.0f;    /* rad/s 每 rad */
  cfg.ki = 0.0f;
  cfg.kd = 0.05f;
  cfg.out_min = -6.0f;
  cfg.out_max = 6.0f;
  cfg.int_min = -2.0f;
  cfg.int_max = 2.0f;
  return cfg;
}

/** 组装几何参数 */
hello_world::chassis_ik_solver::ChassisGeometry MakeGeometry()
{
  hello_world::chassis_ik_solver::ChassisGeometry g;
  g.wheel_radius = kWheelRadiusM;
  g.half_wheelbase = kHalfWheelbaseM;
  g.half_track = kHalfTrackM;
  g.max_wheel_vel = kMaxWheelVelMps;
  return g;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/

void MainTaskInit(void)
{
  /* ---------- 1. 通讯层一次性初始化（CAN 滤波器 / 串口 DMA / IWDG） ---------- */
  CommTaskInit();

  /* ---------- 2. 底盘几何参数 ---------- */
  const auto geometry = MakeGeometry();
  if (GetChassisIkSolver() != nullptr) {
    GetChassisIkSolver()->Init(geometry);
  }
  if (GetChassis() != nullptr) {
    GetChassis()->Init(geometry);
  }

  /* ---------- 3. 控制回路 PID 参数 ---------- */
  if (GetRobot() != nullptr) {
    GetRobot()->ConfigureFollowPid(MakeFollowConfig());
  }

  /* ---------- 4. 注册到状态机 ---------- */
  robot::Chassis* chassis = GetChassis();
  robot::Robot* robot = GetRobot();
  hello_world::motor::Motor* motors[hello_world::chassis_ik_solver::kWheelCount] = {
      nullptr, nullptr, nullptr, nullptr};
  hello_world::pid::Pid* pids[hello_world::chassis_ik_solver::kWheelCount] = {
      nullptr, nullptr, nullptr, nullptr};

  for (uint8_t i = 0; i < hello_world::chassis_ik_solver::kWheelCount; ++i) {
    motors[i] = GetMotorWheel(static_cast<int>(i));
    pids[i] = GetPidMotorWheel(static_cast<int>(i));
  }

  if (robot != nullptr) {
    robot->RegisterChassis(chassis);
    robot->RegisterImu(GetImu());
    robot->RegisterRemoteControl(GetRemoteControl());
    robot->RegisterComm(GetGimbalChassisComm());
    robot->RegisterMotors(motors);
    robot->RegisterWheelPids(pids);
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
  /* 只认 TIM6：别的定时器（TIM2 计时 / TIM4 空闲）不能当成 1ms 控制周期 */
  if ((htim != nullptr) && (htim->Instance == TIM6)) {
    MainTask();
    CommTask();
  }
}
