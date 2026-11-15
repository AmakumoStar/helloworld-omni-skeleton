/**
 *******************************************************************************
 * @file      : robot.cpp
 * @brief     : 底盘板顶层状态机实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 三态 + 三模式 + 急停最外层
 *******************************************************************************
 * @attention :
 *  下发链路只有一条：Update() → OutputWheels() / OutputZero()。
 *  急停判断放在 Update() 的最外层，任何模式分支都绕不过去。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "robot.hpp"

#include "chassis_ik_solver.hpp"
#include "tick.hpp"

namespace robot
{
/* Private constants ---------------------------------------------------------*/

namespace
{
/** 轮子数量 */
constexpr uint8_t kWheelCount = hello_world::chassis_ik_solver::kWheelCount;
}  // namespace

/* Exported function definitions ---------------------------------------------*/

void Robot::Update(float dt)
{
  /* 推进 IMU：零漂采样 + 姿态解算都在这里，必须每周期调用，
   * 否则 isCalibrated() 永远是 false，状态机永远进不了 Working。 */
  if (imu_ != nullptr) {
    imu_->Update();
  }

  UpdateInputs();
  UpdateState();

  /* 急停 / 非 Working：一律全零，绝不留任何残留指令 */
  if (estop_ || (life_state_ != LifeState::kWorking)) {
    OutputZero();
  } else {
    GenerateCommands(dt);
    OutputWheels(dt);
  }

  /* 填充本板要发给云台的报文（真正的 encode + 发送在 CommTask 里做） */
  if (comm_ != nullptr) {
    ChassisData& d = comm_->chassis_data();
    d.online = true;
    d.ready = (life_state_ == LifeState::kWorking);
    d.estop = estop_;
    d.mode = mode_;
    d.gimbal_mode = GimbalMode::kJointSpace;
    /* 遥控接收机实际接在【底盘板】上，云台板收不到 DBUS。
     * 因此把右摇杆的归一化值（-1~1）放进 yaw_des / pitch_des 转发给云台板，
     * 云台侧再乘自己的速率上限。这样在不动接线的前提下，云台也能被操作手控制。 */
    if (rc_ != nullptr) {
      /* 同样先过死区，避免中位抖动被转成云台的持续漂移 */
      d.yaw_des = -ApplyStickDeadzone(rc_->rc_rh());   /* 右摇杆横向 → yaw 角速度（归一化） */
      d.pitch_des = ApplyStickDeadzone(rc_->rc_rv());  /* 右摇杆纵向 → pitch 角速度（归一化） */
    } else {
      d.yaw_des = 0.0f;
      d.pitch_des = 0.0f;
    }
    d.w_des = command_.w;
    d.seq = ++comm_seq_;
  }
}

void Robot::Reset(void)
{
  command_.setZero();
  if (chassis_ != nullptr) {
    chassis_->Stop();
  }
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    if (wheel_pids_[i] != nullptr) {
      wheel_pids_[i]->Reset();
    }
    if (motors_[i] != nullptr) {
      motors_[i]->setInput(0.0f);
    }
  }
  follow_pid_.Reset();
}

void Robot::UpdateInputs(void)
{
  /* ---- 遥控器 ---- */
  bool rc_online = (rc_ != nullptr) && (!rc_checker_.isOffline());
  bool estop_switch = false;
  if (rc_ != nullptr) {
    estop_switch =
        (rc_->rc_l_switch() == hello_world::remote_control::kSwitchStateDown);
    /* 右拨杆选模式：Up = 分离，Mid = 跟随，Down = 小陀螺 */
    switch (rc_->rc_r_switch()) {
      case hello_world::remote_control::kSwitchStateMid:
        mode_ = ChassisMode::kFollow;
        break;
      case hello_world::remote_control::kSwitchStateDown:
        mode_ = ChassisMode::kSpin;
        break;
      case hello_world::remote_control::kSwitchStateUp:
      default:
        mode_ = ChassisMode::kSeparate;
        break;
    }
  }
  estop_ = estop_switch;

  /* ---- IMU ---- */
  const bool imu_ready = (imu_ != nullptr) && imu_->isCalibrated();

  /* ---- 电机 ---- */
  bool motors_ok = true;
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    if (motors_[i] == nullptr) {
      motors_ok = false;
    }
  }

  /* ---- 双板通讯 ---- */
  const bool peer_online = (comm_ != nullptr) && (!comm_->isOffline());
  delta_ = peer_online ? comm_->gimbal_data().yaw : 0.0f;

  inputs_.imu_ready = imu_ready;
  inputs_.motors_ready = motors_ok;
  inputs_.rc_online = rc_online;
  inputs_.peer_online = peer_online;
  inputs_.estop = estop_switch;
}

void Robot::UpdateState(void)
{
  const LifeState next = NextLifeState(life_state_, inputs_);
  if (next == life_state_) {
    return;
  }

  /* 进入新状态前的动作 */
  if (next == LifeState::kResurrection) {
    ready_tick_ = hello_world::tick::GetTickMs();
  }
  /* 只要不是 Working，就把残留指令与积分全部清掉，防止切回来时突跳 */
  if (next != LifeState::kWorking) {
    Reset();
  }

  life_state_ = next;
}

void Robot::GenerateCommands(float dt)
{
  if ((rc_ == nullptr) || (chassis_ == nullptr)) {
    command_.setZero();
    return;
  }

  const ChassisLimits& lim = chassis_->limits();

  /* 左摇杆：竖 → 前后，横 → 左右（右推为 −y，因为 y 轴指向左方）
   * 所有摇杆量先过死区，保证中位抖动不会变成残余速度（零输入即零输出） */
  const float vx_cmd = ApplyStickDeadzone(rc_->rc_lv()) * lim.max_vx;
  const float vy_cmd = -ApplyStickDeadzone(rc_->rc_lh()) * lim.max_vy;
  /* 右摇杆横向 → 原地转向（右推为顺时针，即负 w） */
  const float w_stick = -ApplyStickDeadzone(rc_->rc_rh()) * lim.max_w;

  switch (mode_) {
    case ChassisMode::kFollow: {
      /* w 由「云台朝向 − 车头朝向」闭环给出；δ 就是两者之差 */
      const float w_follow = follow_pid_.UpdateErr(WrapToPi(delta_), dt);
      command_.vx = vx_cmd;
      command_.vy = vy_cmd;
      command_.w = Clamp(w_follow, -lim.max_w, lim.max_w);
      break;
    }
    case ChassisMode::kSpin: {
      /* 小陀螺：w 由拨轮设定，必须限幅；平移仍以云台朝向为基准 */
      const float w_spin = ApplyStickDeadzone(rc_->rc_wheel()) * lim.max_spin_w;
      command_.vx = vx_cmd;
      command_.vy = vy_cmd;
      command_.w = Clamp(w_spin, -lim.max_spin_w, lim.max_spin_w);
      break;
    }
    case ChassisMode::kSeparate:
    default: {
      command_.vx = vx_cmd;
      command_.vy = vy_cmd;
      command_.w = w_stick;
      break;
    }
  }

  chassis_->SetCommand(command_, delta_);
}

void Robot::OutputWheels(float dt)
{
  if (chassis_ == nullptr) {
    return;
  }

  chassis_->Update(dt);

  const float radius = chassis_->ik_solver().geometry().wheel_radius;
  const hello_world::chassis_ik_solver::WheelVel& wheel = chassis_->wheel_vel();

  for (uint8_t i = 0; i < kWheelCount; ++i) {
    if ((motors_[i] == nullptr) || (wheel_pids_[i] == nullptr)) {
      continue;
    }
    /* 轮速 m/s → 轮子角速度 rad/s → 速度环 PID → 电流/转矩输入 */
    const float target_rad_s = (radius > 0.0f) ? (wheel.v[i] / radius) : 0.0f;
    const float out = wheel_pids_[i]->Update(target_rad_s, motors_[i]->vel(), dt);
    motors_[i]->setInput(out);
  }
}

void Robot::OutputZero(void)
{
  command_.setZero();
  if (chassis_ != nullptr) {
    chassis_->Stop();
  }
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    if (wheel_pids_[i] != nullptr) {
      wheel_pids_[i]->Reset();
    }
    if (motors_[i] != nullptr) {
      motors_[i]->setInput(0.0f);
    }
  }
}

/* Private function definitions ----------------------------------------------*/
}  // namespace robot
