/**
 *******************************************************************************
 * @file      : robot.cpp
 * @brief     : 云台板顶层状态机实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 三态 + 摇杆积分目标 + 急停最外层
 *******************************************************************************
 * @attention :
 *  下发链路只有一条：Update() → OutputMotors() / OutputZero()。
 *  急停判断放在 Update() 最外层，任何分支都绕不过去。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "robot.hpp"

#include "bench_test.hpp"
#include "motor_DaMiao.hpp"
#include "pid_tune.hpp"
#include "tick.hpp"

namespace robot
{
/* Private constants ---------------------------------------------------------*/

namespace
{
/* 台架自测开关：
 *   false（默认）→ 正常用遥控器操作，验收必须保持 false
 *   true         → 不依赖遥控器，用带自动停止的方波目标让云台往复转动，
 *                   仅在"接收机坏了但仍要调 PID"时临时打开 */
constexpr bool kBenchTest = false;
constexpr hello_world::bench::Config kBenchCfg{};

/* 达妙电机的 MIT 接口在派生类里；本工程编译时关闭了 RTTI（-fno-rtti），
 * 因此这里只能做静态向下转型，前提是 Instance 层注册的确实是 DM-J4310。 */
hello_world::motor::DaMiao* AsDaMiao(hello_world::motor::Motor* motor)
{
  return static_cast<hello_world::motor::DaMiao*>(motor);
}
}  // namespace

/* Exported function definitions ---------------------------------------------*/

void Robot::Update(float dt)
{
  /* 记录上电时刻，供「电机就绪超时兜底」使用（只记录一次） */
  if (boot_tick_ == 0u) {
    boot_tick_ = hello_world::tick::GetTickMs();
  }

  /* 推进 IMU：零漂采样 + 姿态解算都在这里，必须每周期调用，
   * 否则 isCalibrated() 永远是 false，状态机永远进不了 Working。 */
  if (imu_ != nullptr) {
    imu_->Update();
  }

  UpdateInputs();
  UpdateState();

  if (estop_ || (life_state_ != LifeState::kWorking)) {
    OutputZero();
  } else {
    GenerateCommands(dt);
    OutputMotors();
  }

  /* 填充本板要发给底盘的报文（encode + 发送在 CommTask 里做） */
  if (comm_ != nullptr) {
    GimbalData& d = comm_->gimbal_data();
    d.online = true;
    d.ready = (life_state_ == LifeState::kWorking);
    d.estop = estop_;
    d.mode = mode_;
    d.yaw = (yaw_motor_ != nullptr) ? yaw_motor_->angle() : 0.0f;
    d.pitch = (pitch_motor_ != nullptr) ? pitch_motor_->angle() : 0.0f;
    d.yaw_vel = (yaw_motor_ != nullptr) ? yaw_motor_->vel() : 0.0f;
    d.seq = ++comm_seq_;
  }
}

void Robot::Reset(void)
{
  /* yaw 目标必须和「反馈量的坐标系」一致：反馈用 IMU 绝对角，目标也要用绝对角。
   * 用关节角当目标会导致一进 Working 就大幅跳转（两者零位不同）。 */
  const float yaw_target = (imu_ != nullptr) ? imu_->yaw()
                                             : ((yaw_motor_ != nullptr) ? yaw_motor_->angle() : 0.0f);
  target_ = GimbalPos(yaw_target,
                      (pitch_motor_ != nullptr) ? pitch_motor_->angle() : 0.0f);
  if (gimbal_ != nullptr) {
    gimbal_->SetTarget(target_);
    gimbal_->Stop();
    gimbal_->yaw_pid().Reset();
    gimbal_->pitch_pid().Reset();
  }
  OutputZero();
}

void Robot::UpdateInputs(void)
{
  bool estop_switch = false;
  if (rc_ != nullptr) {
    estop_switch =
        (rc_->rc_l_switch() == hello_world::remote_control::kSwitchStateDown);
  }
  estop_ = estop_switch;

  const bool imu_ready = (imu_ != nullptr) && imu_->isCalibrated();
  const bool local_rc_online = (rc_ != nullptr) && (!rc_checker_.isOffline());
  const bool peer_online = (comm_ != nullptr) && (!comm_->isOffline());
  /* 遥控接收机接在底盘板上：只要底盘在线，摇杆值就会经双板帧转发过来，
   * 等效于本板也有控制源（底盘自己已经用它的 rc_online 把关）。 */
  const bool rc_online = local_rc_online || peer_online;
  /* 双板互保：对面报告急停，本板也立刻归零 */
  if (peer_online && comm_->chassis_data().estop) {
    estop_ = true;
  }

  /* 达妙电机上电失能，必须等它回应「已使能」才算就绪。
   * 兜底：上电超过 kMotorReadyTimeoutMs 且 yaw 已使能，就认为"电机可用"
   * （pitch 硬件可能缺失）。判定用纯函数 MotorsReady()，与当前状态无关。 */
  bool motors_ok = (yaw_motor_ != nullptr) && (pitch_motor_ != nullptr);
  if (motors_ok) {
    const bool yaw_ok = AsDaMiao(yaw_motor_)->is_enabled();
    const bool pitch_ok = AsDaMiao(pitch_motor_)->is_enabled();
    /* 调试开关：只要求 yaw 使能（pitch 硬件没通时用）。
     * ⚠️ 验收必须为 false —— 考核要求两轴闭环。 */
    constexpr bool kRequireBothJoints = true;
    motors_ok = MotorsReady(yaw_ok, pitch_ok, kRequireBothJoints,
                            hello_world::tick::GetTickMs(), boot_tick_,
                            kMotorReadyTimeoutMs);
  }

  inputs_.imu_ready = imu_ready;
  inputs_.motors_ready = motors_ok;
  inputs_.rc_online = rc_online;
  inputs_.peer_online = peer_online;
  inputs_.estop = estop_;

  /* 台架自测下伪装"遥控在线"，否则没有遥控器就永远进不了 Working */
  if constexpr (kBenchTest) {
    inputs_.rc_online = true;
  }
}

void Robot::UpdateState(void)
{
  const LifeState next = NextLifeState(life_state_, inputs_);
  if (next == life_state_) {
    return;
  }

  if (next == LifeState::kResurrection) {
    ready_tick_ = hello_world::tick::GetTickMs();
  }
  /* 任何状态切换都清一遍：残留指令、PID 积分，并把目标对齐到当前实际指向。
   * 注意进 Working 也要对齐——IMU 的 yaw 在 Dead/Resurrection 期间可能已经漂移，
   * 不对齐的话一进 Working 云台会突然甩一下。 */
  Reset();

  life_state_ = next;
}

void Robot::GenerateCommands(float dt)
{
  if (gimbal_ == nullptr) {
    return;
  }

  /* 调参期：每周期把「可调增益」写进 yaw 位置环（set_config 不清积分状态），
   * 这样用 J-Link 直接改内存就能实时调参，不必反复编译烧录。 */
  {
    hello_world::pid::Pid::Config cfg = gimbal_->yaw_pid().config();
    cfg.kp = hello_world::tune::g_yaw_kp;
    cfg.ki = hello_world::tune::g_yaw_ki;
    cfg.kd = hello_world::tune::g_yaw_kd;
    gimbal_->yaw_pid().set_config(cfg);
  }

  const GimbalLimits& lim = gimbal_->limits();
  const bool local_rc_online = (rc_ != nullptr) && (!rc_checker_.isOffline());

  /* yaw 用 IMU 绝对角做反馈：底盘转动时云台才会"反抗"并保持指向，
   * 这是「跟随」能收敛、「小陀螺」枪口不跟着车转的前提。 */
  const float yaw_feedback =
      (imu_ != nullptr) ? imu_->yaw()
                        : ((yaw_motor_ != nullptr) ? yaw_motor_->angle() : 0.0f);
  const float pitch_feedback =
      (pitch_motor_ != nullptr) ? pitch_motor_->angle() : 0.0f;

  if constexpr (kBenchTest) {
    /* ---- 台架自测：不用遥控器，用带自动停止的方波目标 ----
     * 安全性：幅度小（约 6.9°）、1.5 s 一个台阶、30 s 后自动回中位并停住。 */
    if (!bench_armed_) {
      bench_armed_ = true;
      bench_t0_ = hello_world::tick::GetTickMs();
      bench_base_ = yaw_feedback;
    }
    const uint32_t t_ms = hello_world::tick::GetTickMs() - bench_t0_;
    GimbalPos p = gimbal_->target();
    p.yaw = bench_base_ + hello_world::bench::SquareTarget(t_ms, kBenchCfg);
    gimbal_->SetTarget(p);
  } else {
    /* 摇杆按速率积分：右摇杆横向 → yaw，纵向 → pitch。
     * 本板读不到 DBUS 时（接收机在底盘板上），改用底盘转发的归一化值。 */
    float yaw_norm = 0.0f;
    float pitch_norm = 0.0f;
    if (local_rc_online) {
      /* 本板直接读摇杆时要先过死区；走转发路径时底盘已经做过死区，不用重复 */
      yaw_norm = -ApplyStickDeadzone(rc_->rc_rh());
      pitch_norm = ApplyStickDeadzone(rc_->rc_rv());
    } else if ((comm_ != nullptr) && (!comm_->isOffline())) {
      /* 防御：转发值必须落在 [-1,1]，否则说明对方数据异常，按边界处理 */
      yaw_norm = Clamp(comm_->chassis_data().yaw_des, -1.0f, 1.0f);
      pitch_norm = Clamp(comm_->chassis_data().pitch_des, -1.0f, 1.0f);
    }

    GimbalPos step;
    step.yaw = (yaw_norm * lim.max_yaw_rate) * dt;
    step.pitch = (pitch_norm * lim.max_pitch_rate) * dt;
    gimbal_->MoveBy(step);
  }

  gimbal_->Update(dt, yaw_feedback, pitch_feedback);
  target_ = gimbal_->target();
}

void Robot::OutputMotors(void)
{
  if (gimbal_ == nullptr) {
    return;
  }

  if (yaw_motor_ != nullptr) {
    /* kp = kd = 0 时 MIT 退化为纯转矩输出，与自写位置环配合使用 */
    AsDaMiao(yaw_motor_)->setMitInput(yaw_motor_->angle(), 0.0f,
                                      gimbal_->mit_kp(), gimbal_->mit_kd(),
                                      gimbal_->output().yaw);
  }
  if (pitch_motor_ != nullptr) {
    AsDaMiao(pitch_motor_)->setMitInput(pitch_motor_->angle(), 0.0f,
                                        gimbal_->mit_kp(), gimbal_->mit_kd(),
                                        gimbal_->output().pitch);
  }
}

void Robot::OutputZero(void)
{
  /* 全零转矩：不能用 setInput(0)（达妙的输入类型是转矩，但 MIT 通道更直接） */
  if (yaw_motor_ != nullptr) {
    AsDaMiao(yaw_motor_)->setMitInput(yaw_motor_->angle(), 0.0f, 0.0f, 0.0f, 0.0f);
  }
  if (pitch_motor_ != nullptr) {
    AsDaMiao(pitch_motor_)->setMitInput(pitch_motor_->angle(), 0.0f, 0.0f, 0.0f, 0.0f);
  }
}

/* Private function definitions ----------------------------------------------*/
}  // namespace robot
