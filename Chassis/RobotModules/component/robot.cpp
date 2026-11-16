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

/* ---------- 台架模式用的一致性变量（可用 J-Link 在线写） ----------
 * 归一化量，实际指令 = 该值 × limits_ 上限。默认 0，上电不动。
 *   g_bench_vx > 0 前进      g_bench_vy > 0 向左      g_bench_w > 0 逆时针自转
 * g_bench_open_duty：开环直给电调的比例（0.1 = 10% 满量程），用于先验证转向。
 *   非 0 时绕过速度环，四轮直给同一个电流 —— 这是区分「指令链问题」和「电调/接线问题」
 *   最直接的一刀。 */
volatile float g_bench_vx = 0.0f;
volatile float g_bench_vy = 0.0f;
volatile float g_bench_w = 0.0f;
volatile float g_bench_open_duty = 0.0f;

/* ===== 台架模式的开关：运行期可写，不用重编译 =====
 * 为什么不再用 constexpr：改常量就得重编重烧，一次试验两分钟，还容易和板上固件错位。
 * 现在的约定：
 *   上电默认 0        → 台架模式关闭，rc/peer/imu/motors/estop 五个联锁全部有效
 *   写 0x5A5A5A5A     → 打开；写别的值（包括手滑写的 1）一律不生效
 *   复位/掉电自动恢复 → 不会"忘了关"
 * ⚠️ 打开后车必须架空：它会绕过全部五个安全前提。 */
volatile uint32_t g_bench_enable = 0u;
constexpr uint32_t kBenchMagic = 0x5A5A5A5Au;

namespace
{
/** 台架模式是否开启：魔数匹配才认，避免手滑写个 1 就把五个联锁全绕过了 */
bool BenchOn(void) { return g_bench_enable == kBenchMagic; }
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
  /* ---------- 台架模式：绕过遥控 ----------
   * DBUS 解码还没修好（rc 恒 0），底盘会卡在 Dead、GenerateCommands 永不执行，
   * 轮子完全动不了。打开台架模式后强制 rc 在线、清除急停，
   * 让状态机能进 Working，速度指令改由 g_bench_* 提供（默认 0，上电不动）。
   * ⚠️ 开关是运行期的 g_bench_enable（写魔数才生效），复位自动关闭。 */
  const bool bench = BenchOn();
  if (bench) {
    rc_online = true;
    estop_switch = false;
  } else if (rc_ != nullptr) {
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
  const bool peer_online_raw = (comm_ != nullptr) && (!comm_->isOffline());
  delta_ = peer_online_raw ? comm_->gimbal_data().yaw : 0.0f;

  /* 台架模式：轮子测试用不到 IMU，一并绕过（正式使用必须关掉台架模式） */
  inputs_.imu_ready = bench ? true : imu_ready;
  /* 台架模式：放开「四个电调都有反馈」这个总闸，让能工作的轮子先转起来，
   * 便于定位到底是哪一个电调没反馈（单个轮子的保护仍由 OutputWheels 的看门狗负责）。 */
  inputs_.motors_ready = bench ? true : motors_ok;
  inputs_.rc_online = rc_online;
  /* 台架模式：云台板可能没通电/没连，peer 也要绕过，否则永远进不了 Working。 */
  inputs_.peer_online = bench ? true : peer_online_raw;
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

  /* ---------- 台架模式 ----------
   * 不看遥控、不看模式，直接用三个可在线写入的量驱动轮子，便于验证
   * 「逆解 / 速度环 / 电调」这些与遥控无关的部分。默认全 0，上电不会动。
   *
   * ⚠️ 这里必须把指令真正推进底盘模块：Chassis::Update() 只认 SetCommand()
   *    存进去的 command_。曾经漏了这一句就直接 return，于是 wheel_vel 恒 0
   *    → 速度环目标恒 0 → 四个轮子一动不动。这就是 2026-11-16 那次
   *    「五个前提全强制为真、轮子仍不转」的真正原因（不是电调、不是接线）。 */
  const bool bench = BenchOn();
  if (bench) {
    command_.vx = g_bench_vx * lim.max_vx;
    command_.vy = g_bench_vy * lim.max_vy;
    command_.w = g_bench_w * lim.max_w;
    chassis_->SetCommand(command_, delta_);
    return;
  }

  /* 左摇杆：竖 → 前后，横 → 左右（右推为 −y，因为 y 轴指向左方）
   * 所有摇杆量先过死区，保证中位抖动不会变成残余速度（零输入即零输出） */
  const float lv_dz = ApplyStickDeadzone(rc_->rc_lv());
  const float lh_dz = ApplyStickDeadzone(rc_->rc_lh());
  const float rh_dz = ApplyStickDeadzone(rc_->rc_rh());

  /* ---------- 上电武装互锁 ----------
   * 实测踩过坑：电调恢复供电的瞬间底盘立刻进闭环，如果此时手上还压着摇杆
   * （或刚做完云台测试），车会直接窜出去。所以要求三轴摇杆全部回中并保持
   * kArmHoldMs 之后才允许输出；期间指令恒零。
   * 急停 / 退出 Working 时 arm_ms_ 会被清零（见 OutputZero），下次进 Working 需重新武装。 */
  constexpr float kArmHoldMs = 200.0f;
  if ((lv_dz == 0.0f) && (lh_dz == 0.0f) && (rh_dz == 0.0f)) {
    arm_ms_ += dt * 1000.0f;
  } else {
    arm_ms_ = 0.0f;
  }
  if (arm_ms_ < kArmHoldMs) {
    command_.setZero();
    return;
  }

  const float vx_cmd = lv_dz * lim.max_vx;
  const float vy_cmd = -lh_dz * lim.max_vy;
  /* 右摇杆横向 → 原地转向（右推为顺时针，即负 w） */
  const float w_stick = -rh_dz * lim.max_w;

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

  /* ---------- 开环占空比测试：绕过速度环，四轮直给同一个电流 ----------
   * 目的：把「0x200 电流指令 → 电调 → 轮子转」这条链单独拎出来验证，
   *      不和 PID、逆解、反馈混杂。判定树：
   *        开环能转 → 问题在 PID/指令链（回去查 g_bench_vx 那条路径）
   *        开环不转 → 看 g_can_tx_fail（发送层）→ 看电调小灯/24V（硬件层）
   * 只在台架模式（g_bench_enable 魔数）且 duty 非 0 时生效；车必须架空。
   * duty ∈ [-1,1] → 电流 raw ∈ [-16384, 16384]，正负号即转向。 */
  if (BenchOn() && (g_bench_open_duty != 0.0f)) {
    const float duty = Clamp(g_bench_open_duty, -1.0f, 1.0f);
    const float raw = duty * 16384.0f;
    for (uint8_t i = 0; i < kWheelCount; ++i) {
      if (motors_[i] != nullptr) {
        (void)motors_[i]->setInput(raw);
      }
    }
    return;
  }

  const float radius = chassis_->ik_solver().geometry().wheel_radius;
  const hello_world::chassis_ik_solver::WheelVel& wheel = chassis_->wheel_vel();
  const uint32_t now_ms = hello_world::tick::GetTickMs();

  for (uint8_t i = 0; i < kWheelCount; ++i) {
    if ((motors_[i] == nullptr) || (wheel_pids_[i] == nullptr)) {
      continue;
    }
    /* 反馈看门狗：电调没反馈时 vel() 恒为 0，速度环会一路涨积分
     * （现场实测 out_ 涨到 ±3500 还在涨），等电调恢复的瞬间四个轮子会猛冲。
     * 所以反馈不新鲜就清积分、输出 0，等反馈恢复再重新入环。 */
    if (!motors_[i]->IsFeedbackFresh(now_ms, kMotorFeedbackTimeoutMs)) {
      wheel_pids_[i]->Reset();
      motors_[i]->setInput(0.0f);
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
  /* 退出闭环就解除武装：再次进入 Working 必须重新「摇杆回中」才允许输出 */
  arm_ms_ = 0.0f;
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
