/**
 *******************************************************************************
 * @file      : main.cpp
 * @brief     : 无硬件自测：把「纯逻辑」的那部分代码全部验证一遍
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 覆盖 角度/坐标变换/状态机/PID/逆解/双板协议/零漂
 *******************************************************************************
 * @attention :
 *  这些用例就是「不需要硬件的那 38 项」的自动化验收：
 *    - 逆解与坐标变换算错 → 上板只会看到车乱跑
 *    - 双板编解码不对称 → 两块板互相读到垃圾
 *    - 状态机漏了急停分支 → 一票否决
 *  全绿之后再上板，剩下的就只是「填参数 + 调 PID」。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bench_test.hpp"
#include "chassis_ik_solver.hpp"
#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu_bias.hpp"
#include "module_state.hpp"
#include "pid.hpp"

namespace hello_world
{
namespace tick
{
void AdvanceMs(uint32_t ms);
void ResetMs(void);
}  // namespace tick
}  // namespace hello_world

/* ------------------------------- 测试框架 -------------------------------- */
namespace
{
int g_failures = 0;
int g_checks = 0;

#define CHECK_TRUE(cond)                                                        \
  do {                                                                          \
    ++g_checks;                                                                 \
    if (!(cond)) {                                                              \
      ++g_failures;                                                             \
      std::printf("  [FAIL] %s:%d  %s\n", __FILE__, __LINE__, #cond);           \
    }                                                                           \
  } while (0)

#define CHECK_NEAR(actual, expected, tol)                                       \
  do {                                                                          \
    ++g_checks;                                                                 \
    const double a_ = static_cast<double>(actual);                              \
    const double e_ = static_cast<double>(expected);                            \
    if (std::fabs(a_ - e_) > static_cast<double>(tol)) {                        \
      ++g_failures;                                                             \
      std::printf("  [FAIL] %s:%d  %s = %.6f, 期望 %.6f (tol %.6f)\n",          \
                  __FILE__, __LINE__, #actual, a_, e_,                         \
                  static_cast<double>(tol));                                   \
    }                                                                           \
  } while (0)

void Section(const char* name)
{
  std::printf("\n== %s ==\n", name);
}

/* ==================== 0. 摇杆死区（零输入即零输出） ==================== */
void TestStickDeadzone()
{
  Section("摇杆死区");
  using robot::ApplyStickDeadzone;

  /* 死区内必须是严格的 0：这是「松手不爬行」的硬指标 */
  CHECK_NEAR(ApplyStickDeadzone(0.0f), 0.0f, 1e-9);
  CHECK_NEAR(ApplyStickDeadzone(0.02f), 0.0f, 1e-9);
  CHECK_NEAR(ApplyStickDeadzone(-0.02f), 0.0f, 1e-9);
  CHECK_NEAR(ApplyStickDeadzone(0.05f), 0.0f, 1e-9);   /* 边界 */
  CHECK_NEAR(ApplyStickDeadzone(-0.05f), 0.0f, 1e-9);

  /* 满量程必须还原成 ±1，不能因为死区损失行程 */
  CHECK_NEAR(ApplyStickDeadzone(1.0f), 1.0f, 1e-6);
  CHECK_NEAR(ApplyStickDeadzone(-1.0f), -1.0f, 1e-6);

  /* 死区外线性重映射：(0.05+0.95*0.5) 应映射到 0.5 */
  CHECK_NEAR(ApplyStickDeadzone(0.525f), 0.5f, 1e-5);
  CHECK_NEAR(ApplyStickDeadzone(-0.525f), -0.5f, 1e-5);

  /* 单调性：摇杆推得越多，输出不能变小 */
  float prev = -1.0f;
  for (int i = 0; i <= 20; ++i) {
    const float in = -1.0f + 0.1f * static_cast<float>(i);
    const float out = ApplyStickDeadzone(in);
    CHECK_TRUE(out >= prev - 1e-6f);
    prev = out;
  }

  /* 死区边界：deadzone >= 1 时不能除零/变号 */
  CHECK_NEAR(ApplyStickDeadzone(1.0f, 1.0f), 1.0f, 1e-3);
  CHECK_NEAR(ApplyStickDeadzone(-1.0f, 1.0f), -1.0f, 1e-3);
  CHECK_NEAR(ApplyStickDeadzone(0.0f, 2.0f), 0.0f, 1e-6);
  CHECK_TRUE(std::isfinite(ApplyStickDeadzone(0.5f, 5.0f)));
  /* 超出量程要饱和，不能放大 */
  CHECK_NEAR(ApplyStickDeadzone(2.0f), 1.0f, 1e-6);
  CHECK_NEAR(ApplyStickDeadzone(-2.0f), -1.0f, 1e-6);

  /* 对称性 */
  for (int i = 1; i <= 10; ++i) {
    const float in = 0.1f * static_cast<float>(i);
    CHECK_NEAR(ApplyStickDeadzone(-in), -ApplyStickDeadzone(in), 1e-6);
  }
}

/* ======================= 1. 角度工具（3.2 / 3.5） ======================= */
void TestAngleTools()
{
  Section("角度归一化 / 角度差");
  using robot::kPi;
  using robot::kTwoPi;
  using robot::AngleDiff;
  using robot::WrapToPi;

  CHECK_NEAR(WrapToPi(0.0f), 0.0f, 1e-6);
  CHECK_NEAR(WrapToPi(1.5f), 1.5f, 1e-6);
  CHECK_NEAR(WrapToPi(-1.5f), -1.5f, 1e-6);
  /* 2π - 0.03 应当等价于 -0.03 */
  CHECK_NEAR(WrapToPi(kTwoPi - 0.03f), -0.03f, 1e-5);
  CHECK_NEAR(WrapToPi(-kTwoPi + 0.03f), 0.03f, 1e-5);
  /* 多圈也要能折叠 */
  CHECK_NEAR(WrapToPi(3.0f * kTwoPi + 0.5f), 0.5f, 1e-4);

  /* 核心用例：359° → 1°，必须当成 +2°，而不是 -358° */
  const float deg_359 = 359.0f * kPi / 180.0f;
  const float deg_1 = 1.0f * kPi / 180.0f;
  CHECK_NEAR(AngleDiff(deg_1, deg_359), 2.0f * kPi / 180.0f, 1e-5);

  /* 限幅 */
  CHECK_NEAR(robot::Clamp(5.0f, -1.0f, 1.0f), 1.0f, 0.0f);
  CHECK_NEAR(robot::Clamp(-5.0f, -1.0f, 1.0f), -1.0f, 0.0f);
}

/* ============ 2. 云台系 → 车体系 旋转（考核说明 3.2 原式） ============ */
void TestFrameRotation()
{
  Section("云台系 → 车体系 旋转");
  using robot::ChassisVel;
  using robot::GimbalToChassis;
  using robot::kPi;

  /* δ = 0：平移不变 */
  ChassisVel v(1.0f, 2.0f, 0.5f);
  ChassisVel out = GimbalToChassis(v, 0.0f);
  CHECK_NEAR(out.vx, 1.0f, 1e-6);
  CHECK_NEAR(out.vy, 2.0f, 1e-6);
  CHECK_NEAR(out.w, 0.5f, 1e-6);

  /* δ = 90°：(1, 0) → (0, -1)，即文档 3.2 的
   *   vx_c = vx·cosδ + vy·sinδ
   *   vy_c = -vx·sinδ + vy·cosδ */
  out = GimbalToChassis(ChassisVel(1.0f, 0.0f, 0.0f), kPi * 0.5f);
  CHECK_NEAR(out.vx, 0.0f, 1e-6);
  CHECK_NEAR(out.vy, -1.0f, 1e-6);

  /* 模长不变（纯旋转） */
  ChassisVel v2(0.8f, -0.6f, 0.0f);
  for (int i = 0; i < 8; ++i) {
    const float delta = static_cast<float>(i) * 0.37f - 1.2f;
    const ChassisVel r = GimbalToChassis(v2, delta);
    const float len_in = std::sqrt(v2.vx * v2.vx + v2.vy * v2.vy);
    const float len_out = std::sqrt(r.vx * r.vx + r.vy * r.vy);
    CHECK_NEAR(len_out, len_in, 1e-5);
  }

  /* 输入角度超圈也要能用 */
  out = GimbalToChassis(ChassisVel(1.0f, 0.0f, 0.0f), kPi * 0.5f + 2.0f * robot::kTwoPi);
  CHECK_NEAR(out.vx, 0.0f, 1e-5);
  CHECK_NEAR(out.vy, -1.0f, 1e-5);
}

/* ================== 3. 麦轮逆解 + 等比限幅（任务 6） ================== */
void TestKinematics()
{
  Section("麦轮逆解");
  using hello_world::chassis_ik_solver::ChassisGeometry;
  using hello_world::chassis_ik_solver::ChassisIkSolver;
  using robot::ChassisVel;

  ChassisGeometry geo;
  geo.wheel_radius = 0.076f;
  geo.half_wheelbase = 0.20f;
  geo.half_track = 0.20f;
  geo.max_wheel_vel = 100.0f; /* 先放开限幅，验纯逆解 */

  ChassisIkSolver solver;
  solver.Init(geo);
  CHECK_NEAR(solver.lever(), 0.40f, 1e-6);

  /* 零输入 → 四轮全零 */
  auto w0 = solver.SolveChassisFrame(ChassisVel(0.0f, 0.0f, 0.0f));
  for (int i = 0; i < 4; ++i) {
    CHECK_NEAR(w0.v[i], 0.0f, 1e-9);
  }

  /* 纯前进 → 四轮同速 */
  auto w1 = solver.SolveChassisFrame(ChassisVel(1.0f, 0.0f, 0.0f));
  for (int i = 0; i < 4; ++i) {
    CHECK_NEAR(w1.v[i], 1.0f, 1e-6);
  }

  /* 纯自旋 → LF/LB 负、RB/RF 正（符号交替） */
  auto w2 = solver.SolveChassisFrame(ChassisVel(0.0f, 0.0f, 1.0f));
  CHECK_NEAR(w2.v[0], -0.4f, 1e-6); /* LF */
  CHECK_NEAR(w2.v[1], -0.4f, 1e-6); /* LB */
  CHECK_NEAR(w2.v[2], 0.4f, 1e-6);  /* RB */
  CHECK_NEAR(w2.v[3], 0.4f, 1e-6);  /* RF */

  /* 纯左移 vy = 1 → LF/LB 反号，RB/RF 同号 */
  auto w3 = solver.SolveChassisFrame(ChassisVel(0.0f, 1.0f, 0.0f));
  CHECK_NEAR(w3.v[0], -1.0f, 1e-6);
  CHECK_NEAR(w3.v[1], 1.0f, 1e-6);
  CHECK_NEAR(w3.v[2], -1.0f, 1e-6);
  CHECK_NEAR(w3.v[3], 1.0f, 1e-6);

  /* δ = 90° 下的「前进」= 车体系的 -y，逆解必须与手工结果一致 */
  auto w4 = solver.Solve(ChassisVel(1.0f, 0.0f, 0.0f), robot::kPi * 0.5f);
  CHECK_NEAR(w4.v[0], 1.0f, 1e-5);  /* LF = vx - vy = 0 - (-1) */
  CHECK_NEAR(w4.v[1], -1.0f, 1e-5); /* LB = vx + vy */
  CHECK_NEAR(w4.v[2], 1.0f, 1e-5);  /* RB = vx - vy */
  CHECK_NEAR(w4.v[3], -1.0f, 1e-5); /* RF = vx + vy */

  /* 限幅：超速时四轮等比缩放，方向不能歪 */
  geo.max_wheel_vel = 1.0f;
  solver.Init(geo);
  auto w5 = solver.SolveAndLimit(ChassisVel(10.0f, 0.0f, 0.0f), 0.0f);
  for (int i = 0; i < 4; ++i) {
    CHECK_NEAR(w5.v[i], 1.0f, 1e-6);
  }
  /* 等比缩放：LF 与 RB 的比值必须保持不变 */
  auto raw = solver.SolveChassisFrame(ChassisVel(3.0f, 0.0f, 1.0f));
  auto lim = solver.ApplyLimit(raw);
  const float ratio_raw = raw.v[0] / raw.v[2];
  const float ratio_lim = lim.v[0] / lim.v[2];
  CHECK_NEAR(ratio_lim, ratio_raw, 1e-5);
  CHECK_TRUE(lim.maxAbs() <= 1.0f + 1e-6f);
}

/* ============ 4. 状态机三态迁移 + 急停（任务 9） ============ */
void TestStateMachine()
{
  Section("状态机三态迁移");
  using robot::EstopActive;
  using robot::LifeInputs;
  using robot::LifeState;
  using robot::NextLifeState;

  LifeInputs in;
  in.imu_ready = true;
  in.motors_ready = true;
  in.rc_online = true;
  in.peer_online = true;

  /* 全就绪：Dead → Resurrection → Working → 保持 */
  CHECK_TRUE(NextLifeState(LifeState::kDead, in) == LifeState::kResurrection);
  CHECK_TRUE(NextLifeState(LifeState::kResurrection, in) == LifeState::kWorking);
  CHECK_TRUE(NextLifeState(LifeState::kWorking, in) == LifeState::kWorking);

  /* 任一前提丢失：Working 立刻回 Dead */
  LifeInputs bad = in;
  bad.imu_ready = false;
  CHECK_TRUE(NextLifeState(LifeState::kWorking, bad) == LifeState::kDead);

  bad = in;
  bad.peer_online = false;
  CHECK_TRUE(NextLifeState(LifeState::kWorking, bad) == LifeState::kDead);

  bad = in;
  bad.rc_online = false;
  CHECK_TRUE(NextLifeState(LifeState::kWorking, bad) == LifeState::kDead);

  /* 急停档：不能进 Working，且 EstopActive 必须为真 */
  LifeInputs estop = in;
  estop.estop = true;
  CHECK_TRUE(!estop.allReady());
  CHECK_TRUE(EstopActive(estop));
  CHECK_TRUE(NextLifeState(LifeState::kWorking, estop) == LifeState::kDead);

  /* 遥控器离线也算「失效即归零」 */
  LifeInputs rc_lost = in;
  rc_lost.rc_online = false;
  CHECK_TRUE(EstopActive(rc_lost));

  /* ===== 死锁回归测试 =====
   * 真实故障：pitch 电机坏了 → motors_ready 永远 0。
   * 如果 Dead→Resurrection 也要求 motors_ready，超时兜底（只在 Resurrection 里生效）
   * 就永远不会被触发，状态机永久卡在 Dead。 */
  LifeInputs no_motor = in;
  no_motor.motors_ready = false;
  CHECK_TRUE(NextLifeState(LifeState::kDead, no_motor) == LifeState::kResurrection);
  CHECK_TRUE(NextLifeState(LifeState::kResurrection, no_motor) == LifeState::kResurrection);
  CHECK_TRUE(NextLifeState(LifeState::kWorking, no_motor) == LifeState::kDead);
  /* 但前提里的其它项丢失时，仍必须回 Dead */
  LifeInputs no_motor_no_rc = no_motor;
  no_motor_no_rc.rc_online = false;
  CHECK_TRUE(NextLifeState(LifeState::kResurrection, no_motor_no_rc) == LifeState::kDead);

  /* Dead 状态下条件不全就一直留在 Dead */
  LifeInputs nothing;
  CHECK_TRUE(NextLifeState(LifeState::kDead, nothing) == LifeState::kDead);
}

/* ======================= 5. PID（任务 3） ======================= */
/* ========== 4b. 电机就绪判定（超时兜底不能依赖状态机状态） ========== */
void TestMotorsReady()
{
  Section("电机就绪判定");
  using robot::MotorsReady;
  const uint32_t kTo = 2000u;

  /* 两轴都使能 → 立即就绪 */
  CHECK_TRUE(MotorsReady(true, true, true, 100u, 0u, kTo));
  /* 只有 yaw，且要求两轴：超时前不就绪 */
  CHECK_TRUE(!MotorsReady(true, false, true, 1000u, 500u, kTo));
  /* 满超时后就绪 */
  CHECK_TRUE(MotorsReady(true, false, true, 2501u, 500u, kTo));

  /* yaw 都没使能 → 永远不就绪（安全：不能因为超时就放行全灭的电调） */
  CHECK_TRUE(!MotorsReady(false, true, true, 99999u, 500u, kTo));
  CHECK_TRUE(!MotorsReady(false, false, true, 99999u, 500u, kTo));

  /* 单轴调试模式：只要 yaw */
  CHECK_TRUE(MotorsReady(true, false, false, 100u, 0u, kTo));
  CHECK_TRUE(!MotorsReady(false, false, false, 99999u, 500u, kTo));

  /* 上电时刻还没记录时不放行 */
  CHECK_TRUE(!MotorsReady(true, false, true, 99999u, 0u, kTo));

  /* 关键回归：进入 Working 之后（now 继续增长）必须仍然成立，
   * 否则会出现「2 秒复活 → 一拍 Working → 又被打回 Dead」的循环。 */
  CHECK_TRUE(MotorsReady(true, false, true, 5000u, 500u, kTo));
  CHECK_TRUE(MotorsReady(true, false, true, 60000u, 500u, kTo));
  CHECK_TRUE(MotorsReady(true, false, true, 60000u, 59000u, kTo) == false);
}

/* ========== 4c. DBUS 帧有效性（掉线时不能当成满杆指令） ========== */
void TestDbusFrameValid()
{
  Section("DBUS 帧有效性");
  using robot::IsDbusFrameValid;

  /* 真实的合法帧（拨杆 1/2，摇杆在中位附近） */
  const uint8_t good[18] = {0x30, 0xA4, 0x34, 0x00, 0x29, 0x6D, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  CHECK_TRUE(IsDbusFrameValid(good, 18));

  /* 接收机掉线：缓冲区全 0 —— 这就是现场"云台自己匀速转"的元凶 */
  const uint8_t zeros[18] = {0};
  CHECK_TRUE(!IsDbusFrameValid(zeros, 18));

  /* 全 FF */
  uint8_t ones[18];
  for (int i = 0; i < 18; ++i) {
    ones[i] = 0xFFu;
  }
  CHECK_TRUE(!IsDbusFrameValid(ones, 18));

  /* 长度不够 */
  CHECK_TRUE(!IsDbusFrameValid(good, 17));

  /* 空指针 */
  CHECK_TRUE(!IsDbusFrameValid(nullptr, 18));

  /* 拨杆位非法（0 不是合法档位） */
  uint8_t bad_sw[18];
  std::memcpy(bad_sw, good, 18);
  bad_sw[5] = 0x00u; /* 左=0 右=0 */
  CHECK_TRUE(!IsDbusFrameValid(bad_sw, 18));

  /* 拨杆只有左非法 */
  std::memcpy(bad_sw, good, 18);
  bad_sw[5] = static_cast<uint8_t>(0x00u | (good[5] & 0x0Fu)); /* 左=0，右保持 */
  CHECK_TRUE(!IsDbusFrameValid(bad_sw, 18));

  /* 合法档位 1/2/3 的各种组合都应通过 */
  for (uint8_t l = 1; l <= 3; ++l) {
    for (uint8_t r = 1; r <= 3; ++r) {
      uint8_t f[18];
      std::memcpy(f, good, 18);
      f[5] = static_cast<uint8_t>((static_cast<uint8_t>(l << 2) | r) << 4);
      CHECK_TRUE(IsDbusFrameValid(f, 18));
    }
  }
}

/* ========== 4d. 台架自测方波（不依赖遥控器时用） ========== */
void TestBenchSquare()
{
  Section("台架自测方波");
  using hello_world::bench::Config;
  using hello_world::bench::SquareTarget;

  Config c;
  c.amplitude = 0.12f;
  c.half_period_ms = 1000u;
  c.total_ms = 5000u;

  CHECK_NEAR(SquareTarget(0u, c), 0.12f, 1e-6);
  CHECK_NEAR(SquareTarget(999u, c), 0.12f, 1e-6);
  CHECK_NEAR(SquareTarget(1000u, c), -0.12f, 1e-6);
  CHECK_NEAR(SquareTarget(1500u, c), -0.12f, 1e-6);
  CHECK_NEAR(SquareTarget(2500u, c), 0.12f, 1e-6);  /* k=2 偶数 → +A */

  /* 超过总时长必须回中位（自动停止） */
  CHECK_NEAR(SquareTarget(5000u, c), 0.0f, 1e-6);
  CHECK_NEAR(SquareTarget(99999u, c), 0.0f, 1e-6);

  /* 半周期为 0 时不产生运动（防除零） */
  Config bad;
  bad.half_period_ms = 0u;
  CHECK_NEAR(SquareTarget(100u, bad), 0.0f, 1e-6);

  /* 幅度始终不超过设定值 */
  for (uint32_t tt = 0u; tt < 5000u; tt += 37u) {
    CHECK_TRUE(std::fabs(SquareTarget(tt, c)) <= c.amplitude + 1e-6f);
  }
}
void TestPid()
{
  Section("PID");
  using hello_world::pid::Pid;

  Pid::Config cfg;
  cfg.kp = 2.0f;
  cfg.ki = 0.0f;
  cfg.kd = 0.0f;
  cfg.out_min = -10.0f;
  cfg.out_max = 10.0f;
  cfg.int_min = -5.0f;
  cfg.int_max = 5.0f;

  Pid pid(cfg);
  /* 被控对象：x' = u，目标 1 */
  float x = 0.0f;
  const float dt = 0.001f;
  for (int i = 0; i < 5000; ++i) {
    const float u = pid.Update(1.0f, x, dt);
    x += u * dt;
  }
  CHECK_NEAR(x, 1.0f, 0.02f); /* 纯 P 收敛到 1 */

  /* 输出限幅必须生效 */
  Pid::Config big = cfg;
  big.kp = 1000.0f;
  Pid sat(big);
  const float out = sat.Update(100.0f, 0.0f, dt);
  CHECK_NEAR(out, 10.0f, 1e-6);
  CHECK_TRUE(sat.saturated());

  /* 积分限幅：长时间存在误差也不能把积分撑爆 */
  Pid::Config integ;
  integ.kp = 0.0f;
  integ.ki = 100.0f;
  integ.out_min = -100.0f;
  integ.out_max = 100.0f;
  integ.int_min = -1.0f;
  integ.int_max = 1.0f;
  Pid ipid(integ);
  for (int i = 0; i < 10000; ++i) {
    (void)ipid.Update(1.0f, 0.0f, dt);
  }
  CHECK_TRUE(ipid.integral() <= 1.0f + 1e-6f);
  CHECK_TRUE(ipid.integral() >= -1.0f - 1e-6f);

  /* Reset 之后状态归零，防止状态切换时残留 */
  sat.Reset();
  CHECK_NEAR(sat.out(), 0.0f, 1e-9);
  CHECK_NEAR(sat.integral(), 0.0f, 1e-9);

  /* dt <= 0 时不得更新状态 */
  const float hold = sat.Update(1.0f, 0.0f, 0.0f);
  CHECK_NEAR(hold, 0.0f, 1e-9);
}

/* ============ 6. 双板通信编解码 / 离线判断（任务 8） ============ */
/* ============ 8. 云台 yaw 必须闭环在【绝对角】（核心设计点） ============ */
void TestGimbalAbsoluteHold()
{
  Section("云台 yaw 反馈：绝对角 vs 关节角");
  using hello_world::pid::Pid;
  using robot::Gimbal;
  using robot::GimbalLimits;
  using robot::GimbalPos;

  /* 和实车一致的参数 */
  Pid::Config cfg;
  cfg.kp = 2.0f;
  cfg.ki = 0.0f;
  cfg.kd = 0.35f;
  cfg.out_min = -3.0f;
  cfg.out_max = 3.0f;
  cfg.int_min = -1.0f;
  cfg.int_max = 1.0f;

  GimbalLimits lim;
  lim.pitch_min = -0.35f;
  lim.pitch_max = 0.60f;
  lim.max_yaw_rate = 6.0f;
  lim.max_pitch_rate = 3.0f;
  lim.max_torque = 3.0f;

  const float dt = 0.001f;
  const float chassis_rate = 1.0f;   /* 车体以 1 rad/s 自旋 */
  const int steps = 1000;            /* 模拟 1 秒 */
  const float plant_gain = 0.5f;     /* 简化一阶模型：关节角速度 = 0.5 × 转矩 */

  /* --- 方案 A：反馈 = 关节角（旧实现，有缺陷） --- */
  float joint_a = 0.0f;
  float abs_a = 0.0f;
  float err_a = 0.0f;
  {
    Gimbal g;
    g.Init(lim);
    g.yaw_pid().Init(cfg);
    g.SetTarget(GimbalPos(0.0f, 0.0f));
    for (int i = 0; i < steps; ++i) {
      const float chassis = chassis_rate * dt * static_cast<float>(i + 1);
      abs_a = chassis + joint_a;
      g.Update(dt, joint_a, 0.0f); /* 旧：拿关节角当反馈 */
      joint_a += g.output().yaw * plant_gain * dt;
      err_a = g.error().yaw;
    }
  }

  /* --- 方案 B：反馈 = IMU 绝对角（新实现） --- */
  float joint_b = 0.0f;
  float abs_b = 0.0f;
  float err_b = 0.0f;
  {
    Gimbal g;
    g.Init(lim);
    g.yaw_pid().Init(cfg);
    g.SetTarget(GimbalPos(0.0f, 0.0f));
    for (int i = 0; i < steps; ++i) {
      const float chassis = chassis_rate * dt * static_cast<float>(i + 1);
      abs_b = chassis + joint_b;
      g.Update(dt, abs_b, 0.0f); /* 新：拿绝对角当反馈 */
      joint_b += g.output().yaw * plant_gain * dt;
      err_b = g.error().yaw;
    }
  }

  /* 旧方案：控制器「以为」自己没偏（误差恒 0），可实际指向已经跟着车转了 1 rad。
   * 这正是「跟随追不上、小陀螺枪口乱转」的根因。 */
  CHECK_NEAR(err_a, 0.0f, 0.01f);
  CHECK_NEAR(abs_a, 1.0f, 0.02f);

  /* 新方案：控制器看见了偏差，输出反向转矩，关节角明显为负（在反抗车体旋转） */
  CHECK_TRUE(err_b < -0.02f);
  CHECK_TRUE(joint_b < -0.02f);

  /* 关键：新方案下云台的实际指向偏离明显更小 */
  CHECK_TRUE(std::fabs(abs_b) < std::fabs(abs_a));
  CHECK_TRUE(std::fabs(abs_b) < 0.8f);
}

void TestDualBoardComm()
{
  Section("双板通信 8 字节编解码");
  using robot::ChassisData;
  using robot::GimbalChassisComm;
  using robot::GimbalData;
  using robot::kCommFrameLen;

  hello_world::tick::ResetMs();

  GimbalChassisComm chassis_side(GimbalChassisComm::CodePart::Chassis,
                                 robot::kCanIdChassisToGimbal,
                                 robot::kCanIdGimbalToChassis);
  GimbalChassisComm gimbal_side(GimbalChassisComm::CodePart::Gimbal,
                                robot::kCanIdGimbalToChassis,
                                robot::kCanIdChassisToGimbal);

  /* --- C2G：底盘发、云台收 --- */
  ChassisData& c = chassis_side.chassis_data();
  c.online = true;
  c.ready = true;
  c.estop = false;
  c.mode = robot::ChassisMode::kSpin;
  c.gimbal_mode = robot::GimbalMode::kJointSpace;
  c.yaw_des = 1.234f;
  c.pitch_des = -0.321f;
  c.w_des = 3.5f;
  c.seq = 42u;

  uint8_t frame[kCommFrameLen] = {0u};
  chassis_side.encode(frame);
  CHECK_TRUE(frame[7] == 42u);

  gimbal_side.decode(frame);
  const ChassisData& received = gimbal_side.chassis_data();
  CHECK_TRUE(received.online);
  CHECK_TRUE(received.ready);
  CHECK_TRUE(!received.estop);
  CHECK_TRUE(received.mode == robot::ChassisMode::kSpin);
  /* int16 缩放 1000 → 量化误差 ≤ 0.001 rad */
  CHECK_NEAR(received.yaw_des, 1.234f, 0.001f);
  CHECK_NEAR(received.pitch_des, -0.321f, 0.001f);
  CHECK_NEAR(received.w_des, 3.5f, 0.06f); /* int8 缩放 20 → 步进 0.05 */
  CHECK_TRUE(received.seq == 42u);
  CHECK_TRUE(gimbal_side.rx_count() == 1u);

  /* --- G2C：云台发、底盘收 --- */
  GimbalData& g = gimbal_side.gimbal_data();
  g.online = true;
  g.ready = true;
  g.estop = true;
  g.mode = robot::GimbalMode::kJointSpace;
  g.yaw = -2.5f;
  g.pitch = 0.45f;
  g.yaw_vel = -1.2f;
  g.seq = 7u;

  uint8_t frame2[kCommFrameLen] = {0u};
  gimbal_side.encode(frame2);
  chassis_side.decode(frame2);

  const GimbalData& got = chassis_side.gimbal_data();
  CHECK_TRUE(got.online);
  CHECK_TRUE(got.estop);
  CHECK_NEAR(got.yaw, -2.5f, 0.001f);
  CHECK_NEAR(got.pitch, 0.45f, 0.001f);
  CHECK_NEAR(got.yaw_vel, -1.2f, 0.06f);
  CHECK_TRUE(got.seq == 7u);

  /* --- 饱和：超出 int16 量程必须被钳住，不能回绕成反号 --- */
  c.yaw_des = 100.0f;
  chassis_side.encode(frame);
  gimbal_side.decode(frame);
  CHECK_TRUE(gimbal_side.chassis_data().yaw_des > 30.0f);

  /* --- 离线判断：超过 100ms 没新帧就离线 --- */
  hello_world::tick::ResetMs();
  gimbal_side.decode(frame); /* 刷新时间戳 */
  CHECK_TRUE(!gimbal_side.isOffline());
  hello_world::tick::AdvanceMs(50u);
  CHECK_TRUE(!gimbal_side.isOffline());
  hello_world::tick::AdvanceMs(80u); /* 累计 130ms */
  CHECK_TRUE(gimbal_side.isOffline());

  /* --- 两板 ID 必须互补 --- */
  CHECK_TRUE(chassis_side.tx_id() == gimbal_side.rx_id());
  CHECK_TRUE(chassis_side.rx_id() == gimbal_side.tx_id());
  CHECK_TRUE(robot::kCanIdChassisToGimbal != robot::kCanIdGimbalToChassis);
}

/* ============ 7. 陀螺零漂收敛判定（任务 4 的纯逻辑部分） ============ */
void TestGyroBias()
{
  Section("陀螺零漂收敛");
  using hello_world::imu::GyroBiasEstimator;

  /* 静止 + 微小噪声 → 应当收敛 */
  GyroBiasEstimator est;
  est.Init(1000.0f, 0.2f, 0.01f, 1u); /* 200 个样本，阈值 0.01 rad/s */
  bool converged = false;
  for (int i = 0; i < 1000 && !converged; ++i) {
    const float noise = ((i % 2) == 0) ? 0.002f : -0.002f; /* 峰值 0.002 */
    const float gyro[3] = {0.01f + noise, -0.02f + noise, 0.03f + noise};
    converged = est.Update(gyro);
  }
  CHECK_TRUE(converged);
  CHECK_TRUE(est.converged());
  CHECK_TRUE(!est.failed());
  CHECK_NEAR(est.bias(0), 0.01f, 0.005f);
  CHECK_NEAR(est.bias(1), -0.02f, 0.005f);
  CHECK_NEAR(est.bias(2), 0.03f, 0.005f);

  /* 上电时车在动 → 不能收敛（标准差超阈值），并且不能卡在「假装收敛」 */
  GyroBiasEstimator moving;
  moving.Init(1000.0f, 0.1f, 0.01f, 1u);
  for (int i = 0; i < 1000 && !moving.converged() && !moving.failed(); ++i) {
    const float gyro[3] = {((i % 2) == 0) ? 1.0f : -1.0f, 0.0f, 0.0f};
    (void)moving.Update(gyro);
  }
  CHECK_TRUE(!moving.converged());

  /* 进度必须单调可用（开机自检指示靠它） */
  GyroBiasEstimator prog;
  prog.Init(1000.0f, 1.0f, 0.01f);
  const float zero[3] = {0.0f, 0.0f, 0.0f};
  (void)prog.Update(zero);
  CHECK_NEAR(prog.progress(), 0.001f, 1e-6f);
}

}  // namespace

int main()
{
  std::printf("========================================\n");
  std::printf(" RoboMaster 电控考核 · 无硬件自测\n");
  std::printf("========================================\n");

  TestStickDeadzone();
  TestAngleTools();
  TestFrameRotation();
  TestKinematics();
  TestStateMachine();
  TestMotorsReady();
  TestDbusFrameValid();
  TestBenchSquare();
  TestPid();
  TestGimbalAbsoluteHold();
  TestDualBoardComm();
  TestGyroBias();

  std::printf("\n----------------------------------------\n");
  std::printf(" 检查项 %d 个，失败 %d 个\n", g_checks, g_failures);
  if (g_failures == 0) {
    std::printf(" 结果：PASS ✅\n");
  } else {
    std::printf(" 结果：FAIL ❌\n");
  }
  std::printf("----------------------------------------\n");
  return (g_failures == 0) ? 0 : 1;
}
