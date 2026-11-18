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
#include "pid.hpp"
#include "tick.hpp"

/* 轮速环在线调参变量（定义在 Instance/component/ins_pid.cpp，可用 J-Link 直接写） */
extern volatile float g_wheel_kp;
extern volatile float g_wheel_ki;
extern volatile float g_wheel_kd;
extern volatile float g_wheel_ff;
/* 前馈衰减拐点（rad/s）：|vel| 到达它时静摩擦前馈衰减到 0（见 OutputWheels 的注释） */
extern volatile float g_wheel_ff_knee;

namespace robot
{
/* Private constants ---------------------------------------------------------*/

namespace
{
/** 轮子数量 */
constexpr uint8_t kWheelCount = hello_world::chassis_ik_solver::kWheelCount;
/** 低于这个目标轮速就不加静摩擦前馈（rad/s），避免中位抖动被放大 */
constexpr float kFfTargetEps = 0.05f;
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

/* ===== 运行期测试开关（J-Link 直接写，不用重烧）=====
 * 加这两个变量的原因：2026-11-17 实测这台遥控器的**两个拨杆根本没进 DBUS 帧**
 * （byte5 的高 4 位恒为 0xF，无论拨杆怎么动都不变），于是：
 *   - 模式选择（右拨杆）拿不到 → 永远停在 Follow，测平移时被转向盖住
 *   - 急停（左拨杆下）拿不到 → 无法验证"急停→全零"
 * 所以把这两条路径改成可以被调试器直接驱动，测试不再依赖拨杆。
 *
 *   g_force_mode : 0 = 用拨杆（默认，正式使用必须是 0）
 *                  1 = 强制分离  2 = 强制跟随  3 = 强制小陀螺
 *   g_estop_force: 1 = 强制急停（验证全零通路用），0 = 正常
 * ⚠️ 两个都是"上电默认安全值"，掉电/复位自动回到 0。
 */
volatile uint32_t g_force_mode = 0u;
volatile uint32_t g_estop_force = 0u;

/* g_ignore_delta：调试开关，1 = **所有模式**的平移都按车体系解算（强制忽略 δ）。
 * 默认 0（正式值）：小陀螺模式的平移随枪口，见 kFrameDelta() 的说明。
 *
 * 2026-11-17 曾置 1（方案 A）：零点虽已标定，但"云台一动方向就跟着转"，操作手反馈
 * 不如按车体走直观。2026-11-18 起改为 0：小陀螺必须随枪口（底盘在自转，车体系没
 * 法用），其余模式仍在 kFrameDelta() 里保留车体系，兼顾操作习惯。
 * ⚠️ 正式使用必须为 0。 */
volatile uint32_t g_ignore_delta = 0u;

/* ================= 车头方向偏移（机械改了 90°）=================
 * 2026-11-17 实车：车头方向**机械上**换到了原方向的左转 90°。
 * 四个轮子的物理位置/接线没动，所以逆解器（chassis_ik_solver 的四个轮子角色）还是
 * **旧车体系**；只要在"解算用的坐标系角度"上统一加 90°，逆解结果就是新车体系。
 *
 * 为什么不是在逆解里改轮子表：轮子的角色是机械定义的（LF/LB/RB/RF 对应哪个电机），
 * 改轮子表等于重排电机 ID + 方向，风险大得多；这里只是把指令所在坐标系转 90°，
 * 一行动量、随时能改回。
 *
 * 数学：v_旧 = R(+90°)·v_新。而 GimbalToChassis(v, δ) 等价于 R(δ)·v，
 *       所以把传给它的 δ 换成 (δ + 90°) 就同时完成了"坐标系转换"。
 *
 * ⚠️ 车头换了以后，**云台关节角的零点（g_delta_offset）必须重新标定**，
 *    否则 δ 偏 90°，小陀螺的"平移随枪口"会歪 90°。
 *    标定方法见 g_dbg_joint 的注释（把枪口对到新车头，读 g_dbg_joint）。 */
/* 运行期可写（J-Link 直接改 float 位模式）：这是调"前推朝哪走"最快的旋钮，
 * 不用重编译。世界方向 = R(θ车 + δ + 这个值)。
 *
 * 2026-11-17 实测定稿：**0**。
 * 走过的弯路（别再重推）：
 *   · +90° 时三个模式都偏左 90°；−90° 时偏右 180° —— 说明"机械改的那 90°"
 *     **不是**枪口相对车体逆解坐标系的偏角，而是"车头长什么样"的定义。
 *   · 取 0 之后：分离/跟随/小陀螺 前推全部正对枪口（实测通过）。
 * 结论：**前推方向恒等于枪口方向**，与车体外形无关；那 90° 只影响"车头"的叫法。
 * （若以后要改成"前推按车体外形走"，把这里写成 ±90° 并重新验三个模式。） */
volatile float g_front_yaw = 0.0f;

/* ================= 分离模式航向保持 =================
 * 需求（2026-11-17）：分离模式下手不能拨动车头 —— 外力把车拧歪，它自己转回来。
 * 做法：进分离模式那一刻把底盘 IMU 的绝对朝向记为 0°，之后
 *       w = kp·(目标 − 当前)，限幅 ±kSepHoldMaxW，再走常规轮速环。
 * 为什么用 IMU 而不是轮速：外力推车时轮子在滑，轮速环根本不知道车转了。
 * 注意：IMU 无磁力计，长时会有度/分钟级漂移，保持的是"相对上电后的朝向"。
 * g_sep_hold_enable 可在线写 0 关掉（调试转向时用）。 */
constexpr float kSepHoldMaxW = 0.60f;  /* 航向保持最大角速度，rad/s（比 max_w 小，避免甩） */
volatile float g_sep_hold_kp = 1.2f;   /* rad/s per rad，运行期可调 */
/* 阻尼项（2026-11-17 加）：只有 P 时，停下后底盘会左右摆几下才停 —— 误差一出现就
 * 全力转回去、冲过头再反向。加 −kd·ω（ω 取底盘 IMU yaw 角速度）后是一次收敛。
 * 调法：摆得多就加大 kd；回正变慢（发黏）就减小 kd。 */
volatile float g_sep_hold_kd = 0.15f;  /* rad/s per rad/s，运行期可调（实测 0.35 有高频抖，0.15 干脆） */
volatile uint32_t g_sep_hold_enable = 1u;

/* δ 零点重标定用的调试镜像（见 UpdateInputs） */
volatile float g_dbg_joint = 0.0f;  /* 云台关节角原始值（多圈，未扣零点），rad */
volatile float g_dbg_delta = 0.0f;  /* 扣零点并归一化后的 δ，rad */

/* g_frame_lead：小陀螺模式的**坐标系超前补偿系数**，单位 s（rad per rad/s）。
 * 为什么需要：平移指令按下发那一拍的 δ 解算，但车轮/整车真正把速度做出来要 ~0.2 s；
 * 这 0.2 s 里底盘又自转 ω·τ，于是**车实际走的直线方向 = R(ω·τ) × 枪口方向**。
 * 2026-11-17 实测：ω≈1.37 rad/s 时前进方向向自旋侧偏 ~15° = 0.26 rad → τ≈0.19 s。
 * 补偿公式 δ_eff = δ − g_frame_lead·ω（ω 取底盘 IMU 的 yaw 角速度）。
 * 只在小陀螺生效；ω=0 时完全不起作用，所以静态直行不受影响。
 * 可在线改（J-Link 写 float 位模式）：偏得更多就调小，反向就写负值。 */
volatile float g_frame_lead = 0.19f;

/* g_delta_offset：云台关节角的**零点标定**（rad），δ 使用 (关节角 − 该值)。
 * 为什么需要：DM 电机报的是编码器关节角，它的 0 位和"车头方向"没有固定关系。
 * 不标定零点，"前推 = 沿枪口走"虽然数学上成立，但操作起来方向全乱。
 * 2026-11-17 实测标定：把云台摆到正对车头时读到的关节角是 2.661 rad（152.5°），
 * 但按该值标定后跟随模式收敛时枪口仍右偏 45°（47.2°），说明人眼对齐有误差。
 * 于是在分离模式（底盘不转）下重新精确标定：offset 再加 0.823 → **3.4842**。
 * 换车/换电机/重新装过后必须重新标定这个值。
 * 标定方法：云台正对车头，读 delta_（g_delta_offset=0 时即原始关节角），写回此处。 */
/* 2026-11-17 机械改 90° 后重新标定：**1.8900**（旧值 3.4842 作废）。
 * 标定方法（和以前一样）：把枪口对到**新车头**方向，读 g_dbg_joint，把它写进这里。
 * 当次读数来自操作手用长基线对准后的关节角，δ 残差 −0.06°，足够。
 * ⚠️ 这个值和 g_front_yaw 是**一对**：两者一起决定"推前进朝哪走"，改一个必须重验另一个。 */
volatile float g_delta_offset = 1.8900f;

/* ===== 跟随环在线整定（J-Link 直接写）=====
 * 2026-11-17 实测：δ=-0.211 rad 时切到跟随模式，车**左右反复摆了几下**才停。
 * 原因是 kp=4.0 相对限幅太大：w = 4×0.211 = 0.84 rad/s，而 max_w 只有 0.40，
 * 输出直接饱和成"满舵"→ 冲过头 → 反向再冲 → 欠阻尼振荡。
 * 改成 kp=1.5（δ=0.21 rad 时只给 0.32 rad/s，不饱和）+ kd=0.30（加阻尼）。
 * 手感目标：转云台 90° 时车头平滑追过去、不来回摆。
 *
 * 2026-11-17 实车再下调：kp=1.5/kd=0.30 时仍有轻微来回摆（云台不动、只有车在摆，
 * 说明是跟随环自身的极限环）。最终在线调到 kp=0.8 / kd=0.15 后基本不摆，
 * 2026-11-17 最终定稿：kp=1.2 / kd=0.20。配合 max_w 由 0.40 提到 0.80，
 * 转 90° 的时间从 3.9 s 缩短到约 2 s，且仍不来回摆。 */
volatile float g_follow_kp = 1.2f;
volatile float g_follow_kd = 0.20f;

/* ===== 小陀螺模式的平移倍率（运行期可调）=====
 * 2026-11-17 晚按操作手要求：**小陀螺档平移要更快**。
 * 自旋时每轮都有一部分轮速被自旋项占掉（0.3068×max_spin_w ≈ 0.46 m/s），
 * 平移余量本来就少，所以只放大 spin 档；分离/跟随保持已验证的 0.20 m/s 不动。
 *
 * 取值：1.0 ~ kSpinTransBoostMax（默认 1.5）。J-Link 写 float 位模式即可，
 * 复位回默认。⚠️ 调好之后要写回 chassis.hpp 的 kSpinTransBoostMax（它是闸门
 * static_assert 的输入，运行期这个变量被 Clamp 在它以内，不会破坏断言）。
 *
 * 换算（名义值，实车另有 ~1.71 倍比例未定案，见 handoff 第五节）：
 *   1.5 × 0.20 = 0.30 m/s 名义 → 实车约 0.5 m/s */
volatile float g_spin_trans_boost = kSpinTransBoostMax;

/* ==================== 小陀螺自旋诊断记录（只写 RAM，不参与控制）====================
 * 为什么要有它：J-Link 的 connect 会把 CPU halt 住，单次 mem32 只能拿到一个**冻结的
 * 快照**；而"自旋不匀速"是一段**时间过程**（正常 → 逐渐变慢 → 稳定在低转速），
 * 必须看曲线，不能只看某一瞬间。这里在 1ms 控制周期里按 100 Hz 采样，把关键量
 * 存进 RAM，跑完再离线导出来，不依赖调试器在线（SWD 可以先拔掉，避免绞线）。
 *
 * 用法（默认已开，不需要改编译开关、不需要 J-Link 在线）：
 *   1) 上电后正常操作，进小陀螺并把拨轮推起来（|w| > 0.2 rad/s 时自动开始录）；
 *   2) 录满 kTraceRows 条（100 Hz × 7 s）自动停；数据一直保留到复位；
 *   3) pwsh -File tools\jlink.ps1 -Action dump-trace -Board chassis
 *      python tools\spin_trace.py --dump spin_trace.bin
 *   （dump-trace 顺手把 g_trace_state/g_trace_n 清零，下一次拨轮就可以再录一遍）
 *
 * 每行 21 个 float，列定义见 TraceTick()。
 * 为什么不改变控制：只读 command_ / chassis_->wheel_vel() / motor->vel() / pid->out()，
 * 一个都不写回，纯观测。
 */
constexpr uint32_t kTraceCols = 21u;
constexpr uint32_t kTraceRows = 700u;              ///< 100 Hz × 7 s
constexpr uint32_t kTraceDiv = 10u;                ///< 1 kHz / 10 = 100 Hz
constexpr uint32_t kTraceModeSpin = 2u;            ///< robot::ChassisMode::kSpin
constexpr float kTraceTriggerW = 0.2f;             ///< 自旋指令超过它才开始录

volatile float g_trace_buf[kTraceRows][kTraceCols];
volatile uint32_t g_trace_state = 0u;    ///< 0=待触发 1=录制中 2=录满
volatile uint32_t g_trace_n = 0u;        ///< 已存条数
volatile uint32_t g_trace_t0_ms = 0u;    ///< 第一条的 uwTick
volatile uint32_t g_trace_arm = 1u;      ///< 1=允许自动触发（只写 RAM，安全）
volatile uint32_t g_trace_mode = 0u;     ///< 触发时的模式（应为 2=Spin）

namespace
{
/** 台架模式是否开启：魔数匹配才认，避免手滑写个 1 就把五个联锁全绕过了 */
bool BenchOn(void) { return g_bench_enable == kBenchMagic; }

/* 自旋诊断的跨周期状态（普通变量，不需要被 J-Link 看见） */
float s_trace_prev_vel[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
float s_trace_jitter[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
uint32_t s_trace_div = 0u;

/**
 * @brief       自旋诊断采样（每个 1ms 周期调用，实际 10 个周期存一条 = 100 Hz）
 * @param       now_ms : 当前 uwTick
 * @param       mode   : 本周期底盘模式原始值（2 = 小陀螺）
 * @param       w_cmd  : command_.w（拨轮给出的自旋指令）
 * @param       delta  : 真实 δ（云台 − 车头），rad
 * @param       tgt    : 四轮速度目标，m/s（逆解 + 限幅之后）
 * @param       vel    : 四轮实际速度，rad/s（M3508 反馈，输出轴）
 * @param       out    : 四轮速度环输出，raw（±16384 满量程）
 * @param       imu_yaw: 底盘 IMU 绝对 yaw，rad（独立的"车真转了多少"证据）
 *
 * 列定义（每行 21 个 float，轮序 LF/LB/RB/RF）：
 *   [0]      t_ms（相对触发时刻）
 *   [1]      mode
 *   [2]      w_cmd       拨轮给出的自旋指令，rad/s
 *   [3]      delta       真实 δ（云台 − 车头），rad
 *   [4..7]   四轮目标 m/s（逆解 + 限幅之后）
 *   [8..11]  四轮实际 rad/s（M3508 反馈，输出轴）
 *   [12..15] 四轮速度环输出 raw（±16384 满量程，本程序限到 ±3000）
 *   [16..19] 四轮"1ms 级速度跳变量"（每 10ms 窗口累计 |Δvel|，rad/s）
 *   [20]     底盘 IMU 绝对 yaw，rad
 *
 * jitter 列是做什么的：如果两颗 C620 被拨成同一个 ID（0x201 帧数是别人的整 2 倍的
 * 可疑点），同一个 M3508 对象会交替收到两台车轮的反馈，vel_ 在 ±目标之间来回跳，
 * 这一列会大到几十；正常轮子随着真实加速，累计值通常 < 1。
 */
void TraceTick(uint32_t now_ms, uint8_t mode, float w_cmd, float delta,
               const float tgt[kWheelCount], const float vel[kWheelCount],
               const float out[kWheelCount], float imu_yaw)
{
  /* 1) 每 1ms 累计速度跳变（跨周期，先累加再判是否该存） */
  for (uint32_t i = 0u; i < kWheelCount; ++i) {
    s_trace_jitter[i] += fabsf(vel[i] - s_trace_prev_vel[i]);
    s_trace_prev_vel[i] = vel[i];
  }

  if (g_trace_arm == 0u) {
    return;
  }

  /* 2) 待触发：只在"小陀螺 + 拨轮真的给了自旋"时开始，避免一上电就录到静止数据 */
  if (g_trace_state == 0u) {
    if ((mode == kTraceModeSpin) && (fabsf(w_cmd) > kTraceTriggerW)) {
      g_trace_state = 1u;
      g_trace_n = 0u;
      g_trace_t0_ms = now_ms;
      g_trace_mode = mode;
      s_trace_div = 0u;
      for (uint32_t i = 0u; i < kWheelCount; ++i) {
        s_trace_jitter[i] = 0.0f;
      }
    } else {
      return;
    }
  }
  if (g_trace_state != 1u) {
    /* 已录满：jitter 归零，避免长时间累加变成无意义的大数 */
    for (uint32_t i = 0u; i < kWheelCount; ++i) {
      s_trace_jitter[i] = 0.0f;
    }
    return;
  }

  /* 3) 抽稀到 100 Hz */
  s_trace_div = s_trace_div + 1u;
  if (s_trace_div < kTraceDiv) {
    return;
  }
  s_trace_div = 0u;

  const uint32_t row = g_trace_n;
  if (row >= kTraceRows) {
    g_trace_state = 2u;
    return;
  }

  volatile float* const r = g_trace_buf[row];
  r[0] = static_cast<float>(now_ms - g_trace_t0_ms);
  r[1] = static_cast<float>(mode);
  r[2] = w_cmd;
  r[3] = delta;
  for (uint32_t i = 0u; i < kWheelCount; ++i) {
    r[4u + i] = tgt[i];
  }
  for (uint32_t i = 0u; i < kWheelCount; ++i) {
    r[8u + i] = vel[i];
  }
  for (uint32_t i = 0u; i < kWheelCount; ++i) {
    r[12u + i] = out[i];
  }
  for (uint32_t i = 0u; i < kWheelCount; ++i) {
    r[16u + i] = s_trace_jitter[i];
  }
  r[20] = imu_yaw;

  g_trace_n = row + 1u;
  if (g_trace_n >= kTraceRows) {
    g_trace_state = 2u;
  }
  for (uint32_t i = 0u; i < kWheelCount; ++i) {
    s_trace_jitter[i] = 0.0f;
  }
}
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
    /* ⚠️ 2026-11-17：遥控离线必须让云台也归零，所以把"离线"并进这一位。
     * 原因：接收机接在**底盘板**上，云台板的在线判据是 rc_online = local || peer
     * （见 Gimbal robot.cpp），只要本板还活着它就认为"遥控在线"，于是继续留在
     * Working 里保持力矩 —— 遥控器断电后云台仍然有力，违反"断遥控必须没力"。
     * ChassisData::estop 的定义本来就是"底盘侧急停（遥控器急停位 / 遥控离线）"，
     * 这里补上"遥控离线"这半句；云台侧现成的 peer-estop 逻辑会走 OutputZero。
     * 注意：只影响**发给云台的那一帧**，不改变本板 estop_ 的语义。 */
    d.estop = estop_ || (!inputs_.rc_online);
    d.mode = mode_;
    d.gimbal_mode = GimbalMode::kJointSpace;
    /* 遥控接收机实际接在【底盘板】上，云台板收不到 DBUS。
     * 因此把右摇杆的归一化值（-1~1）放进 yaw_des / pitch_des 转发给云台板，
     * 云台侧再乘自己的速率上限。这样在不动接线的前提下，云台也能被操作手控制。 */
    if ((rc_ != nullptr) && inputs_.rc_online) {
      /* 同样先过死区，避免中位抖动被转成云台的持续漂移 */
      d.yaw_des = -ApplyStickDeadzone(rc_->rc_rh());   /* 右摇杆横向 → yaw 角速度（归一化） */
      /* 2026-11-17：模式已挪到**右上三挡拨杆**，右摇杆竖直整个让给 pitch。
       * 不再有 |rv| ≤ 0.5 的门限 —— 那是"选择器"时代的限制，会导致 pitch 打不到
       * 行程两端。现在右摇杆竖直全程透传，云台侧按 max_pitch_rate 积分目标，
       * 位置环负责"停住"（松手即停在当前位置）。 */
      d.pitch_des = ApplyStickDeadzone(rc_->rc_rv());
    } else {
      /* 没有接收机，或**遥控离线**：一律给 0。
       * 旧代码在离线时仍然转发 rc_ 里"最后收到的那一拍摇杆值"，云台会按这个陈旧值
       * 一直积分目标（断遥控后云台自己慢慢转），比"保持力矩"更危险。 */
      d.yaw_des = 0.0f;
      d.pitch_des = 0.0f;
    }
    d.w_des = command_.w;
    d.seq = ++comm_seq_;
  }

  /* ---------- 小陀螺诊断记录：只读本周期最终结果，写 RAM，不参与控制 ---------- */
  {
    float tgt[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
    float vel[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
    float out[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
    if (chassis_ != nullptr) {
      const hello_world::chassis_ik_solver::WheelVel& wv = chassis_->wheel_vel();
      for (uint8_t i = 0; i < kWheelCount; ++i) {
        tgt[i] = wv.v[i];
      }
    }
    for (uint8_t i = 0; i < kWheelCount; ++i) {
      if (motors_[i] != nullptr) {
        vel[i] = motors_[i]->vel();
      }
      if (wheel_pids_[i] != nullptr) {
        out[i] = wheel_pids_[i]->out();
      }
    }
    TraceTick(hello_world::tick::GetTickMs(), static_cast<uint8_t>(mode_), command_.w, delta_,
              tgt, vel, out, (imu_ != nullptr) ? imu_->yaw() : 0.0f);
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
    /* 急停判定（2026-11-17 晚按操作手要求改）：**左上小拨杆上推 = 急停**，唯一通路。
     *
     * 为什么改：左摇杆整根要留给底盘平移 —— 旧写法"左摇杆拉到底(≤-0.9)=急停"把后退
     * 行程吃掉了，操作手要倒车就得冒着触发急停的风险。
     *
     * 旧的两条都撤掉了（左摇杆 ≤ -0.9、左拨杆拨到最下）：
     *   · 左摇杆到底 —— 与"摇杆控制底盘"冲突，见上；
     *   · 左拨杆最下 —— 现在最下是"非急停"档，把最上留给急停，拨杆三档都用满：
     *       上 = 急停（拉一下就跑不掉了，fail-safe 方向朝上，手下滑也不会误触发）
     *       中 / 下 = 正常运行
     * ⚠️ 上电时拨杆若停在"上"，车会一直 Dead（看上去像没反应）——这是设计行为。
     * ⚠️ 开关取值实测：上 = kSwitchStateUp(1) / 中 = Mid(3) / 下 = Down(2)，
     *    见 docs/known_issues.md 第 6 节。 */
    estop_switch = (rc_->rc_l_switch() == hello_world::remote_control::kSwitchStateUp);
    /* ---------- 模式选择：**右上三挡拨杆**（2026-11-17 实测拨杆已进帧）----------
     * 映射（与 DT7 定义一致，实测 data[5]）：上=1 → 分离，中=3 → 跟随，下=2 → 小陀螺。
     * 为什么从"右摇杆拨一下切一档"改成拨杆：右摇杆竖直要整个让给云台 pitch，
     * 一根轴不能同时当模式选择器（之前 |rv|>0.5 时不许 pitch，pitch 就没法用）。
     *
     * ⚠️ 上电互锁：复位后必须**先在"分离"档出现一次**，之后才按拨杆选模式。
     *    否则上电时拨杆若停在小陀螺档，一武装就直接自旋（这车 1.0 rad/s 的自旋
     *    已经在验收里了，不能靠人记得先回位）。复位后 latch 自动清零。
     *
     * 调试时 g_force_mode 优先（1/2/3 → Separate/Follow/Spin）。 */
    if ((g_force_mode >= 1u) && (g_force_mode <= 3u)) {
      mode_ = static_cast<ChassisMode>(static_cast<uint8_t>(g_force_mode - 1u));
    } else {
      const hello_world::remote_control::SwitchState sw = rc_->rc_r_switch();
      if (!mode_sw_latched_) {
        /* 互锁未解除：强制分离，直到看到一次"分离"档 */
        mode_ = ChassisMode::kSeparate;
        if (sw == hello_world::remote_control::kSwitchStateUp) {
          mode_sw_latched_ = true;
        }
      } else if (sw == hello_world::remote_control::kSwitchStateDown) {
        mode_ = ChassisMode::kSpin;
      } else if (sw == hello_world::remote_control::kSwitchStateMid) {
        mode_ = ChassisMode::kFollow;
      } else {
        mode_ = ChassisMode::kSeparate;
      }
    }
  }
  /* 急停：拨杆拿不到时可用 g_estop_force 强制置位，验证"急停→全零"通路 */
  estop_ = estop_switch || (g_estop_force == 1u);

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
  const float joint_raw = peer_online_raw ? comm_->gimbal_data().yaw : 0.0f;
  delta_ = joint_raw;
  /* 关节角零点标定：云台正对车头时把当时读到的值写进 g_delta_offset，
   * 之后 δ≈0 就代表"枪口正对车头"。见 g_delta_offset 的注释。
   * ⚠️ 这里保留**真实 δ**（跟随模式要用它算 w）；
   *    只有"平移坐标系"会因为 g_ignore_delta=1 而用 0，见 GenerateCommands。 */
  delta_ = WrapToPi(delta_ - g_delta_offset);

  /* 调试镜像（J-Link 直接读，重标 δ 零点时用）：
   *   g_dbg_joint = 云台关节角原始值（多圈，未扣零点）
   *   g_dbg_delta = 扣零点并归一化后的 δ
   * 标定：把枪口对到"新车头"方向，读 g_dbg_joint，把 g_delta_offset 写成该值。 */
  g_dbg_joint = joint_raw;
  g_dbg_delta = delta_;

  /* 模式切换沿：进分离模式那一刻重新取航向保持目标（见 GenerateCommands 的 kSeparate） */
  if (mode_ != last_mode_) {
    if (mode_ == ChassisMode::kSeparate) {
      sep_hold_valid_ = false;
    }
    last_mode_ = mode_;
  }

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
    chassis_->SetCommand(command_, kFrameDelta());
    return;
  }

  /* 左摇杆：竖 → 前后，横 → 左右（右推为 −y，因为 y 轴指向左方）
   * 所有摇杆量先过死区，保证中位抖动不会变成残余速度（零输入即零输出） */
  const float lv_dz = ApplyStickDeadzone(rc_->rc_lv());
  const float lh_dz = ApplyStickDeadzone(rc_->rc_lh());
  const float rh_dz = ApplyStickDeadzone(rc_->rc_rh());

  /* ---------- 上电武装互锁（2026-11-17 修正） ----------
   * 目的：防止"带着摇杆上电"时底盘立刻窜车。
   * 语义：进入 Working 后，先要求**所有**摇杆回中并保持 kArmHoldMs，之后一直保持武装；
   *       武装完成后摇杆怎么推都不再清零指令。
   *
   * ⚠️ 旧写法是"任一摇杆偏转就把 arm_ms_ 清零"，于是推杆的同一拍指令就被清零 ——
   *    现场表现：左摇杆一推，四个轮子立刻停；松杆 200 ms 后又恢复跟随模式的慢转。
   *    结果就是底盘永远走不了（只有台架开环能转）。别再把它写成"持续检查"。
   * 急停 / 退出 Working 时 OutputZero() 会把 arm_ms_ 清零，下次进 Working 需重新武装。 */
  constexpr float kArmHoldMs = 200.0f;
  if (arm_ms_ < kArmHoldMs) {
    /* 武装阶段：四轴都要回中（含右摇杆竖直，防止带 pitch 杆上电） */
    const float rv_dz = ApplyStickDeadzone(rc_->rc_rv());
    if ((lv_dz == 0.0f) && (lh_dz == 0.0f) && (rv_dz == 0.0f) && (rh_dz == 0.0f)) {
      arm_ms_ += dt * 1000.0f;
    } else {
      arm_ms_ = 0.0f;
    }
    command_.setZero();
    return;
  }

  const float vx_cmd = lv_dz * lim.max_vx;
  const float vy_cmd = -lh_dz * lim.max_vy;
  /* ⚠️ 右摇杆横向**不再**驱动底盘转向（方案 A，2026-11-17）。
   * 原因：同一根轴同时被用于「底盘 w」和「云台 yaw_des」，操作上分不清 ——
   * 想转车头时枪口跟着甩，跟随模式反而追不上。
   * 现在右摇杆横向只用来瞄枪口（见 Update() 里的 yaw_des）；
   * 需要"原地转"请进跟随模式：转云台，车头自动追过去。 */

  switch (mode_) {
    case ChassisMode::kFollow: {
      /* w 由「云台朝向 − 车头朝向」闭环给出；δ 就是两者之差。
       * 每周期把在线整定值写进跟随环（set_config 不动积分，改完立刻生效）。 */
      hello_world::pid::Pid::Config fc;
      fc.kp = g_follow_kp;
      fc.ki = 0.0f;
      fc.kd = g_follow_kd;
      fc.out_min = -6.0f;
      fc.out_max = 6.0f;
      fc.int_min = -2.0f;
      fc.int_max = 2.0f;
      follow_pid_.set_config(fc);
      const float w_follow = follow_pid_.UpdateErr(WrapToPi(delta_), dt);
      command_.vx = vx_cmd;
      command_.vy = vy_cmd;
      command_.w = Clamp(w_follow, -lim.max_w, lim.max_w);
      break;
    }
    case ChassisMode::kSpin: {
      /* 小陀螺：w 由拨轮设定，必须限幅；平移仍以云台朝向为基准。
       * 2026-11-17 晚：**平移单独放大**（g_spin_trans_boost，默认 1.5×）。
       * 只放大 spin —— 分离/跟随维持已验证的 0.20 m/s，不引入新变量。
       * 上限受 kSpinTransBoostMax 夹住，而闸门 kMaxWheelVelMps 已按它算过
       * （main_task.cpp 的 static_assert），所以这里放大不会被闸门等比削掉。 */
      const float w_spin = ApplyStickDeadzone(rc_->rc_wheel()) * lim.max_spin_w;
      const float boost = Clamp(g_spin_trans_boost, 1.0f, kSpinTransBoostMax);
      command_.vx = vx_cmd * boost;
      command_.vy = vy_cmd * boost;
      command_.w = Clamp(w_spin, -lim.max_spin_w, lim.max_spin_w);
      break;
    }
    case ChassisMode::kSeparate:
    default: {
      /* 分离模式：平移 + **航向保持**（2026-11-17 新需求：手不能拨动车头）。
       * 目标 = 进分离那一刻的车头绝对朝向（sep_hold_valid_ 失效时重新取）。
       * w 由 P 环给出并限幅；不打滑时不顶、被外力拧歪才回正。 */
      command_.vx = vx_cmd;
      command_.vy = vy_cmd;
      if ((g_sep_hold_enable != 0u) && (imu_ != nullptr)) {
        if (!sep_hold_valid_) {
          sep_yaw_target_ = imu_->yaw();
          sep_hold_valid_ = true;
        }
        const float err = WrapToPi(sep_yaw_target_ - imu_->yaw());
        /* PD：P 把车拧回来，D（−kd·ω）负责刹车，避免冲过头来回摆。
         * ω 用 IMU 的 yaw 角速度（不是误差微分），手推时同样生效。 */
        const float rate = imu_->yawRate();
        command_.w =
            Clamp(g_sep_hold_kp * err - g_sep_hold_kd * rate, -kSepHoldMaxW, kSepHoldMaxW);
      } else {
        command_.w = 0.0f;
      }
      break;
    }
  }

  chassis_->SetCommand(command_, kFrameDelta());
}

/**
 * @brief       平移解算用的坐标系角度
 * @retval      0 = 车体系；否则 = 真实 δ（云台系）
 * @note        ⚠️ 只用于 SetCommand 的坐标系参数。跟随模式算 w 必须用真实 delta_，
 *              否则 δ 恒为 0、跟随模式失效。
 *
 * 2026-11-18：**小陀螺模式用真实 δ，其余模式仍按车体系**。
 *   为什么小陀螺必须用 δ：底盘在自转，车体系里的"前"每时每刻都在转 —— 操作手推着
 *   "前"走，车却一边转一边画圈，平移根本没法用。锁到云台系以后，推前 = 沿枪口走，
 *   枪口指着哪儿车就往哪儿平移，这才是验收项「拨轮控自旋；平移方向仍随枪口」。
 *   为什么其余模式不用 δ：分离模式底盘不主动转，两种坐标系只在"云台被人为转开"
 *   时有差别，操作手反馈车体系更直观（2026-11-17 现场结论）；跟随模式底盘本来就在
 *   往枪口对齐，δ→0，两者差别可以忽略。
 *   要强制所有模式都按车体系（例如单独验方向），把 g_ignore_delta 写 1。
 *   云台板离线时 delta_ 在 UpdateInputs() 里被置 0，会自动退回车体系。
 */
float Robot::kFrameDelta(void) const
{
  if (g_ignore_delta == 1u) {
    return 0.0f;   /* 调试：直接用旧车体系（验四轮方向时用），不含 90° 偏移 */
  }

  /* ===== 2026-11-17 定稿：**所有模式**都用 δ（平移随枪口）=====
   * 曾经只在"小陀螺"用 δ、分离/跟随用车体系，现场立刻暴露两个问题：
   *   · 分离模式：底盘航向保持不动，于是"前"在世界上是固定方向，转云台不跟 → 用户预期不符
   *   · 跟随模式：底盘对齐后 δ→0，只剩 g_front_yaw 的 90°，前推直接偏到左边 90°
   * 根因是把两件事混在一起了：
   *   a) 机械上枪口相对车体物理坐标系本来就偏 90°（g_front_yaw 就是它）；
   *   b) "前推朝哪走"应该恒等于枪口。
   * 只要解算统一用 (δ + g_front_yaw)，世界方向 = R(枪口方位) —— a 被消掉，
   * 所有模式都满足 b。推导：世界方向 = R(θ车 + δ + f)，而 δ = φ枪 − θ车 − a，
   * 代入得 R(φ − a + f)，取 f = a 即得 R(φ)（正对枪口）。
   *
   * 角速度超前补偿只对小陀螺做：那是"底盘在自转、坐标系也在转"才有的相位滞后；
   * 分离/跟随的 ω≈0，加了也没用，反而可能引入噪声。 */
  float frame = WrapToPi(delta_ + g_front_yaw);
  if (mode_ != ChassisMode::kSpin || imu_ == nullptr) {
    return frame;
  }
  float lead = g_frame_lead * imu_->yawRate();
  if (lead > 0.6f) {
    lead = 0.6f;
  } else if (lead < -0.6f) {
    lead = -0.6f;
  }
  return WrapToPi(frame - lead);
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

    /* 指令为 0 时清掉积分并直接输出 0。
     * 为什么必须清：ki=150，目标为 0 时误差恒为 0 → 积分**冻结**在上一轮的值上，
     * 一直给着几十~一百多 raw 的电流（实测松手后 out_ 还是 ±170），轮子会慢慢爬。
     * 清零后松手就是纯滑行，符合"松手就停"。 */
    if (fabsf(target_rad_s) < 1e-4f) {
      wheel_pids_[i]->Reset();
      (void)motors_[i]->setInput(0.0f);
      continue;
    }

    /* 每周期把在线调参值写进轮速环：set_config 不动积分，改完下一个周期就生效 */
    hello_world::pid::Pid::Config wheel_cfg;
    wheel_cfg.kp = g_wheel_kp;
    wheel_cfg.ki = g_wheel_ki;
    wheel_cfg.kd = g_wheel_kd;
    wheel_cfg.out_min = -3000.0f;
    wheel_cfg.out_max = 3000.0f;
    wheel_cfg.int_min = -1000.0f;
    wheel_cfg.int_max = 1000.0f;
    wheel_pids_[i]->set_config(wheel_cfg);

    float out = wheel_pids_[i]->Update(target_rad_s, motors_[i]->vel(), dt);

    /* 静摩擦前馈：低目标速度下纯 PID 给不出克服静摩擦的电流（推满杆轮子纹丝不动）。
     * 2026-11-17 实测静摩擦门槛 ≈ raw 655~819；稳态电流由积分项负责补齐。
     *
     * ⚠️ 2026-11-18：从"常数"改成**随速度衰减**。
     *    常数 ff 在轮子动起来之后就是**过量推力**（动摩擦 ≪ 静摩擦）：架空实测
     *    150/150/ff=700 时超调 **106%**、2 s 后还差 36%，就是操作手说的
     *    "一开始快然后变慢"。现在 |vel| 达到 g_wheel_ff_knee 时 ff 衰减到 0，
     *    只保留"破静摩擦"那几十毫秒的助推（knee=1.0 rad/s ≈ 0.078 m/s 轮面速度）。 */
    const float ff_knee = (g_wheel_ff_knee > 0.1f) ? g_wheel_ff_knee : 0.1f;
    const float ff_scale = Clamp(1.0f - (fabsf(motors_[i]->vel()) / ff_knee), 0.0f, 1.0f);
    const float ff_eff = g_wheel_ff * ff_scale;
    if (target_rad_s > kFfTargetEps) {
      out += ff_eff;
    } else if (target_rad_s < -kFfTargetEps) {
      out -= ff_eff;
    }
    (void)motors_[i]->setInput(out);
  }
}

void Robot::OutputZero(void)
{
  command_.setZero();
  /* 退出闭环就解除武装：再次进入 Working 必须重新「摇杆回中」才允许输出 */
  arm_ms_ = 0.0f;
  /* 航向保持目标作废：下次进 Working 时按"当时"的车头朝向重新取，
   * 否则会拿上一次会话的旧目标，一上电就往旧方向转。 */
  sep_hold_valid_ = false;
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
