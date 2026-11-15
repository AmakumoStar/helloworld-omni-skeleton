/**
 *******************************************************************************
 * @file      :module_state.hpp
 * @brief     :底盘板 / 云台板共享的状态类型、枚举与纯数学工具
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 补全三态枚举 / 模式枚举 / 运动结构
 *                                                2. 补角度归一化与坐标系变换工具
 *******************************************************************************
 * @attention :
 *  1. 本文件被底盘板、云台板同时编译，只允许放「纯类型 / 枚举 / inline 函数」，
 *     严禁依赖 STM32 HAL、电机驱动或任何具体硬件。
 *  2. 全工程的单位约定（见 docs/units_and_protocol.md）：
 *       - 速度 vx / vy：m/s      - 角速度 w：rad/s
 *       - 角度 yaw / pitch：rad  - 角度一律以「逆时针为正」（z 轴向上，右手法则）
 *  3. 坐标系约定：前进正方向恒为云台朝向。
 *       - 云台系：x = 云台朝向（枪管方向），y = 云台左方，z = 上
 *       - 车体系：x = 车头朝向，      y = 车体左方，z = 上
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MODULE_STATE_HPP_
#define MODULE_STATE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <cstdint>

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
namespace robot
{

/* ------------------------------ 数学常量 --------------------------------- */

inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kTwoPi = 2.0f * kPi;

/* ------------------------- 双板 CAN 标准帧 ID ---------------------------- */
/* 两板必须使用同一组 ID；已避开电机占用：
 *   M3508 控制 0x200 / 0x1FF，反馈 0x201~0x208
 *   DM-J4310 发送 0x01~0x0F，反馈 0x11~0x1F
 */
inline constexpr uint32_t kCanIdChassisToGimbal = 0x210u;  ///< 底盘板 → 云台板
inline constexpr uint32_t kCanIdGimbalToChassis = 0x211u;  ///< 云台板 → 底盘板

/* ------------------------------ 状态枚举 --------------------------------- */

/** 生命状态：上电未就绪 → 恢复自检 → 正常工作 */
enum class LifeState : uint8_t {
  kDead = 0,          ///< 上电未就绪 / 失效：所有电机指令必须为 0
  kResurrection = 1,  ///< 恢复自检：使能电机、等 IMU 收敛、清残留指令
  kWorking = 2,       ///< 正常工作：按模式生成指令
};

/** 手动控制源（本考核只用遥控器，预留扩展） */
enum class ControlSource : uint8_t {
  kNone = 0,           ///< 无有效控制源
  kRemoteControl = 1,  ///< DT7 遥控器
};

/** 底盘工作模式 */
enum class ChassisMode : uint8_t {
  kSeparate = 0,  ///< 分离：底盘 (vx, vy, w) 全部来自遥控器直接映射
  kFollow = 1,    ///< 跟随：w 由「云台朝向 − 车头朝向」闭环给出
  kSpin = 2,      ///< 小陀螺：绕自身中心自旋，平移仍以云台朝向为基准
};

/** 云台工作模式（关节空间控 yaw / pitch） */
enum class GimbalMode : uint8_t {
  kJointSpace = 0,  ///< 关节空间位置控制
};

/* ---------------------------- 模式切换档位 ------------------------------- */
/* 遥控器右拨杆：Up = 分离，Mid = 跟随，Down = 小陀螺（验收时按此演示） */
/* 遥控器左拨杆：Down = 急停（其余档位才允许输出）                      */

/* ------------------------------ 运动结构 --------------------------------- */

/** 底盘运动状态：平移 + 自旋，单位 m/s、m/s、rad/s */
struct ChassisVel {
  float vx = 0.0f;  ///< 云台系 x 轴（云台朝向）线速度，m/s
  float vy = 0.0f;  ///< 云台系 y 轴（云台左方）线速度，m/s
  float w = 0.0f;   ///< 绕 z 轴角速度，rad/s，逆时针为正

  ChassisVel() = default;
  ChassisVel(float vx_in, float vy_in, float w_in) : vx(vx_in), vy(vy_in), w(w_in) {}

  ChassisVel operator+(const ChassisVel& rhs) const
  {
    return ChassisVel(vx + rhs.vx, vy + rhs.vy, w + rhs.w);
  }
  ChassisVel operator-(const ChassisVel& rhs) const
  {
    return ChassisVel(vx - rhs.vx, vy - rhs.vy, w - rhs.w);
  }
  ChassisVel operator*(float scale) const
  {
    return ChassisVel(vx * scale, vy * scale, w * scale);
  }
  ChassisVel& operator+=(const ChassisVel& rhs)
  {
    vx += rhs.vx;
    vy += rhs.vy;
    w += rhs.w;
    return *this;
  }
  ChassisVel& operator-=(const ChassisVel& rhs)
  {
    vx -= rhs.vx;
    vy -= rhs.vy;
    w -= rhs.w;
    return *this;
  }
  ChassisVel operator-() const { return ChassisVel(-vx, -vy, -w); }

  /** 是否近似为 0（用于急停 / 失能判定） */
  bool isZero(float eps = 1e-6f) const
  {
    return std::fabs(vx) < eps && std::fabs(vy) < eps && std::fabs(w) < eps;
  }

  /** 归零（急停、状态切换时使用） */
  void setZero()
  {
    vx = 0.0f;
    vy = 0.0f;
    w = 0.0f;
  }
};

/** 云台关节位置：yaw / pitch，单位 rad */
struct GimbalPos {
  float yaw = 0.0f;    ///< 偏航角，rad，支持连续旋转（可超 ±π，由使用方归一化）
  float pitch = 0.0f;  ///< 俯仰角，rad，受软件限位约束

  GimbalPos() = default;
  GimbalPos(float yaw_in, float pitch_in) : yaw(yaw_in), pitch(pitch_in) {}

  GimbalPos operator+(const GimbalPos& rhs) const
  {
    return GimbalPos(yaw + rhs.yaw, pitch + rhs.pitch);
  }
  GimbalPos operator-(const GimbalPos& rhs) const
  {
    return GimbalPos(yaw - rhs.yaw, pitch - rhs.pitch);
  }
  GimbalPos operator*(float scale) const { return GimbalPos(yaw * scale, pitch * scale); }
  GimbalPos& operator+=(const GimbalPos& rhs)
  {
    yaw += rhs.yaw;
    pitch += rhs.pitch;
    return *this;
  }
  GimbalPos operator-() const { return GimbalPos(-yaw, -pitch); }

  void setZero()
  {
    yaw = 0.0f;
    pitch = 0.0f;
  }
};

/* ---------------------------- 纯数学工具 --------------------------------- */

/** 限幅 */
inline float Clamp(float value, float lower, float upper)
{
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

/** 摇杆死区半宽（归一化量） */
inline constexpr float kStickDeadzone = 0.05f;

/**
 * @brief       摇杆死区 + 区间重映射
 * @param       value: 归一化摇杆值 [-1, 1]
 * @param       deadzone: 死区半宽
 * @retval      死区内返回 0；死区外重新映射回 [-1, 1]
 * @note        DT7 中位有 ±0.02 左右的抖动，不做死区会导致「松手后车缓慢爬行」，
 *              直接违反考核要求的「零输入即零输出」。所有摇杆量都必须先过这里。
 */
inline float ApplyStickDeadzone(float value, float deadzone = kStickDeadzone)
{
  const float v = Clamp(value, -1.0f, 1.0f);
  if (deadzone <= 0.0f) {
    return v;
  }
  if (v > deadzone) {
    return (v - deadzone) / (1.0f - deadzone);
  }
  if (v < -deadzone) {
    return (v + deadzone) / (1.0f - deadzone);
  }
  return 0.0f;
}

/**
 * @brief       把角度归一化到 [-π, π)
 * @note        359° → 1° 必须当成 +2°，而不是 −358°；所有角度差都要先过这里
 */
inline float WrapToPi(float angle)
{
  if (!std::isfinite(angle)) {
    return 0.0f;
  }
  float wrapped = std::fmod(angle + kPi, kTwoPi);
  if (wrapped < 0.0f) {
    wrapped += kTwoPi;
  }
  return wrapped - kPi;
}

/** 角度差（目标 − 实际），结果落在 [-π, π)，是云台 / 跟随环的唯一入口 */
inline float AngleDiff(float target, float actual)
{
  return WrapToPi(target - actual);
}

/** 角度是否落在区间内（用于 pitch 软件限位） */
inline bool AngleInRange(float angle, float lower, float upper)
{
  return angle >= lower && angle <= upper;
}

/**
 * @brief       云台系 → 车体系 的平移速度旋转（考核说明 3.2 的硬性约定）
 * @param       vel_gimbal: 云台系平移速度（vx 指向云台朝向），m/s
 * @param       delta: 云台相对底盘的偏航角 δ = 云台朝向 − 车头朝向，rad
 * @retval      车体系平移速度，m/s；w 原样透传
 * @note        文档 3.2 给出的是
 *                vx_c = vx_g·cosδ + vy_g·sinδ
 *                vy_c = −vx_g·sinδ + vy_g·cosδ
 *              该式的符号取决于「δ 的正方向」与「y 轴指向」的约定。实车若发现平移方向
 *              相反，只需把 kFrameDeltaSign 取反，无需改动任何调用点。
 */
inline constexpr float kFrameDeltaSign = 1.0f;

inline ChassisVel GimbalToChassis(const ChassisVel& vel_gimbal, float delta)
{
  const float d = kFrameDeltaSign * WrapToPi(delta);
  const float c = std::cos(d);
  const float s = std::sin(d);
  ChassisVel out;
  out.vx = vel_gimbal.vx * c + vel_gimbal.vy * s;
  out.vy = -vel_gimbal.vx * s + vel_gimbal.vy * c;
  out.w = vel_gimbal.w;
  return out;
}

/* --------------------------- 状态机迁移规则 ------------------------------ */

/** 状态机 / 急停判断的输入汇总（纯数据，便于在 PC 上单测） */
struct LifeInputs {
  bool imu_ready = false;     ///< IMU 零漂已收敛
  bool motors_ready = false;  ///< 电机已使能（达妙已收到使能反馈）
  bool rc_online = false;     ///< 遥控器在线（DBUS 有有效数据）
  bool peer_online = false;   ///< 对方板在线（双板 CAN 帧未超时）
  bool estop = false;         ///< 遥控器处于急停档

  /**
   * @brief       进入「自检(Resurrection)」所需的前提：**不含 motors_ready**
   * @note        自检阶段本来就是"去把电机使能好、等 IMU 收敛"，所以不能要求电机已就绪，
   *              否则 Dead → Resurrection 永远进不去，超时兜底也永远不生效 → 死锁。
   *              （现场真踩过：pitch 电机坏了 → 永远停在 Dead，什么都动不了。）
   */
  bool prerequisitesOk() const
  {
    return imu_ready && rc_online && peer_online && !estop;
  }

  /** 是否满足进入 Working 的全部前提 */
  bool allReady() const
  {
    return prerequisitesOk() && motors_ready;
  }
};

/**
 * @brief       急停判定（放在状态机最外层，任何分支都必须先过这里）
 * @note        急停档或遥控器离线 → 立即全零输出
 */
inline bool EstopActive(const LifeInputs& in)
{
  return in.estop || !in.rc_online;
}

/**
 * @brief       电机就绪判定（含超时兜底），纯函数
 * @param       yaw_ok:        yaw 关节已使能
 * @param       pitch_ok:      pitch 关节已使能
 * @param       require_both:  true = 两轴都要（验收要求），false = 只要 yaw（单轴调试）
 * @param       now_ms:        当前 tick（ms）
 * @param       boot_ms:       上电时刻 tick（ms），0 表示还没记录
 * @param       timeout_ms:    超时时长（ms）
 * @retval      电机是否可以认为「就绪」
 * @note        ⚠️ 超时判定【不能】依赖当前状态机状态。曾经的写法是
 *              "只在 Resurrection 里生效"，结果：等满 2 s 进 Working 的那一拍
 *              ready 又变 false → 立刻被打回 Dead → 每 2 s 循环一次，永远动不了。
 *              改成"上电满了 timeout 且 yaw 已使能"就一直成立。
 */
inline bool MotorsReady(bool yaw_ok, bool pitch_ok, bool require_both, uint32_t now_ms,
                        uint32_t boot_ms, uint32_t timeout_ms)
{
  const bool enabled = require_both ? (yaw_ok && pitch_ok) : yaw_ok;
  if (enabled) {
    return true;
  }
  if (!yaw_ok || (boot_ms == 0u)) {
    return false;
  }
  return static_cast<uint32_t>(now_ms - boot_ms) > timeout_ms;
}

/* ---------------------------- DBUS 帧有效性 ------------------------------ */

/** DBUS 帧长 */
inline constexpr uint16_t kDbusFrameLen = 18u;

/**
 * @brief       判断一帧 DBUS 数据内容是否有效
 * @param       data: 缓冲区
 * @param       size: 收到的字节数
 * @retval      true = 内容合法
 * @note        ⚠️ 只判断"收到 18 字节"远远不够。接收机掉线 / 没对频时，
 *              DMA 缓冲区会保持全 0（或全 FF），而原始值 0 经 (x-1024)/660
 *              会被解码成 -1.55，再被死区钳成 ±1 —— 变成满量程指令。
 *              现场表现就是「云台自己以恒定速度转」（目标角被 6 rad/s 一直积分）。
 *              因此必须校验内容：
 *                1. 不能整帧全 0 / 全 FF
 *                2. 两个拨杆的原始值只能是 1 / 2 / 3
 *              非法帧不得刷新在线状态，这样 100 ms 后会自动离线归零。
 */
inline bool IsDbusFrameValid(const uint8_t* data, uint16_t size)
{
  if ((data == nullptr) || (size < kDbusFrameLen)) {
    return false;
  }
  uint8_t acc = 0u;
  for (uint16_t i = 0; i < kDbusFrameLen; ++i) {
    acc = static_cast<uint8_t>(acc | data[i]);
  }
  if ((acc == 0x00u) || (acc == 0xFFu)) {
    return false;
  }
  /* 拨杆位在 byte5 的高 4 位：左占 bit2-3，右占 bit0-1 */
  const uint8_t sw = data[5];
  const uint8_t left = static_cast<uint8_t>(((sw >> 4) & 0x0Cu) >> 2);
  const uint8_t right = static_cast<uint8_t>((sw >> 4) & 0x03u);
  if ((left < 1u) || (left > 3u) || (right < 1u) || (right > 3u)) {
    return false;
  }
  return true;
}

/**
 * @brief       三态迁移
 * @param       cur: 当前状态
 * @param       in:  本周期输入
 * @retval      下一状态
 * @note        规则：
 *                Dead          → 前提（不含电机）满足则进 Resurrection；电机没就绪也先进自检
 *                Resurrection  → 全部满足则进 Working；前提丢失退回 Dead
 *                Working       → 持续满足则保持；任一丢失立即回 Dead
 *              这些规则保证「电机暂时没响应」不会把状态机锁死，而只是停在自检里等
 *              （RobotModules 里 2 s 超时兜底再放行）。
 *              急停 / 离线由 EstopActive() 单独判断，并在输出层强制归零。
 */
inline LifeState NextLifeState(LifeState cur, const LifeInputs& in)
{
  const bool pre = in.prerequisitesOk();
  const bool ready = in.allReady();
  switch (cur) {
    case LifeState::kDead:
      return pre ? LifeState::kResurrection : LifeState::kDead;
    case LifeState::kResurrection:
      return ready ? LifeState::kWorking : (pre ? LifeState::kResurrection : LifeState::kDead);
    case LifeState::kWorking:
      return ready ? LifeState::kWorking : LifeState::kDead;
    default:
      return LifeState::kDead;
  }
}

/** 状态名（调试 / 日志用） */
inline const char* LifeStateName(LifeState state)
{
  switch (state) {
    case LifeState::kDead:
      return "Dead";
    case LifeState::kResurrection:
      return "Resurrection";
    case LifeState::kWorking:
      return "Working";
    default:
      return "Unknown";
  }
}

/** 底盘模式名（调试 / 日志用） */
inline const char* ChassisModeName(ChassisMode mode)
{
  switch (mode) {
    case ChassisMode::kSeparate:
      return "Separate";
    case ChassisMode::kFollow:
      return "Follow";
    case ChassisMode::kSpin:
      return "Spin";
    default:
      return "Unknown";
  }
}

}  // namespace robot

#endif /* MODULE_STATE_HPP_ */
