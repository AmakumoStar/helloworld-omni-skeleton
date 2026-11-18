/**
 *******************************************************************************
 * @file      : gimbal.hpp
 * @brief     : 云台运动模块：yaw / pitch 关节空间位置环（角度回绕 + 软件限位）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 初版：外置 PID → 转矩，MIT 下发
 *******************************************************************************
 * @attention :
 *  1. 位置环 PID 用的是自己写的 hello_world::pid::Pid，输出为转矩（N·m），
 *     再通过达妙 MIT 接口下发（kp = 0、kd = 0 时 MIT 退化为纯转矩输出）。
 *  2. 若改用驱动器内部位置环（MIT 的 kp / kd），用 SetMitGains() 写入，
 *     该函数会把 kp 钳到 [0, 500]、kd 钳到 (0, 5]，符合 DM-J4310 手册限制。
 *  3. yaw 支持连续旋转：所有角度差必须先经 robot::AngleDiff() 归一化到 [-π, π)，
 *     否则 359° → 1° 会被当成 −358° 反向大转。
 *  4. pitch 有软件限位，目标与指令都要钳位，防止顶死堵转。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_ROBOT_MODULES_GIMBAL_HPP_
#define GIMBAL_ROBOT_MODULES_GIMBAL_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <cstdint>

#include "module_state.hpp"
#include "pid.hpp"

namespace robot
{
/* Exported constants --------------------------------------------------------*/

/** DM-J4310 MIT 模式参数上限（见电机手册） */
inline constexpr float kMitKpMax = 500.0f;
inline constexpr float kMitKdMax = 5.0f;
/** 位置控制时 kd 不能为 0，给一个安全下限 */
inline constexpr float kMitKdMin = 0.01f;

/** 云台限幅与增益参数（数值待实机整定） */
struct GimbalLimits {
  float pitch_min = -0.35f;      ///< pitch 软件下限，rad
  float pitch_max = 0.60f;       ///< pitch 软件上限，rad
  float max_yaw_rate = 6.0f;     ///< 摇杆最大 yaw 角速度，rad/s
  float max_pitch_rate = 3.0f;   ///< 摇杆最大 pitch 角速度，rad/s
  /* ===== 两轴转矩上限**分开**（2026-11-18）=====
   * max_torque 管 **yaw**（保持 1.0 N·m，历史红线：11-16 的飞转发生在 yaw 自由轴）；
   * max_torque_pitch 管 **pitch**（2.0 N·m）。
   * 为什么 pitch 要更大：pitch 是**重力轴**，静止就要拿 ~0.6 N·m 顶住（占用 1.0 上限的 60%），
   *   于是"向下拨"只剩 0.4 N·m 余量、"向上拨"有 1.6 N·m —— 手感差 4 倍。
   *   提到 2.0 后变成 1.4 vs 2.6（差 1.86 倍），同时饱和点 5.7° → 11.5°，整体更硬。
   * ⚠️ 两个上限都必须 ≥ 各自的重力/扰动所需，否则又会退回"向下软"。
   * ⚠️ 改这个值等于放宽安全边界：改完必须重新做"手推不飞 + 静置 30 s"验证。 */
  float max_torque = 3.0f;       ///< **yaw** 位置环输出转矩上限，N·m
  float max_torque_pitch = 3.0f; ///< **pitch** 位置环输出转矩上限，N·m
};

/* Exported functions ------------------------------------------------------- */

/** MIT kp 限幅 */
inline float ClampMitKp(float kp) { return Clamp(kp, 0.0f, kMitKpMax); }

/** MIT kd 限幅：位置控制时不能为 0 */
inline float ClampMitKd(float kd) { return Clamp(kd, kMitKdMin, kMitKdMax); }

/* Exported types ------------------------------------------------------------*/

class Gimbal
{
 public:
  Gimbal(void) = default;
  ~Gimbal(void) = default;

  /**
   * @brief       初始化限幅参数
   * @param       limits: pitch 限位 / 摇杆速率上限 / 转矩上限
   * @retval      None
   */
  void Init(const GimbalLimits& limits)
  {
    limits_ = limits;
    Stop();
  }

  void set_limits(const GimbalLimits& limits) { limits_ = limits; }
  const GimbalLimits& limits(void) const { return limits_; }

  /**
   * @brief       写入 MIT 模式增益（仅在改用驱动器内部位置环时使用）
   * @param       kp: 位置环比例，钳到 [0, 500]
   * @param       kd: 速度环比例，钳到 (0, 5]
   */
  void SetMitGains(float kp, float kd)
  {
    mit_kp_ = ClampMitKp(kp);
    mit_kd_ = ClampMitKd(kd);
  }
  float mit_kp(void) const { return mit_kp_; }
  float mit_kd(void) const { return mit_kd_; }

  /**
   * @brief       设置绝对目标关节角
   * @param       target: yaw / pitch，rad
   * @note        yaw 不做折叠（连续旋转靠累计），pitch 会被钳到限位内
   */
  void SetTarget(const GimbalPos& target)
  {
    target_.yaw = target.yaw;
    target_.pitch = Clamp(target.pitch, limits_.pitch_min, limits_.pitch_max);
  }

  /**
   * @brief       相对增量（遥控摇杆按速率积分时使用）
   * @param       delta: 本周期增量，rad
   */
  void MoveBy(const GimbalPos& delta)
  {
    target_.yaw += delta.yaw;
    target_.pitch = Clamp(target_.pitch + delta.pitch, limits_.pitch_min, limits_.pitch_max);
  }

  /** 零指令（急停 / 非 Working）：目标跟随当前角度，输出清零 */
  void Stop(void)
  {
    output_.setZero();
    err_.setZero();
    /* 状态切换 / 初始化之后目标会被重对齐到当前角度（Robot::Reset），那是**合法**的
     * 大跳变，所以下一拍只记录基准、不判跳变。见 Update() 的保护策略 ①。 */
    primed_ = false;
  }

  /**
   * @brief       1ms 周期：角度差归一化 → PID → 转矩
   * @param       dt: 周期，单位 s
   * @param       yaw_feedback: 当前 yaw【绝对角】（IMU 相对开机姿态），单位 rad
   * @param       pitch_feedback: 当前 pitch 关节角（相对车体），单位 rad
   * @retval      None
   * @note        yaw 必须用【绝对角】闭环：若用关节角，底盘一转动云台就跟着车体一起转，
   *              结果是「跟随」永远追不上、「小陀螺」的枪口跟着车转，两者都失效。
   *              关节角闭环只能让云台"相对车体不动"，而考核要求的是"相对地面指稳"。
   *              pitch 是相对车体的俯仰，与车体偏航无关，用关节角即可。
   */
  void Update(float dt, float yaw_feedback, float pitch_feedback)
  {
    /* ========== 保护策略（2026-11-17 改，依据实车实测） ==========
     * 旧写法：|err| > 34° → 清积分 + 清输出 + 把 err_ 也写成 0。
     * 实测后果（手拧云台 >34° 后完全没力，回推 30° 力才回来）：
     *   清输出 → 关节失去**唯一**能让误差回落的手段 → 误差永远 >34° →
     *   下一拍继续清零 ⇒ 物理自锁，云台从此撒手，只有外力把它推回 34° 内才解。
     *   而且把 err_ 写成 0 会掩盖真实误差，排查时读到假 0。
     * 现在改成两条互补的判据，既能防飞车、又不会永久撒手：
     *   ① 单拍**跳变**检测：目标或反馈在 1 ms 内跳 > kMaxJumpRad（11.5°）。
     *      物理上不可能（摇杆 2 rad/s ⇒ 0.002 rad/拍；手拧最快也就 ~0.01 rad/拍），
     *      只有「信号失效」才会出现（目标被外部写入 / IMU 异常 / 反馈跳变）。
     *      此时把目标重对齐到当前实际角度（离合器），停手，不追假目标。
     *   ② 误差 > 34° 时**只清积分**（红线原意），不清输出、不改 err_。
     *      防飞转仍由「ki = 0 + 积分限幅 ±0.15 + 本条清积分」三条守住。 */

    /* ---------- ① 跳变检测（先判信号是否可信，再谈控制） ---------- */
    if (!primed_) {
      prev_target_ = target_;
      prev_yaw_fb_ = yaw_feedback;
      prev_pitch_fb_ = pitch_feedback;
      primed_ = true;
    } else {
      /* 必须用 AngleDiff 算变化量：yaw 反馈在 ±π 处回绕，直接相减会被当成跳变 */
      const float d_tgt_y = AngleDiff(target_.yaw, prev_target_.yaw);
      const float d_fb_y = AngleDiff(yaw_feedback, prev_yaw_fb_);
      const float d_tgt_p = AngleDiff(target_.pitch, prev_target_.pitch);
      const float d_fb_p = AngleDiff(pitch_feedback, prev_pitch_fb_);

      prev_target_ = target_;
      prev_yaw_fb_ = yaw_feedback;
      prev_pitch_fb_ = pitch_feedback;

      if ((std::fabs(d_tgt_y) > kMaxJumpRad) || (std::fabs(d_fb_y) > kMaxJumpRad) ||
          (std::fabs(d_tgt_p) > kMaxJumpRad) || (std::fabs(d_fb_p) > kMaxJumpRad)) {
        /* 离合器：放弃旧目标，把目标钉到当前实际位置，本拍不驱动 */
        target_.yaw = yaw_feedback;
        target_.pitch = Clamp(pitch_feedback, limits_.pitch_min, limits_.pitch_max);
        yaw_pid_.Reset();
        pitch_pid_.Reset();
        err_.setZero();
        output_.setZero();
        prev_target_ = target_;
        ++jump_cnt_;
        return;
      }
    }

    /* 角度差必须先归一化，这是 yaw 连续旋转的关键 */
    err_.yaw = AngleDiff(target_.yaw, yaw_feedback);
    err_.pitch = AngleDiff(target_.pitch, pitch_feedback);

    /* ---------- ② 红线：误差过大只清积分 ----------
     * 只 Reset()（清积分/微分历史），**不清输出、不改 err_** —— 理由见本函数开头。 */
    constexpr float kMaxTrackErr = 0.60f;  /* rad，约 34° */
    if ((err_.yaw > kMaxTrackErr) || (err_.yaw < -kMaxTrackErr)) {
      yaw_pid_.Reset();
    }
    if ((err_.pitch > kMaxTrackErr) || (err_.pitch < -kMaxTrackErr)) {
      pitch_pid_.Reset();
    }

    /* pitch 目标已在 SetTarget 里钳位；反馈越界时不再继续加油门 */
    if (pitch_feedback >= limits_.pitch_max && err_.pitch > 0.0f) {
      err_.pitch = 0.0f;
    }
    if (pitch_feedback <= limits_.pitch_min && err_.pitch < 0.0f) {
      err_.pitch = 0.0f;
    }

    float yaw_out = yaw_pid_.UpdateErr(err_.yaw, dt);
    float pitch_out = pitch_pid_.UpdateErr(err_.pitch, dt);

    output_.yaw = Clamp(yaw_out, -limits_.max_torque, limits_.max_torque);
    output_.pitch = Clamp(pitch_out, -limits_.max_torque_pitch, limits_.max_torque_pitch);
  }

  /* 数据获取 */
  const GimbalPos& target(void) const { return target_; }
  const GimbalPos& error(void) const { return err_; }
  /** 本周期输出（转矩，N·m） */
  const GimbalPos& output(void) const { return output_; }
  hello_world::pid::Pid& yaw_pid(void) { return yaw_pid_; }
  hello_world::pid::Pid& pitch_pid(void) { return pitch_pid_; }

  /** 目标是否落在 pitch 限位内（调试用） */
  bool targetInRange(void) const
  {
    return AngleInRange(target_.pitch, limits_.pitch_min, limits_.pitch_max);
  }

  /** 跳变保护触发次数（只读调试量：写入后不清零，看它涨就说明判过假信号） */
  uint32_t jump_cnt(void) const { return jump_cnt_; }

 private:
  GimbalLimits limits_{};                 ///< 限幅参数
  GimbalPos target_{};                    ///< 目标关节角
  GimbalPos err_{};                       ///< 归一化后的角度差
  GimbalPos output_{};                    ///< 转矩输出
  float mit_kp_ = 0.0f;                   ///< MIT 内部位置环 kp（默认走外置 PID）
  float mit_kd_ = 0.0f;                   ///< MIT 内部位置环 kd
  hello_world::pid::Pid yaw_pid_{};       ///< yaw 位置环
  hello_world::pid::Pid pitch_pid_{};     ///< pitch 位置环

  /* ---------- 跳变检测（见 Update() 的保护策略 ①） ----------
   * 门限 0.20 rad ≈ 11.5°/拍 = 200 rad/s。物理上不可能：
   *   摇杆积分 2 rad/s ⇒ 0.002 rad/拍；手拧云台最快 ~10 rad/s ⇒ 0.01 rad/拍；
   *   底盘自旋 1.5 rad/s ⇒ 0.0015 rad/拍。
   * 真实的信号失效（目标被 J-Link/上位机写错、IMU 异常、反馈跳变）通常是 0.5 rad 以上，
   * 所以 0.2 既不会误判物理扰动，又能挡住假信号。 */
  static constexpr float kMaxJumpRad = 0.20f;
  GimbalPos prev_target_{};    ///< 上一拍目标（跳变检测基准）
  float prev_yaw_fb_ = 0.0f;   ///< 上一拍 yaw 反馈
  float prev_pitch_fb_ = 0.0f; ///< 上一拍 pitch 反馈
  bool primed_ = false;        ///< 是否已有基准；Stop() 里清掉，下一拍只记录不判
  uint32_t jump_cnt_ = 0u;     ///< 跳变保护触发次数
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */
