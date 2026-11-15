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
  float max_torque = 3.0f;       ///< 位置环输出转矩上限，N·m（留余量，避免打满）
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
    /* 角度差必须先归一化，这是 yaw 连续旋转的关键 */
    err_.yaw = AngleDiff(target_.yaw, yaw_feedback);
    err_.pitch = AngleDiff(target_.pitch, pitch_feedback);

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
    output_.pitch = Clamp(pitch_out, -limits_.max_torque, limits_.max_torque);
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

 private:
  GimbalLimits limits_{};                 ///< 限幅参数
  GimbalPos target_{};                    ///< 目标关节角
  GimbalPos err_{};                       ///< 归一化后的角度差
  GimbalPos output_{};                    ///< 转矩输出
  float mit_kp_ = 0.0f;                   ///< MIT 内部位置环 kp（默认走外置 PID）
  float mit_kd_ = 0.0f;                   ///< MIT 内部位置环 kd
  hello_world::pid::Pid yaw_pid_{};       ///< yaw 位置环
  hello_world::pid::Pid pitch_pid_{};     ///< pitch 位置环
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */
