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
   * 各留余量：下限 = -0.111 + 余量，上限 = +0.687 - 余量
   *   2026-11-17：余量 5° ⇒ -0.02 / +0.60
   *   2026-11-18：操作手两次要求"收紧范围"（下限抬高、上限减小），逐次各 2°、3°。
   *     ⚠️ 第一次我把"提高"误当成"放宽"，方向做反了；这里按**收紧**改正：
   *   当前：余量 **6°** ⇒ 下限 **-0.008**、上限 **+0.583**
   *     （下限 -0.111+0.1047=-0.0063→取 -0.008；上限 0.687-0.1047=0.5823→取 0.583）
   *     软件行程 0.591 rad（33.9°）；机械行程 0.798 rad（45.7°）
   *   ⚠️ 代价：余量越大，"最后这段没有抵抗"的保护带就越宽（往下拨到下限前的那 6° 是软的，
   *     因为限位保护会把 err 清零）。这不是 PID 的问题，是保护带本身的性质。
   * ⚠️ 原值 -0.35 比机械下限还低 0.24 rad（13.7°）：下半程一直以 1 N·m 死顶机械限位，
   *    既磨结构又骗积分。这也是"pitch 目标被钳在区间外"类问题的来源。 */
  lim.pitch_min = -0.008f;
  lim.pitch_max = 0.583f;
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
  /* ---------- pitch 转矩上限单独放开到 2.0（2026-11-18，操作手确认）----------
   * 原因（实测）：重力 G ≈ 0.60 N·m，静止就占用 1.0 上限的 60%，
   *   于是"向下拨"只剩 0.4 N·m 余量、"向上拨"有 1.6 N·m —— 手感差 4 倍（操作手反馈"向下拨比向上软"）。
   *   放到 2.0 后：向下 1.4 / 向上 2.6（差 1.86 倍），饱和点 5.7° → 11.5°，整体更硬。
   * ⚠️ **只放 pitch，yaw 保持 1.0** —— 11-16 的飞转发生在 yaw 自由轴，没有理由跟着放开。
   * ⚠️ 这是放宽安全边界：必须重新验证「手推不飞 + 静置 30 s 稳定 + 无啸叫/抖」。
   *   三道防线（ki=0、积分限幅、跳变保护）不变；若出现持续顶力矩，先退回 1.5 再排。 */
  lim.max_torque_pitch = 2.0f;
  return lim;
}

/** yaw 位置环参数
 * 实测 kp = 6 会明显震荡：输出转矩上限只有 1 N·m，误差超过 0.17 rad 就饱和，
 * 变成"打满—回弹"的极限环。所以 kp 不能大过约 3~4（饱和点 0.25~0.33 rad）。
 * 2026-11-18：操作手反馈"太软"，由 1.5/0.10 提到 **2.5/0.15**（饱和点 0.40 rad ≈ 23°），
 * 刚度与阻尼都上了一档；最终值见文件末尾 g_yaw_* 的注释。 */
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
 * 验收前请把最终值同步写回 MakeYawConfig() 的默认值。
 *
 * ⚠️ 红线：g_yaw_ki 必须 0；max_torque 保持 1.0 N·m 不动。要"更硬"只能提 kp/kd。
 *
 * 2026-11-18：操作手反馈"yaw 太软"（手一推就动、回正无力）。
 *   kp 1.5 → 2.5：输出饱和点由 max_torque/kp = 1.0/1.5 = 0.667 rad(38°)
 *                 缩到 1.0/2.5 = 0.40 rad(23°) —— 同样 1 N·m 的权限，误差 23° 就打满，
 *                 手感明显变硬，但仍留 23° 的线性区（不会像 kp=6 那样 9.6° 就饱和进极限环）。
 *   kd 0.10 → 0.15：阻尼 +50%，压住回正的过冲（原来一次回正会轻微来回摆）。
 *   下一步阶梯（还软/发振就按这个调）：
 *     硬一点：3.0 / 0.15      再硬：3.0 / 0.20
 *     发振/啸叫：2.0 / 0.15，再不行 1.5 / 0.12（回到原值）
 *   判据：手推 yaw 松手后**一次收敛**、不来回摆、无高频抖；推的过程明显"顶手"。 */
volatile float hello_world::tune::g_yaw_kp = 2.5f;
volatile float hello_world::tune::g_yaw_ki = 0.0f;
volatile float hello_world::tune::g_yaw_kd = 0.15f;

/* pitch 位置环：实车"手拨不够硬"，2026-11-17 由 5.0/0.12 提到 10.0/0.25。
 * ki 仍然为 0（防飞转红线，别加）。 */
volatile float hello_world::tune::g_pitch_kp = 10.0f;
volatile float hello_world::tune::g_pitch_kd = 0.25f;

/* pitch 重力前馈（N·m，正 = 抬头）。详细原因见 pid_tune.hpp 的注释。
 *
 * 2026-11-18 实测定值：**0.60 N·m**
 *   方法：`ki=0` 时稳态 `output().pitch = kp×err`，而总转矩 = 前馈 + output = 重力 G。
 *   测得（前馈 0.30 时）err=0.0328 rad、output=0.304 ⇒ G ≈ 0.30+0.304 = **0.60**。
 *   写 0.60 之后：err 由 1.88° 降到 **0.40°**，output 由 0.304 降到 **0.044**。
 *   残差 0.044 ⇒ 向下/向上转矩余量 0.956 vs 1.044（差 9%，之前差 186%）。
 *   若要更精确可再加 0.04（0.64），收益极小，保持整数便于记忆。
 * ⚠️ 6 N·m 级的前馈是**持续输出**（云台要一直顶住重力），注意电机温升。
 * ⚠️ 目标被限位钳住、误差被清零时该前馈会被跳过（见 robot.cpp 的 at_limit 判断）。 */
volatile float hello_world::tune::g_pitch_grav_ff = 0.60f;
