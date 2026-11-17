/**
 *******************************************************************************
 * @file      : ins_motor.cpp
 * @brief     : 底盘轮电机实例化（4 × M3508）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 静态实例 + 接线配置项
 *******************************************************************************
 * @attention :
 *  轮序 0=LF 1=LB 2=RB 3=RF，与考核说明 6.2 的表格一致。
 *  下表里的 ID / 方向 / 角度偏移是「待填」项，必须按实车接线标定：
 *    - ID：C620 电调拨码对应的 CAN 反馈 ID = 0x200 + id
 *    - 方向：手转轮子，看 motor->angle() 增减是否与期望一致，不一致就把 kDirFwd
 *      改成 kDirRev
 *    - 角度偏移：把轮子转到机械零点，读 angle()，填进 offset
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_motor.hpp"

#include "chassis_ik_solver.hpp"
#include "motor.hpp"

namespace
{
using hello_world::motor::kDirFwd;
using hello_world::motor::kDirRev;
using hello_world::motor::M3508;
using hello_world::motor::OptionalParams;

constexpr int kWheelCount = static_cast<int>(hello_world::chassis_ik_solver::kWheelCount);

/* ====== 实机接线配置 ======
 * 来源：电路拓扑.png —— 「底盘电机3508，减速比 14，id:1-4」。
 *
 * ⚠️ 待确认：图里写「减速比 14」，而 M3508 P19 出厂是 3591/187 ≈ 19.2。
 *    如果实车确实换了 14:1 减速箱，必须在下面的 OptionalParams 里配置：
 *      - 拆除自带减速器 + 外接 14:1 → remove_build_in_reducer = true, ex_redu_rat = 14
 *      - 自带 19.2 后再串 14:1        → remove_build_in_reducer = false, ex_redu_rat = 14
 *    不配置的话，速度环「目标 rad/s ↔ 实际 rad/s」会差约 1.37 倍，表现为
 *    「给 1 m/s 跑出 0.73 m/s」或反复振荡。
 *
 * 方向 / 角度偏移仍要标定（图里不含这些信息）：
 *    方向：手转轮子看 angle() 增减，不符就把 kDirFwd 改成 kDirRev
 *    偏移：把轮子转到机械零点，读 angle()，填进 offset */
constexpr uint8_t kWheelId[kWheelCount] = {1u, 2u, 3u, 4u};
/* 轮序：0=LF 1=LB 2=RB 3=RF
 *
 * ⚠️ 2026-11-17 实车标定修正：原来四个都是 kDirRev，落地一测就暴露了问题 ——
 *    前推左摇杆，车**原地顺时针转**（不是直行），横推时轮子互相顶住发出堵转声。
 *    原因：麦轮车左右两侧的电机/减速器是**镜像安装**的，"同一个 dir" 在左侧是往前推、
 *    在右侧就是往后推 → 左前右后 → 自转。横移时对角两轮本该反向，方向错的那一对互顶。
 *    判据：前推时车顺时针（往右）转 ⇒ 左侧两轮方向是对的、右侧两轮反了，
 *    所以只把右侧（RB、RF）从 kDirRev 改成 kDirFwd。
 *    改完的验证：落地前推应直行；若要再翻一次（车往后走），把四个一起反过来即可。 */
constexpr hello_world::motor::Dir kWheelDir[kWheelCount] = {
    kDirRev, kDirRev,   /* LF, LB —— 左侧：保持 kDirRev */
    kDirFwd, kDirFwd};  /* RB, RF —— 右侧：镜像安装，方向要反过来 */
constexpr float kWheelAngleOffset[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};

OptionalParams MakeWheelParams(int idx)
{
  OptionalParams opt;
  /* 麦轮速度环用报文原始量（kRaw），输出范围 ±16384 */
  opt.input_type = hello_world::motor::InputType::kRaw;
  opt.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf;
  opt.dir = kWheelDir[idx];
  opt.angle_offset = kWheelAngleOffset[idx];
  return opt;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/

hello_world::motor::Motor* GetMotorWheel(int idx)
{
  /* 函数内静态：首次调用（MainTaskInit）时构造，之后直接取地址，
   * 因此在 CAN 中断里调用也是安全的（-fno-threadsafe-statics 下无守护变量） */
  static M3508 wheels[kWheelCount] = {
      M3508(kWheelId[0], MakeWheelParams(0)),
      M3508(kWheelId[1], MakeWheelParams(1)),
      M3508(kWheelId[2], MakeWheelParams(2)),
      M3508(kWheelId[3], MakeWheelParams(3)),
  };

  if ((idx < 0) || (idx >= kWheelCount)) {
    return nullptr;
  }
  return &wheels[idx];
}

/* Private function definitions ----------------------------------------------*/
