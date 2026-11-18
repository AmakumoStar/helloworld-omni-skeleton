/**
 * @file    pid_tune.hpp
 * @brief   运行期可调的 PID 增益（调参用，不需要重新编译）
 * @note    这些变量放在 .data 段，调参时用 J-Link / Ozone 直接写内存即可：
 *            mem32 <addr> 1        （写 IEEE-754 位模式）
 *          每 1 ms 控制周期会把它们写进 yaw 位置环（不清积分状态）。
 *          ⚠️ 验收前请把最终值写回 main_task.cpp 的 MakeYawConfig()，并把
 *             这里的初值同步成一样的，避免"换个板子跑回默认值"。
 */
#ifndef GIMBAL_INC_PID_TUNE_HPP_
#define GIMBAL_INC_PID_TUNE_HPP_

namespace hello_world {
namespace tune {

extern volatile float g_yaw_kp;  ///< yaw 位置环比例
extern volatile float g_yaw_ki;  ///< yaw 位置环积分
extern volatile float g_yaw_kd;  ///< yaw 位置环微分（对误差微分）

/* pitch 位置环（2026-11-17 加）：实车"手拨不够硬"——原来 kp=5，
 * 力矩 1 N·m 上限下误差 11.5° 就饱和，推一点点就让位。
 * 现在默认 kp=10（3° 误差就有 0.52 N·m）、kd=0.25 压住回弹。
 * 注意 ki 仍然必须为 0（防飞转红线），重力静差靠 kp 压，不靠积分。 */
extern volatile float g_pitch_kp;  ///< pitch 位置环比例
extern volatile float g_pitch_kd;  ///< pitch 位置环微分（对误差微分）

/**
 * pitch **重力前馈**（N·m，正 = 抬头方向）。
 *
 * 为什么需要它（2026-11-18 实测现象：手向下拨的恢复力明显弱于向上拨）：
 *   `ki` 被红线锁死为 0，顶住俯仰重力只能靠 P 项 ⇒ 稳态必然下垂 err = G/kp，
 *   于是电机的可用转矩量程是「向下 1−G / 向上 1+G」，两边不对称（G≈0.4 时差 2.3 倍）。
 *   把 G 作为常量前馈补上，PID 就回到 0 附近工作，上下手感对称、下垂消失，
 *   而且**不需要 ki**，不动红线。
 *
 * 怎么定这个数：稳态时 `output().pitch ≈ G`（ki=0 时 PID 输出就等于顶住重力的转矩），
 *   J-Link 读 `Gimbal::output().pitch` 的稳定值即为 G，填到这里。
 * ⚠️ 调大好 → 云台会稳定地"偏高" G_err/kp 度；调小 → 仍下垂。以 ±0.05 为步进。
 * ⚠️ 只在闭环"跟目标"时生效；目标被限位钳住、err 被清零时自动跳过（避免顶机械限位）。
 */
extern volatile float g_pitch_grav_ff;

}  // namespace tune
}  // namespace hello_world

#endif /* GIMBAL_INC_PID_TUNE_HPP_ */
