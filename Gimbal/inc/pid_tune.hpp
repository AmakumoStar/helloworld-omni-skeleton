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

}  // namespace tune
}  // namespace hello_world

#endif /* GIMBAL_INC_PID_TUNE_HPP_ */
