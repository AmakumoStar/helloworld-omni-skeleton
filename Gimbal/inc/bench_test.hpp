/**
 * @file    bench_test.hpp
 * @brief   台架自测用的目标发生器（不依赖遥控器）
 * @note    纯算法，可在 PC 上单测。只在 kBenchTest == true 时被引用。
 *
 *          为什么需要它：接收机没数据时整车会停在 Dead，云台不会动，
 *          没法做 PID 阶跃响应测试。这里用一个受控方波代替遥控指令，
 *          并带自动停止（到点回中位），保证"只有有限幅度的往复运动"。
 */
#ifndef GIMBAL_INC_BENCH_TEST_HPP_
#define GIMBAL_INC_BENCH_TEST_HPP_

#include <cstdint>

namespace hello_world {
namespace bench {

/** 自测参数 */
struct Config {
  float amplitude = 0.12f;       ///< 目标偏移幅度，rad（约 6.9°）
  uint32_t half_period_ms = 1500u;  ///< 方波半周期，ms
  uint32_t total_ms = 30000u;    ///< 总时长，超过后回中位并停止，ms
};

/**
 * @brief       方波目标偏移
 * @param       t_ms: 自测开始后的毫秒数
 * @param       c:    参数
 * @retval      目标偏移（rad）；t_ms 超过总时长后返回 0（回中位）
 * @note        取值只有 +amplitude / -amplitude / 0 三种，便于观察阶跃响应。
 */
inline float SquareTarget(uint32_t t_ms, const Config& c)
{
  if ((c.half_period_ms == 0u) || (t_ms >= c.total_ms)) {
    return 0.0f;
  }
  const uint32_t k = t_ms / c.half_period_ms;
  return (((k % 2u) == 0u) ? c.amplitude : -c.amplitude);
}

}  // namespace bench
}  // namespace hello_world

#endif /* GIMBAL_INC_BENCH_TEST_HPP_ */
