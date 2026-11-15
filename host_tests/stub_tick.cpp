/**
 *******************************************************************************
 * @file      : stub_tick.cpp
 * @brief     : host 测试用的时间基准桩（替换需要 SysTick 的真实实现）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 初版
 *******************************************************************************
 * @attention :
 *  这样 GimbalChassisComm::isOffline() 就可以在 PC 上被测到，
 *  测试里用 AdvanceMs() 手动推进时间。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#include <cstdint>

namespace hello_world
{
namespace tick
{
namespace
{
uint32_t g_ms = 0u;
}

uint64_t GetTickUs(void)
{
  return static_cast<uint64_t>(g_ms) * 1000ull;
}

uint32_t GetTickMs(void)
{
  return g_ms;
}

uint32_t GetTickS(void)
{
  return g_ms / 1000u;
}

void DelayUs(uint32_t us)
{
  (void)us;
}

/** 测试专用：推进时间 */
void AdvanceMs(uint32_t ms)
{
  g_ms += ms;
}

/** 测试专用：重置时间 */
void ResetMs(void)
{
  g_ms = 0u;
}

}  // namespace tick
}  // namespace hello_world
