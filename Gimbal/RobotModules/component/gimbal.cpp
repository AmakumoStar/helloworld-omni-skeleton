/**
 *******************************************************************************
 * @file      : gimbal.cpp
 * @brief     : 云台运动模块（头文件内联实现，本文件负责编译期契约检查）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 初版
 *******************************************************************************
 * @attention :
 *  robot::Gimbal 的实现写在 gimbal.hpp 里（角度处理 + PID 调用都是短逻辑），
 *  这样 host 端单测可以直接 include 头文件验证角度回绕与限位。
 *  本文件负责把 gimbal.hpp 纳入编译单元并钉死接口契约。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "gimbal.hpp"

#include <type_traits>

namespace
{
/* MIT 增益上限必须与 DM-J4310 手册一致，改错会直接烧云台 */
static_assert(robot::kMitKpMax == 500.0f, "MIT kp 上限必须为 500");
static_assert(robot::kMitKdMax == 5.0f, "MIT kd 上限必须为 5");
static_assert(robot::kMitKdMin > 0.0f, "位置控制时 kd 不能为 0");

static_assert(std::is_trivially_copyable<robot::GimbalPos>::value,
              "GimbalPos 必须可平凡复制");
static_assert(std::is_trivially_copyable<robot::GimbalLimits>::value,
              "GimbalLimits 必须可平凡复制");
}  // namespace

/* Private function definitions ----------------------------------------------*/
