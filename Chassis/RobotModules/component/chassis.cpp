/**
 *******************************************************************************
 * @file      : chassis.cpp
 * @brief     : 底盘运动模块（头文件内联实现，本文件负责编译期契约检查）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 初版
 *******************************************************************************
 * @attention :
 *  robot::Chassis 的实现全部写在 chassis.hpp 里（都是短小的纯逻辑），这样
 *  host 端单测可以直接 include 头文件而不必链接一整个嵌入式工程。
 *  本文件的存在意义是把 chassis.hpp 纳入编译单元，保证它「永远可编译」，
 *  并用 static_assert 把几条接口契约钉死（改了会当场编译失败）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "chassis.hpp"

#include <type_traits>

#include "chassis_ik_solver.hpp"

namespace
{
/* 轮序：0=LF 1=LB 2=RB 3=RF，必须与考核说明 6.2 的表格一致 */
static_assert(hello_world::chassis_ik_solver::kWheelCount == 4u,
              "底盘必须是 4 轮麦轮");

/* 参数结构必须可平凡复制，才能在中断里安全地整体赋值、也便于 host 端测试 */
static_assert(std::is_trivially_copyable<robot::ChassisLimits>::value,
              "ChassisLimits 必须可平凡复制");
static_assert(std::is_trivially_copyable<robot::ChassisVel>::value,
              "ChassisVel 必须可平凡复制");
static_assert(std::is_trivially_copyable<hello_world::chassis_ik_solver::WheelVel>::value,
              "WheelVel 必须可平凡复制");
}  // namespace

/* Private function definitions ----------------------------------------------*/
