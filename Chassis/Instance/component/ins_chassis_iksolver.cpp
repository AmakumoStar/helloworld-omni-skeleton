/**
 *******************************************************************************
 * @file      : ins_chassis_iksolver.cpp
 * @brief     : 底盘逆解器实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 静态实例
 *******************************************************************************
 * @attention :
 *  几何参数在 MainTaskInit 里写入（与 robot::Chassis 使用同一组数值）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_chassis_iksolver.hpp"

#include "chassis_ik_solver.hpp"

/* Exported function definitions ---------------------------------------------*/

hello_world::chassis_ik_solver::ChassisIkSolver* GetChassisIkSolver()
{
  static hello_world::chassis_ik_solver::ChassisIkSolver solver;
  return &solver;
}

/* Private function definitions ----------------------------------------------*/
