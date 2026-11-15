/**
 *******************************************************************************
 * @file      : ins_robot.cpp
 * @brief     : 云台板顶层对象实例化（robot::Robot / robot::Gimbal）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 静态实例
 *******************************************************************************
 * @attention :
 *  注册关系在 MainTaskInit 里建立，这里只负责造对象。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_robot.hpp"

#include "gimbal.hpp"
#include "robot.hpp"

/* Exported function definitions ---------------------------------------------*/

robot::Robot* GetRobot()
{
  static robot::Robot robot;
  return &robot;
}

robot::Gimbal* GetGimbal()
{
  static robot::Gimbal gimbal;
  return &gimbal;
}

/* Private function definitions ----------------------------------------------*/
