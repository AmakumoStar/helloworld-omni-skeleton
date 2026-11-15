/**
 *******************************************************************************
 * @file      : ins_robot.cpp
 * @brief     : 底盘板顶层对象实例化（robot::Robot / robot::Chassis）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 静态实例
 *******************************************************************************
 * @attention :
 *  对象的相互注册在 MainTaskInit 里完成，这里只负责「造出来」。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_robot.hpp"

#include "chassis.hpp"
#include "robot.hpp"

/* Exported function definitions ---------------------------------------------*/

robot::Robot* GetRobot()
{
  static robot::Robot robot;
  return &robot;
}

robot::Chassis* GetChassis()
{
  static robot::Chassis chassis;
  return &chassis;
}

/* Private function definitions ----------------------------------------------*/
