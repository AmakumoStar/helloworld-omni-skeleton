/**
 *******************************************************************************
 * @file      : ins_chassis_gimbal_comm.cpp
 * @brief     : 双板通讯对象实例化（云台板侧）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 静态实例 + CAN ID 绑定
 *******************************************************************************
 * @attention :
 *  云台板的收发方向与底盘板相反：
 *    tx = 0x211（G2C）  rx = 0x210（C2G）
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_chassis_gimbal_comm.hpp"

#include "gimbal_chassis_comm.hpp"
#include "module_state.hpp"

/* Exported function definitions ---------------------------------------------*/

robot::GimbalChassisComm* GetGimbalChassisComm()
{
  static robot::GimbalChassisComm comm(robot::GimbalChassisComm::CodePart::Gimbal,
                                       robot::kCanIdGimbalToChassis,
                                       robot::kCanIdChassisToGimbal);
  return &comm;
}

/* Private function definitions ----------------------------------------------*/
