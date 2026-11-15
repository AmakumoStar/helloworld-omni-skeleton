/**
 *******************************************************************************
 * @file      : ins_chassis_gimbal_comm.cpp
 * @brief     : 双板通讯对象实例化（底盘板侧）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 静态实例 + CAN ID 绑定
 *******************************************************************************
 * @attention :
 *  两个 ID 来自 RobotComponents/component/module_state.hpp，两板必须一致：
 *    C2G = 0x210（底盘发、云台收）
 *    G2C = 0x211（云台发、底盘收）
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
  static robot::GimbalChassisComm comm(robot::GimbalChassisComm::CodePart::Chassis,
                                       robot::kCanIdChassisToGimbal,
                                       robot::kCanIdGimbalToChassis);
  return &comm;
}

/* Private function definitions ----------------------------------------------*/
