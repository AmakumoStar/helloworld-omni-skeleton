/**
 *******************************************************************************
 * @file      : ins_comm.cpp
 * @brief     : 遥控器解码对象实例化（DT7，接在云台板）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 静态 DT7 实例
 *******************************************************************************
 * @attention :
 *  DT7 只负责解码；串口收发的接线在 Task/component/comm_task.cpp。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_comm.hpp"

/* Exported function definitions ---------------------------------------------*/

hello_world::remote_control::DT7* GetRemoteControl()
{
  static hello_world::remote_control::DT7 dt7;
  return &dt7;
}

/* Private function definitions ----------------------------------------------*/
