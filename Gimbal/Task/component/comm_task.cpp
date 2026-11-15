/**
 *******************************************************************************
 * @file      : comm_task.cpp
 * @brief     : 云台板通讯层：CAN 收发、DBUS 接收、IWDG 喂狗
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. CanFilter/Start/DMA/喂狗 + 发送
 *******************************************************************************
 * @attention :
 *  ⚠️ 骨架没有调用 HAL_CAN_Start()，这里必须补上，否则收不到任何反馈帧。
 *  ⚠️ 喂狗只放在 CommTask() 一处，禁止放进中断回调。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"

#include "HW_can.hpp"
#include "can.h"
#include "dma.h"
#include "gimbal_chassis_comm.hpp"
#include "ins_all.hpp"
#include "iwdg.h"
#include "motor.hpp"
#include "motor_DaMiao.hpp"
#include "robot.hpp"   /* robot::Robot 完整定义（ins_all 只有前置声明） */
#include "usart.h"

/* USART3 的 DMA 句柄定义在 usart.c（C 链接）；关掉半传输中断，避免一帧 DBUS 被拆坏 */
extern "C" {
extern DMA_HandleTypeDef hdma_usart3_rx;
}

namespace
{
constexpr uint16_t kDbusLen = 18u;

/* ====== CAN 通道分配（由实机接线决定） ====== */
CAN_HandleTypeDef* const kCanMotor = &hcan1;
/* 双板通信也走 CAN1：实测两块板共用同一路 CAN1（CAN1 能收到底盘板的 0x200，
 * 而 CAN2 上一帧都没有），所以 0x210/0x211 必须发在 CAN1 上对面才收得到。 */
CAN_HandleTypeDef* const kCanComm = &hcan1;

/* 两个关节各自走哪一路 CAN：默认都走 hcan1。
 * 如果实车把 pitch 接在另一路（比如云台板 CAN2），把 kCanJointPitch 改成 &hcan2 即可。
 * 判别方法：看 Pitch 的 dm_status_.rotor_temp/mos_temp 是否恒为 0（恒 0 = 一帧都没收到）。 */
CAN_HandleTypeDef* const kCanJointYaw = &hcan1;
CAN_HandleTypeDef* const kCanJointPitch = &hcan1;

/* 调试期开关：把关节帧同时发到 CAN1 和 CAN2。
 * 已用嗅探确认 CAN2 上一帧都没有（pitch 不在这一路），所以现在关掉，减少总线负载。 */
constexpr bool kDuplicateJointsToBothCan = false;

uint8_t g_dbus_rx_buf[kDbusLen] = {0};

}  // namespace

/* Exported function definitions ---------------------------------------------*/

void CommTaskInit(void)
{
  CanFilter_Init(kCanMotor);
  CanFilter_Init(kCanComm);

  (void)HAL_CAN_Start(kCanMotor);
  (void)HAL_CAN_Start(kCanComm);

  if (kCanMotor == &hcan1) {
    (void)HAL_CAN_ActivateNotification(kCanMotor, CAN_IT_RX_FIFO0_MSG_PENDING);
  } else {
    (void)HAL_CAN_ActivateNotification(kCanMotor, CAN_IT_RX_FIFO1_MSG_PENDING);
  }
  if (kCanComm == &hcan1) {
    (void)HAL_CAN_ActivateNotification(kCanComm, CAN_IT_RX_FIFO0_MSG_PENDING);
  } else {
    (void)HAL_CAN_ActivateNotification(kCanComm, CAN_IT_RX_FIFO1_MSG_PENDING);
  }

  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_rx_buf, kDbusLen);
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

  (void)HAL_IWDG_Refresh(&hiwdg);
}

void CommTask(void)
{
  /* ---------- 1. 达妙电机：一发一收，每个关节单独一帧 ---------- */
  hello_world::motor::Motor* joints[2] = {GetMotorYaw(), GetMotorPitch()};
  /* 索引 0 = yaw，1 = pitch；各自走哪一路 CAN 见文件头的 kCanJointYaw / kCanJointPitch */
  CAN_HandleTypeDef* const joint_can[2] = {kCanJointYaw, kCanJointPitch};
  for (int i = 0; i < 2; ++i) {
    if (joints[i] == nullptr) {
      continue;
    }
    uint8_t buf[8] = {0u};
    if (joints[i]->encode(buf)) {
      /* MIT 模式发送 ID = 电机 ID；使能/寄存器命令的 ID 由 encode 内部更新，
       * 所以必须用 tx_id() 取，而不是自己算。 */
      hello_world::motor::DaMiao* dm = static_cast<hello_world::motor::DaMiao*>(joints[i]);
      CAN_Send_Msg(joint_can[i], buf, dm->tx_id(), 8u);
      if (kDuplicateJointsToBothCan) {
        CAN_HandleTypeDef* const other =
            (joint_can[i] == &hcan1) ? &hcan2 : &hcan1;
        CAN_Send_Msg(other, buf, dm->tx_id(), 8u);
      }
    }
  }

  /* ---------- 2. 双板通信帧 ---------- */
  robot::GimbalChassisComm* comm = GetGimbalChassisComm();
  if (comm != nullptr) {
    uint8_t comm_frame[robot::kCommFrameLen] = {0u};
    comm->encode(comm_frame);
    CAN_Send_Msg(kCanComm, comm_frame, comm->tx_id(), robot::kCommFrameLen);
  }

  /* ---------- 3. 喂狗（唯一入口） ---------- */
  (void)HAL_IWDG_Refresh(&hiwdg);
}

/* Private function definitions ----------------------------------------------*/

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
  if ((huart != nullptr) && (huart->Instance == USART3)) {
    /* 只有内容合法才解码并刷新在线状态；非法帧不刷新 → 100 ms 后离线归零 */
    if (robot::IsDbusFrameValid(g_dbus_rx_buf, Size)) {
      hello_world::remote_control::DT7* rc = GetRemoteControl();
      if (rc != nullptr) {
        rc->decode(g_dbus_rx_buf);
      }
      robot::Robot* robot = GetRobot();
      if (robot != nullptr) {
        robot->NotifyRemoteFrame();
      }
    }
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_rx_buf, kDbusLen);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  }
}
