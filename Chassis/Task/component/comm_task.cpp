/**
 *******************************************************************************
 * @file      : comm_task.cpp
 * @brief     : 底盘板通讯层：CAN 收发、DBUS 接收、IWDG 喂狗
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. CanFilter/Start/DMA/喂狗 + 发送
 *******************************************************************************
 * @attention :
 *  ⚠️ 骨架只提供了 CanFilter_Init/CAN_Send_Msg 和接收回调，**没有调用
 *  HAL_CAN_Start()**：不启动 CAN 模块的话，滤波器和回调配得再对也收不到一帧。
 *  这里把 Start + 首次 ActivateNotification 补上。
 *
 *  ⚠️ 喂狗只有一个入口：CommTask()。不要放到 MainTask、更不要放到 CAN/UART 回调里，
 *  否则主循环卡死时仍会被喂狗，验收的「屏蔽喂狗必须复位」就会失败。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"

#include "HW_can.hpp"
#include "can.h"
#include "chassis_ik_solver.hpp"
#include "dma.h"
#include "gimbal_chassis_comm.hpp"
#include "ins_all.hpp"
#include "iwdg.h"
#include "motor.hpp"
#include "robot.hpp"   /* robot::Robot 完整定义（ins_all 只有前置声明） */
#include "usart.h"

/* USART3 的 DMA 句柄定义在 usart.c（C 链接）；这里取用是为了关掉半传输中断，
 * 否则 DMA 收满 9 字节就会触发回调并重启接收，把一帧 DBUS 拆坏。 */
extern "C" {
extern DMA_HandleTypeDef hdma_usart3_rx;
}

namespace
{
/* DBUS 帧长 */
constexpr uint16_t kDbusLen = 18u;

/* ====== CAN 通道分配（由实机接线决定，改这两行即可） ====== */
CAN_HandleTypeDef* const kCanMotor = &hcan1;  ///< 电机（M3508 / 达妙）
/* 双板通信也走 CAN1：实测（云台板嗅探）两块板共用同一路 CAN1，
 * CAN1 上能收到底盘板的 0x200，而 CAN2 上一帧都没有。 */
CAN_HandleTypeDef* const kCanComm = &hcan1;   ///< 双板通信

/* DBUS 接收缓冲区（DMA 直接写入） */
uint8_t g_dbus_rx_buf[kDbusLen] = {0};

}  // namespace

/* Exported function definitions ---------------------------------------------*/

void CommTaskInit(void)
{
  /* ---------- CAN：滤波器 + 启动 + 首次使能接收中断 ---------- */
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

  /* ---------- DBUS：空闲 + DMA 接收 18 字节 ---------- */
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_rx_buf, kDbusLen);
  /* 关掉 DMA 半传输中断，避免同一个缓冲被回调两次 */
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

  /* ---------- IWDG：main.c 已 MX_IWDG_Init 启动，这里先喂一次 ---------- */
  (void)HAL_IWDG_Refresh(&hiwdg);
}

void CommTask(void)
{
  /* ---------- 1. 电机控制帧 ---------- */
  /* 4 个 M3508 同属 0x200 一帧：逐个 encode 写入自己的 2 字节槽位，最后发一帧 */
  uint8_t frame[8] = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
  bool has_motor = false;
  for (uint8_t i = 0; i < hello_world::chassis_ik_solver::kWheelCount; ++i) {
    hello_world::motor::Motor* motor = GetMotorWheel(static_cast<int>(i));
    if (motor != nullptr) {
      (void)motor->encode(frame);
      has_motor = true;
    }
  }
  if (has_motor) {
    /* 调试期"全打"策略：同时覆盖两种未知
     *   1) 电调挂在哪一路 CAN —— CAN1 和 CAN2 都发
     *      （云台板嗅探显示 CAN1 上只有 0x200，从没出现过 0x201~0x204 反馈）
     *   2) 电调 ID 是 1~4 还是 5~8 —— 0x200 和 0x1FF 都发
     *      （ID 不对时症状和"没供电"完全一样：不发反馈、不收指令）
     * 确认接线与 ID 之后，把不需要的组合删掉即可。 */
    const uint32_t tx_ids[2] = {hello_world::motor::M3508::GetTxId(1u),   /* 0x200，ID 1~4 */
                                hello_world::motor::M3508::GetTxId(5u)};  /* 0x1FF，ID 5~8 */
    CAN_HandleTypeDef* const other_bus = (kCanMotor == &hcan1) ? &hcan2 : &hcan1;
    for (uint8_t b = 0; b < 2u; ++b) {
      CAN_HandleTypeDef* const bus = (b == 0u) ? kCanMotor : other_bus;
      for (uint8_t t = 0; t < 2u; ++t) {
        CAN_Send_Msg(bus, frame, tx_ids[t], 8u);
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

  /* ---------- 3. 喂狗 ----------
   * 只有「本周期整条链路跑完」才喂：update() 在 MainTask 里已先执行，
   * 这里能走到就说明控制链路没有卡死。 */
  (void)HAL_IWDG_Refresh(&hiwdg);
}

/* Private function definitions ----------------------------------------------*/

/* ===== 调试计数：区分「串口根本没数据」和「接收机在发空帧」 =====
 *   g_dbus_rx_events：DMA 空闲事件次数（串口收到过多少"帧边界"）
 *   g_dbus_rx_valid ：其中内容通过校验的次数
 * 两者都停在 0        → 接收机没接 / 没供电 / 引脚不对
 * events 涨、valid 不涨 → 有数据但内容非法（没对频 / 遥控器关着）
 */
volatile uint32_t g_dbus_rx_events = 0u;
volatile uint32_t g_dbus_rx_valid = 0u;

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
  if ((huart != nullptr) && (huart->Instance == USART3)) {
    g_dbus_rx_events = g_dbus_rx_events + 1u;
    /* 收满 18 字节**且内容合法**才解码并刷新在线状态；
     * 内容非法时不刷新 → 100 ms 后 rc_online 变 0 → 失效归零。 */
    if (robot::IsDbusFrameValid(g_dbus_rx_buf, Size)) {
      g_dbus_rx_valid = g_dbus_rx_valid + 1u;
      hello_world::remote_control::DT7* rc = GetRemoteControl();
      if (rc != nullptr) {
        rc->decode(g_dbus_rx_buf);
      }
      robot::Robot* robot = GetRobot();
      if (robot != nullptr) {
        robot->NotifyRemoteFrame();
      }
    }
    /* 重新启动下一轮接收（不重启就只能收到一帧） */
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_rx_buf, kDbusLen);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  }
}
