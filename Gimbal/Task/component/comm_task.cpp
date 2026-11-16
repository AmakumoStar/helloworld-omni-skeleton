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

/* 两个关节各自走哪一路 CAN（实车实测结果）：
 *   yaw   → CAN1：反馈是 0x012，CAN1 上能看到
 *   pitch → CAN2：反馈是 0x011，**只在 CAN2 上出现**
 * 判别方法：看该关节的 dm_status_.rotor_temp/mos_temp 是否恒为 0（恒 0 = 一帧都没收到）。 */
CAN_HandleTypeDef* const kCanJointYaw = &hcan1;
CAN_HandleTypeDef* const kCanJointPitch = &hcan1;

/* 调试期开关：把关节帧同时发到 CAN1 和 CAN2（实测：pitch 只有双发时才回应，暂保持 true）。
 * ⚠️ 教训：曾经因为「嗅探确认 CAN2 没帧」把它关掉，但那时 CAN2 根本没 Start，
 * 这是循环论证。现在两路都启动了，pitch 在 CAN2 上工作正常，不需要双发。 */
constexpr bool kDuplicateJointsToBothCan = true;

uint8_t g_dbus_rx_buf[kDbusLen] = {0};

}  // namespace

/* Exported function definitions ---------------------------------------------*/

void CommTaskInit(void)
{
  /* ---------- CAN：两路都必须「滤波器 + Start + 接收中断」 ----------
   * ⚠️ 曾经的 bug（与底盘板同一个）：这里只对 kCanMotor / kCanComm 做初始化，
   *    而两者都指向 &hcan1，于是 hcan2 从未 Start。
   *    后果：若某个关节（如 pitch）接在 CAN2 上，就会出现
   *      「指令发不到它」+「它的反馈永远收不到」，现象和「电机没接」完全一样。
   *    注意：CAN2 没启动时嗅探表里 CAN2 恒为 0 帧，
   *    拿它当「CAN2 上没有设备」的证据是循环论证。 */
  CAN_HandleTypeDef* const can_buses[2] = {&hcan1, &hcan2};

  /* ① 先把【两路】滤波器都配好，再去 Start 任何一路。
   * ⚠️ 关键顺序：滤波器寄存器位于 CAN1 的寄存器区，只有 CAN1 处于初始化模式时才写得进。
   *    如果先把 CAN1 Start 起来再配 CAN2，CAN2 的 FilterBank/FIFO 分配会被静默丢弃，
   *    退回默认值（FIFO0）——现象就是「总线明显有帧，但我们一帧都收不到」。
   *    本工程真踩过这个坑：CAN2 的 FFA1R 读到 0（FIFO0），而代码按 FIFO1 去处理。 */
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);

  /* ② 再启动两路（某一路没接线也不影响另一路） */
  for (uint8_t i = 0; i < 2u; ++i) {
    (void)HAL_CAN_Start(can_buses[i]);
  }

  /* ③ 最后开接收中断：两个 FIFO 都开，不去猜滤波器把帧放进了哪个 FIFO */
  for (uint8_t i = 0; i < 2u; ++i) {
    const uint32_t fifo_it = CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING;
    if (HAL_CAN_ActivateNotification(can_buses[i], fifo_it) != HAL_OK) {
      /* HAL 内部要求 State==READY，不满足会静默失败：直接置位兜底 */
      __HAL_CAN_ENABLE_IT(can_buses[i], CAN_IT_RX_FIFO0_MSG_PENDING);
      __HAL_CAN_ENABLE_IT(can_buses[i], CAN_IT_RX_FIFO1_MSG_PENDING);
    }
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
