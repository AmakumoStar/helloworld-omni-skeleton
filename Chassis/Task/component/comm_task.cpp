/**
 *******************************************************************************
 * @file      : comm_task.cpp
 * @brief     : 底盘板通讯层：CAN 收发、DBUS 接收、IWDG 喂狗
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. CanFilter/Start/DMA/喂狗 + 发送
 *  V1.1.0      2026-11-16      Hello World     2. 修复 hcan2 从未 Start
 *                                              3. DBUS 三路探测（USART1/3/6）
 *                                              4. 补 HAL_UART_ErrorCallback
 *******************************************************************************
 * @attention :
 *  ⚠️ 骨架只提供了 CanFilter_Init/CAN_Send_Msg 和接收回调，**没有调用
 *  HAL_CAN_Start()**：不启动 CAN 模块的话，滤波器和回调配得再对也收不到一帧。
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

namespace
{
/* DBUS 帧长 */
constexpr uint16_t kDbusLen = 18u;
/* 流缓冲：空闲中断一次收多少字节。DBUS 真实帧前面带 0x20 帧头（长度不是 18），
 * 按固定 18 字节接收会让窗口逐帧漂移，现象是「帧收到了但解析出的拨杆值乱跳」。
 * 所以改成收一大块，再在块内搜索合法帧，不依赖对齐。 */
constexpr uint16_t kDbusStreamLen = 64u;

/* ====== CAN 通道分配（由实机接线决定） ====== */
CAN_HandleTypeDef* const kCanMotor = &hcan1;  ///< 电机（M3508 / 达妙）
/* 双板通信也走 CAN1：实测两块板共用同一路 CAN1。 */
CAN_HandleTypeDef* const kCanComm = &hcan1;   ///< 双板通信

/* ====== DBUS 三路探测 ======
 * 为什么是三路：接收机到底插在板子的哪个串口上并不确定（不同批次/线束不同），
 * 而「插错口」和「极性反了」在现象上几乎一样（都收不到合法帧）。
 * 这里让 USART1/3/6 全部按 DBUS 电气参数（100 kbaud / 9 位 / 偶校验）接收，
 * 分别统计，就能一次定位到底是「哪个口有信号」。
 *
 *   索引 0 = USART3（PC11，CubeMX 默认 DBUS 口，最可能）
 *   索引 1 = USART1（PB7）
 *   索引 2 = USART6（PG9）
 *
 * 统计含义：
 *   g_dbus_ev[i]  该口收到 18 字节的次数
 *   g_dbus_ok[i]  其中内容通过 DBUS 校验的次数
 *   g_dbus_err[i] 该口出现 UART 错误（帧错误/噪声/溢出）的次数
 *
 * 判读：
 *   ev>0 且 ok>0        → 接收机在这个口，正常
 *   ev==0 且 err>0      → 该口有信号但帧不合法（极性反了 / 波特率不对）
 *   三者全为 0          → 该口根本没有信号线
 */
constexpr uint8_t kDbusPorts = 3u;
uint8_t g_dbus_buf[kDbusPorts][kDbusLen] = {};
volatile uint32_t g_dbus_ev[kDbusPorts] = {0u, 0u, 0u};
volatile uint32_t g_dbus_ok[kDbusPorts] = {0u, 0u, 0u};
volatile uint32_t g_dbus_err[kDbusPorts] = {0u, 0u, 0u};
UART_HandleTypeDef* const kDbusUart[kDbusPorts] = {&huart3, &huart1, &huart6};

/** 由 UART 句柄反查端口索引（非法返回 kDbusPorts） */
uint8_t DbusPortIndex(UART_HandleTypeDef* huart)
{
  for (uint8_t i = 0; i < kDbusPorts; ++i) {
    if (huart == kDbusUart[i]) {
      return i;
    }
  }
  return kDbusPorts;
}

/** 把某一路 UART 配成 DBUS 电气参数并挂上 18 字节接收 */
void DbusArm(uint8_t idx)
{
  if (idx >= kDbusPorts) {
    return;
  }
  (void)HAL_UART_Receive_IT(kDbusUart[idx], g_dbus_buf[idx], kDbusLen);
}

/** 重新配置并挂接收（初始化时用） */
void DbusConfigurePort(uint8_t idx)
{
  if (idx >= kDbusPorts) {
    return;
  }
  UART_HandleTypeDef* const h = kDbusUart[idx];
  (void)HAL_UART_AbortReceive(h);
  h->Init.BaudRate = 100000u;
  h->Init.WordLength = UART_WORDLENGTH_9B;
  h->Init.Parity = UART_PARITY_EVEN;
  h->Init.StopBits = UART_STOPBITS_1;
  h->Init.Mode = UART_MODE_RX;
  h->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  h->Init.OverSampling = UART_OVERSAMPLING_16;
  (void)HAL_UART_Init(h);
  DbusArm(idx);
}

/** 处理一整帧：只有内容合法才刷新「遥控在线」，否则让它超时掉线 */
void DbusHandleFrame(uint8_t idx, uint16_t size)
{
  g_dbus_ev[idx] = g_dbus_ev[idx] + 1u;
  if (!robot::IsDbusFrameValid(g_dbus_buf[idx], size)) {
    return;
  }
  g_dbus_ok[idx] = g_dbus_ok[idx] + 1u;
  hello_world::remote_control::DT7* rc = GetRemoteControl();
  if (rc != nullptr) {
    rc->decode(g_dbus_buf[idx]);
  }
  robot::Robot* rb = GetRobot();
  if (rb != nullptr) {
    rb->NotifyRemoteFrame();
  }
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/

void CommTaskInit(void)
{
  /* ---------- CAN：两路都必须「滤波器 + Start + 接收中断」 ----------
   * ⚠️ 曾经的 bug：这里只对 kCanMotor / kCanComm 做初始化，而两者都指向 &hcan1，
   *    于是 hcan2 从未 Start —— 它的接收永远 0 帧，发送也是静默失败。
   *    症状就是「双路广播其实只发出去一路」，电调一旦挂在 CAN2 上就完全没反应。
   *    CubeMX 已经 MX_CAN1_Init / MX_CAN2_Init 过，硬件没问题，漏的是这一步。 */
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

  /* ---------- DBUS：三路都按 DBUS 参数接收（见上面的探测说明） ----------
   * 用中断接收而不是 DMA：USART1 没有配 RX DMA，三路统一走 IT 更简单，
   * 且 100 kbaud 下每字节一次中断（约 10 kHz）对 168 MHz 的负载可以接受。 */
  for (uint8_t i = 0; i < kDbusPorts; ++i) {
    DbusConfigurePort(i);
  }

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
     *   2) 电调 ID 是 1~4 还是 5~8 —— 0x200 和 0x1FF 都发
     * 确认接线与 ID 之后，把不需要的组合删掉即可。
     * （实测：本车 4 个电调在 CAN2，ID 1~4，即 0x201~0x204） */
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

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
  const uint8_t idx = DbusPortIndex(huart);
  if (idx >= kDbusPorts) {
    return;
  }
  DbusHandleFrame(idx, kDbusLen);
  /* 必须重新挂上，否则只能收到一帧 */
  DbusArm(idx);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
  const uint8_t idx = DbusPortIndex(huart);
  if (idx >= kDbusPorts) {
    return;
  }
  g_dbus_err[idx] = g_dbus_err[idx] + 1u;
  /* ⚠️ 关键：HAL 在出错时会中止接收，不重新挂就会永久收不到东西。
   * 现场表现是「插拔接收机之后就再也没数据了」，只能重启板子。 */
  (void)HAL_UART_AbortReceive(huart);
  DbusArm(idx);
}
