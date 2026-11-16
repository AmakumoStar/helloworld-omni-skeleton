/**
 *******************************************************************************
 * @file      : HW_can.cpp
 * @brief     : CAN 收发基础封装实现（接收回调按 StdId 分发）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：CanFilter_Init + CAN_Send_Msg
 *  V1.1.0      2026-11-10      Hello World     2. 补全两个 RxFifo 回调的 StdId 分发
 *******************************************************************************
 * @attention :
 *  分发顺序：先按电机反馈 ID 匹配，再按双板帧 ID 匹配，都不命中就丢弃。
 *  M3508 反馈 ID = 0x200 + id；双板帧 C2G/G2C = 0x210/0x211，互不重叠。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"

#include "can.h"
#include "chassis_ik_solver.hpp"
#include "gimbal_chassis_comm.hpp"
#include "ins_all.hpp"
#include "tick.hpp"
#include "motor.hpp"

/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];
static uint32_t p_tx_mailbox;

/* ===== 调试用：接收帧 ID 嗅探表 =====
 * 目的：确认「电调到底有没有在总线上说话」以及「它用的是哪个 ID」。
 * 不参与控制，只统计；烧录后跑几秒，用 J-Link / Ozone 读这几个变量。
 *   g_rx_total[0]=CAN1 总帧数，[1]=CAN2
 *   g_rx_uniq[bus] = 已记录的不同 ID 个数（最多 8 个）
 *   g_rx_ids[bus][i] / g_rx_cnt[bus][i] = 第 i 个 ID 及其帧数
 */
volatile uint32_t g_rx_total[2] = {0u, 0u};
volatile uint32_t g_rx_ids[2][8] = {{0u}, {0u}};
volatile uint32_t g_rx_cnt[2][8] = {{0u}, {0u}};
volatile uint32_t g_rx_uniq[2] = {0u, 0u};

/* ===== 调试用：发送计数 =====
 * 为什么必须加：CAN_Send_Msg 原来把 HAL_CAN_AddTxMessage 的返回值丢掉了。
 * 三个发送邮箱都满（或总线错误被动）时，发送会静默失败 —— 现象和「电调没收到指令」
 * 一模一样，而且从任何别的变量上都看不出来。
 *   g_can_tx_ok[0/1]   CAN1/CAN2 成功进入邮箱的帧数
 *   g_can_tx_fail[0/1] 失败帧数（正常应恒为 0；非 0 说明发送链路有问题） */
volatile uint32_t g_can_tx_ok[2] = {0u, 0u};
volatile uint32_t g_can_tx_fail[2] = {0u, 0u};

/** 记录一帧的 ID（中断上下文，只做计数；C++20 起 volatile 不应用 ++） */
static void RecordRxId(uint8_t bus, uint32_t std_id)
{
  g_rx_total[bus] = g_rx_total[bus] + 1u;
  const uint32_t n = g_rx_uniq[bus];
  for (uint32_t i = 0; i < n; ++i) {
    if (g_rx_ids[bus][i] == std_id) {
      g_rx_cnt[bus][i] = g_rx_cnt[bus][i] + 1u;
      return;
    }
  }
  if (n < 8u) {
    g_rx_ids[bus][n] = std_id;
    g_rx_cnt[bus][n] = 1u;
    g_rx_uniq[bus] = n + 1u;
  }
}

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief       按标准帧 ID 分发一帧 8 字节数据
 * @param       bus:    0 = CAN1(FIFO0)，1 = CAN2(FIFO1)
 * @param       std_id: 标准帧 ID
 * @param       data:   收到的 8 字节
 * @retval      None
 * @note        在中断上下文调用，不得阻塞、不得打日志
 */
static void RouteRxFrame(uint8_t bus, uint32_t std_id, const uint8_t* data)
{
  RecordRxId(bus, std_id);

  /* 1. 电机反馈：M3508 的反馈 ID = 0x200 + 电机 ID */
  for (uint8_t i = 0; i < hello_world::chassis_ik_solver::kWheelCount; ++i) {
    hello_world::motor::Motor* motor = GetMotorWheel(static_cast<int>(i));
    if (motor == nullptr) {
      continue;
    }
    if (hello_world::motor::M3508::GetRxId(motor->id()) == std_id) {
      if (motor->decode(data)) {
        /* 反馈看门狗：轮速环闭环前用 IsFeedbackFresh() 判断，断了就不许积分 */
        motor->NotifyFeedback(hello_world::tick::GetTickMs());
      }
      return;
    }
  }

  /* 2. 双板通信帧 */
  robot::GimbalChassisComm* comm = GetGimbalChassisComm();
  if ((comm != nullptr) && (comm->rx_id() == std_id)) {
    comm->decode(data);
    return;
  }

  /* 3. 未命中：丢弃 */
}

/* Exported function definitions ---------------------------------------------*/

void CanFilter_Init(CAN_HandleTypeDef* hcan)
{
  CAN_FilterTypeDef can_filter;

  /* 全收：掩码全 0，任何 ID 均通过 */
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterActivation = ENABLE;

  if (hcan == &hcan1) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterBank = 0;
  } else if (hcan == &hcan2) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    can_filter.FilterBank = 14;
  } else {
    return;
  }

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }
}

void CAN_Send_Msg(CAN_HandleTypeDef* hcan, uint8_t* msg, uint32_t id, uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  tx_header.StdId = id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = len;
  const uint8_t bus = (hcan == &hcan2) ? 1u : 0u;
  if (HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &p_tx_mailbox) == HAL_OK) {
    g_can_tx_ok[bus] = g_can_tx_ok[bus] + 1u;
  } else {
    /* 邮箱全满 / 总线错误：以前这里什么都不记，故障完全隐形 */
    g_can_tx_fail[bus] = g_can_tx_fail[bus] + 1u;
  }
}

/**
 * @brief   FIFO0 接收中断回调：两路 CAN 都要处理
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    ⚠️ 不要按「CAN1 只用 FIFO0、CAN2 只用 FIFO1」写死：
 *          滤波器寄存器属于 CAN1 的寄存器区，只有 CAN1 在初始化模式时才写得进；
 *          初始化顺序一旦不对，某些 bank 的 FIFO 分配会退回默认值（FIFO0），
 *          帧就进了我们没处理的 FIFO，现象是「总线有帧、我们一帧收不到」。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1, can1_rx_data) == HAL_OK) {
      RouteRxFrame(0u, rx_header1.StdId, can1_rx_data);
    }
  } else if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header2, can2_rx_data) == HAL_OK) {
      RouteRxFrame(1u, rx_header2.StdId, can2_rx_data);
    }
  } else {
    return;
  }
  (void)HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   FIFO1 接收中断回调：两路 CAN 都要处理
 * @param   hcan: CAN 句柄
 * @retval  None
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header1, can1_rx_data) == HAL_OK) {
      RouteRxFrame(0u, rx_header1.StdId, can1_rx_data);
    }
  } else if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2, can2_rx_data) == HAL_OK) {
      RouteRxFrame(1u, rx_header2.StdId, can2_rx_data);
    }
  } else {
    return;
  }
  (void)HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}
