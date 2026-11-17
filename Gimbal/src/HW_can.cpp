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
 *  云台板上分发两类帧：
 *    - 达妙反馈：ID = 电机 ID + 0x10（yaw / pitch 各一个）
 *    - 双板帧：C2G = 0x210
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"

#include "can.h"
#include "gimbal_chassis_comm.hpp"
#include "ins_all.hpp"
#include "tick.hpp"
#include "motor.hpp"
#include "motor_DaMiao.hpp"

/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];
static uint32_t p_tx_mailbox;

/* ===== 调试用：接收帧 ID 嗅探表 =====
 * 目的：确认「电机到底有没有在总线上说话」以及「它用的是哪个 ID」。
 * 不参与控制，只统计；烧录后跑几秒，用 Ozone / J-Link 读这几个变量即可。
 *   g_rx_total[0] = CAN1 收到的总帧数，[1] = CAN2
 *   g_rx_uniq[bus] = 已记录到的不同 ID 个数（最多 8 个）
 *   g_rx_ids[bus][i] / g_rx_cnt[bus][i] = 第 i 个 ID 及其帧数
 */
volatile uint32_t g_rx_total[2] = {0u, 0u};
volatile uint32_t g_rx_ids[2][8] = {{0u}, {0u}};
volatile uint32_t g_rx_cnt[2][8] = {{0u}, {0u}};
volatile uint32_t g_rx_uniq[2] = {0u, 0u};

/**
 * @brief       记录一帧的 ID（中断上下文，只做计数）
 */
static void RecordRxId(uint8_t bus, uint32_t std_id)
{
  g_rx_total[bus] = g_rx_total[bus] + 1u;  /* C++20 起对 volatile 用 ++ 已弃用 */
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

/**
 * @brief       按标准帧 ID 分发一帧 8 字节数据
 * @param       bus:    0 = CAN1(FIFO0)，1 = CAN2(FIFO1)
 * @param       std_id: 标准帧 ID
 * @param       data:   收到的 8 字节
 * @retval      None
 * @note        中断上下文：不阻塞、不打日志
 */
static void RouteRxFrame(uint8_t bus, uint32_t std_id, const uint8_t* data)
{
  RecordRxId(bus, std_id);

  /* 1. 达妙关节反馈：ID = id + 0x10 */
  hello_world::motor::Motor* joints[2] = {GetMotorYaw(), GetMotorPitch()};
  for (int i = 0; i < 2; ++i) {
    if (joints[i] == nullptr) {
      continue;
    }
    if (hello_world::motor::DaMiao::GetRxId(joints[i]->id()) == std_id) {
      if (joints[i]->decode(data)) {
        /* 反馈看门狗：闭环前用 IsFeedbackFresh() 判断，断了就不许积分 */
        joints[i]->NotifyFeedback(hello_world::tick::GetTickMs());
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
  /* ⚠️ 必须零初始化：曾经写成 `CAN_FilterTypeDef can_filter;`（未初始化），
   * 而下面只填了 7 个字段，`SlaveStartFilterBank` 是**栈上的垃圾值**。
   * 这个字段 = CAN2 的起始滤波器 bank（HAL 会写进 FMR.CAN2SB）；
   * 垃圾值一旦是 0，CAN2 就会去用 **bank 0** —— 那是给 CAN1 配的、指向 FIFO0 的
   * "全收"滤波器。结果：CAN2 的帧全部进 FIFO0，而云台只开了 CAN2_RX1 中断
   * → 中断永不触发 → FIFO 满/溢出（2026-11-17 实测 RF0R=0x1B: FMP=3 + FULL + OVR）
   * → pitch 的 0x011 永远读不到，之前被误判成"CAN2 物理静默、要查线束"。 */
  CAN_FilterTypeDef can_filter = {0};

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
  /* CAN2 从 bank 14 开始（bank 0~13 归 CAN1）。两路都写成 14 也没问题：
   * 它写的是同一个 FMR.CAN2SB，与"这次配的是哪一路"无关。 */
  can_filter.SlaveStartFilterBank = 14;

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
  (void)HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &p_tx_mailbox);
}

/**
 * @brief   CAN1 FIFO0 接收中断回调
 */
/**
 * @brief   接收回调：两路 CAN × 两个 FIFO 都要处理
 * @note    ⚠️ 不要按「CAN1 只用 FIFO0、CAN2 只用 FIFO1」来写死。
 *          滤波器寄存器属于 CAN1 的寄存器区，只有在 CAN1 处于初始化模式时才写得进；
 *          初始化顺序一旦不对，某些 bank 的 FIFO 分配会退回默认值（FIFO0），
 *          帧就会进到我们没处理的 FIFO 里，现象是「总线有帧、我们一帧都收不到」。
 *          这里把四种组合全部兜住，从哪个 FIFO 出来都能正确分发。
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
 * @brief   FIFO1 接收回调（同样两路都处理）
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
