/**
 *******************************************************************************
 * @file      : gimbal_chassis_comm.hpp
 * @brief     : 底盘板 ↔ 云台板 双板 CAN 通信协议（8 字节定长）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 补全 ChassisData / GimbalData 字段
 *                                                2. 实现对称的 encode / decode / isOffline
 *******************************************************************************
 * @attention :
 *  1. 两块板各实例化一个对象，用 CodePart 决定编解码方向：
 *       CodePart::Chassis → encode 打包 ChassisData（C2G），decode 解 GimbalData（G2C）
 *       CodePart::Gimbal  → encode 打包 GimbalData（G2C），decode 解 ChassisData（C2G）
 *  2. 报文字段与缩放比例见 docs/units_and_protocol.md；两端必须严格一致。
 *  3. 浮点量一律按「先乘比例、再四舍五入、再饱和到 int16」处理，收发对称。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_CHASSIS_COMM_HPP_
#define GIMBAL_CHASSIS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "module_state.hpp"
#include "tick.hpp"

/* Exported macro ------------------------------------------------------------*/
namespace robot
{
/* Exported constants --------------------------------------------------------*/

/** 帧长（固定 8 字节） */
inline constexpr uint8_t kCommFrameLen = 8u;
/** 角度缩放：rad → int16（0.001 rad ≈ 0.057°，量程 ±32.7 rad ≈ ±5.2 圈） */
inline constexpr float kCommAngleScale = 1000.0f;
/** 辅助量缩放：rad/s → int8（量程 ±6.35 rad/s） */
inline constexpr float kCommAuxScale = 20.0f;
/** 在线判断阈值，单位 ms */
inline constexpr uint32_t kCommOfflineThresMs = 100u;

/* Exported types ------------------------------------------------------------*/

/**
 * 底盘 → 云台（C2G）帧数据
 * 至少覆盖：底盘在线 / 就绪、底盘工作模式、期望云台 yaw / pitch
 */
struct ChassisData {
  bool online = false;        ///< 底盘板是否在线（本板视角恒为 true）
  bool ready = false;         ///< 底盘是否已完成自检、可接受指令
  bool estop = false;         ///< 底盘侧急停（遥控器急停位 / 遥控离线）
  ChassisMode mode = ChassisMode::kSeparate;          ///< 底盘工作模式
  GimbalMode gimbal_mode = GimbalMode::kJointSpace;   ///< 期望云台模式
  float yaw_des = 0.0f;       ///< 期望云台 yaw，rad（连续角度，允许超 ±π）
  float pitch_des = 0.0f;     ///< 期望云台 pitch，rad
  float w_des = 0.0f;         ///< 小陀螺期望角速度，rad/s（也用作跟随前馈）
  uint8_t seq = 0;            ///< 帧序号，用于判断丢帧

  void Reset(void)
  {
    online = false;
    ready = false;
    estop = false;
    mode = ChassisMode::kSeparate;
    gimbal_mode = GimbalMode::kJointSpace;
    yaw_des = 0.0f;
    pitch_des = 0.0f;
    w_des = 0.0f;
    seq = 0u;
  }
};

/**
 * 云台 → 底盘（G2C）帧数据
 * 至少覆盖：云台在线 / 就绪、当前模式、实际 yaw / pitch 角度反馈
 */
struct GimbalData {
  bool online = false;  ///< 云台板是否在线
  bool ready = false;   ///< 云台是否已完成自检
  bool estop = false;   ///< 云台侧急停
  GimbalMode mode = GimbalMode::kJointSpace;  ///< 云台当前工作模式
  float yaw = 0.0f;     ///< 云台实际 yaw 关节角，rad（连续角度）
  float pitch = 0.0f;   ///< 云台实际 pitch 关节角，rad
  float yaw_vel = 0.0f; ///< 云台 yaw 角速度，rad/s（可用于底盘前馈）
  uint8_t seq = 0;      ///< 帧序号

  void Reset(void)
  {
    online = false;
    ready = false;
    estop = false;
    mode = GimbalMode::kJointSpace;
    yaw = 0.0f;
    pitch = 0.0f;
    yaw_vel = 0.0f;
    seq = 0u;
  }
};

/* 底盘↔云台双板 CAN 通讯协议类。底盘板与云台板各实例化一个，用 code_part 区分方向。 */
class GimbalChassisComm
{
 public:
  /* 标记本实例运行在哪块板，决定 encode / decode 的方向。 */
  enum class CodePart : uint8_t { Chassis = 0, Gimbal = 1 };

  GimbalChassisComm(CodePart code_part, uint32_t tx_id, uint32_t rx_id);

  /* 编码己方帧：把本地 ChassisData / GimbalData 打包进 8 字节。 */
  void encode(uint8_t tx_data[kCommFrameLen]);
  /* 解码对方帧：把 8 字节还原成对方语义并更新本地缓存。 */
  void decode(const uint8_t rx_data[kCommFrameLen]);

  /* 判断对方是否离线（超过阈值时长未收到新帧）。 */
  bool isOffline() const;

  /* 数据访问（供 RobotModules / Task 读写）。 */
  ChassisData& chassis_data() { return chassis_data_; }
  GimbalData& gimbal_data() { return gimbal_data_; }
  const ChassisData& chassis_data() const { return chassis_data_; }
  const GimbalData& gimbal_data() const { return gimbal_data_; }

  /* 调试观测 */
  uint32_t tx_id() const { return tx_id_; }
  uint32_t rx_id() const { return rx_id_; }
  /** 累计收到的有效帧数（上板交接前用它确认链路是否通） */
  uint32_t rx_count() const { return rx_count_; }
  /** 上次收到帧距现在的时长，单位 ms */
  uint32_t offlineTicks() const;

 private:
  CodePart code_part_;
  uint32_t tx_id_;
  uint32_t rx_id_;
  uint32_t last_rx_tick_;  ///< 上次收到对方帧的 GetTickMs() 时间戳
  uint32_t rx_count_ = 0u; ///< 累计收到帧数
  ChassisData chassis_data_;
  GimbalData gimbal_data_;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot

#endif /* GIMBAL_CHASSIS_COMM_HPP_ */
