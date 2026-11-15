/**
 *******************************************************************************
 * @file      : gimbal_chassis_comm.cpp
 * @brief     : 双板通信协议实现（8 字节定长，收发严格对称）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 实现 encode / decode / isOffline
 *******************************************************************************
 * @attention :
 *  字节布局（C2G 与 G2C 共用同一结构，只是语义字段不同）：
 *    byte0      flags   : bit0 在线, bit1 就绪, bit2 急停
 *    byte1      mode    : bit0~1 底盘模式, bit2~3 云台模式
 *    byte2~3    yaw     : int16 小端，×1000（rad）
 *    byte4~5    pitch   : int16 小端，×1000（rad）
 *    byte6      aux     : int8（C2G: w_des ×20；G2C: yaw_vel ×20），单位 rad/s
 *    byte7      seq     : uint8 帧序号
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "gimbal_chassis_comm.hpp"

#include <cmath>

#include "tick.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
namespace
{
/* 本地限幅，避免依赖 robot::Clamp 的命名 */
inline float ClampLocal(float value, float lower, float upper)
{
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

/** 浮点 → int16（按比例缩放、四舍五入、饱和） */
inline int16_t ToInt16(float value, float scale)
{
  const float scaled = ClampLocal(value * scale, -32768.0f, 32767.0f);
  return static_cast<int16_t>(scaled >= 0.0f ? (scaled + 0.5f) : (scaled - 0.5f));
}

/** int16 → 浮点 */
inline float FromInt16(int16_t value, float scale)
{
  return static_cast<float>(value) / scale;
}

/** 浮点 → int8（辅助量） */
inline int8_t ToInt8(float value, float scale)
{
  const float scaled = ClampLocal(value * scale, -128.0f, 127.0f);
  return static_cast<int8_t>(scaled >= 0.0f ? (scaled + 0.5f) : (scaled - 0.5f));
}

/** int8 → 浮点 */
inline float FromInt8(int8_t value, float scale)
{
  return static_cast<float>(value) / scale;
}

/** 小端写 16 位 */
inline void PutI16(uint8_t* dst, int16_t value)
{
  const uint16_t raw = static_cast<uint16_t>(value);
  dst[0] = static_cast<uint8_t>(raw & 0xFFu);
  dst[1] = static_cast<uint8_t>((raw >> 8) & 0xFFu);
}

/** 小端读 16 位 */
inline int16_t GetI16(const uint8_t* src)
{
  const uint16_t raw = static_cast<uint16_t>(src[0]) |
                       static_cast<uint16_t>(static_cast<uint16_t>(src[1]) << 8);
  return static_cast<int16_t>(raw);
}

/* 模式位打包：底盘模式占 bit0~1，云台模式占 bit2~3 */
inline uint8_t PackMode(robot::ChassisMode chassis_mode, robot::GimbalMode gimbal_mode)
{
  const uint8_t c = static_cast<uint8_t>(static_cast<uint8_t>(chassis_mode) & 0x03u);
  const uint8_t g = static_cast<uint8_t>((static_cast<uint8_t>(gimbal_mode) & 0x03u) << 2);
  return static_cast<uint8_t>(c | g);
}
inline robot::ChassisMode UnpackChassisMode(uint8_t raw)
{
  return static_cast<robot::ChassisMode>(raw & 0x03u);
}
inline robot::GimbalMode UnpackGimbalMode(uint8_t raw)
{
  return static_cast<robot::GimbalMode>((raw >> 2) & 0x03u);
}

/* 标志位 */
inline uint8_t PackFlags(bool online, bool ready, bool estop)
{
  uint8_t flags = 0u;
  if (online) {
    flags |= 0x01u;
  }
  if (ready) {
    flags |= 0x02u;
  }
  if (estop) {
    flags |= 0x04u;
  }
  return flags;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/

robot::GimbalChassisComm::GimbalChassisComm(
    robot::GimbalChassisComm::CodePart code_part, uint32_t tx_id, uint32_t rx_id)
    : code_part_(code_part), tx_id_(tx_id), rx_id_(rx_id),
      last_rx_tick_(hello_world::tick::GetTickMs())
{
}

void robot::GimbalChassisComm::encode(uint8_t tx_data[robot::kCommFrameLen])
{
  if (tx_data == nullptr) {
    return;
  }

  /* 按本板身份选择要打包的一组数据 */
  const bool is_chassis_side = (code_part_ == CodePart::Chassis);

  bool online = false;
  bool ready = false;
  bool estop = false;
  uint8_t mode_byte = 0u;
  float yaw = 0.0f;
  float pitch = 0.0f;
  float aux = 0.0f;
  uint8_t seq = 0u;

  if (is_chassis_side) {
    ChassisData& d = chassis_data_;
    online = d.online;
    ready = d.ready;
    estop = d.estop;
    mode_byte = PackMode(d.mode, d.gimbal_mode);
    yaw = d.yaw_des;
    pitch = d.pitch_des;
    aux = d.w_des;
    seq = d.seq;
  } else {
    GimbalData& d = gimbal_data_;
    online = d.online;
    ready = d.ready;
    estop = d.estop;
    mode_byte = PackMode(ChassisMode::kSeparate, d.mode);
    yaw = d.yaw;
    pitch = d.pitch;
    aux = d.yaw_vel;
    seq = d.seq;
  }

  tx_data[0] = PackFlags(online, ready, estop);
  tx_data[1] = mode_byte;
  PutI16(&tx_data[2], ToInt16(yaw, kCommAngleScale));
  PutI16(&tx_data[4], ToInt16(pitch, kCommAngleScale));
  tx_data[6] = static_cast<uint8_t>(ToInt8(aux, kCommAuxScale));
  tx_data[7] = seq;
}

void robot::GimbalChassisComm::decode(const uint8_t rx_data[robot::kCommFrameLen])
{
  if (rx_data == nullptr) {
    return;
  }

  const uint8_t flags = rx_data[0];
  const uint8_t mode_byte = rx_data[1];
  const float yaw = FromInt16(GetI16(&rx_data[2]), kCommAngleScale);
  const float pitch = FromInt16(GetI16(&rx_data[4]), kCommAngleScale);
  const float aux = FromInt8(static_cast<int8_t>(rx_data[6]), kCommAuxScale);
  const uint8_t seq = rx_data[7];

  const bool online = (flags & 0x01u) != 0u;
  const bool ready = (flags & 0x02u) != 0u;
  const bool estop = (flags & 0x04u) != 0u;

  if (code_part_ == CodePart::Chassis) {
    /* 底盘板收 G2C → 更新 gimbal_data_ */
    GimbalData& d = gimbal_data_;
    d.online = online;
    d.ready = ready;
    d.estop = estop;
    d.mode = UnpackGimbalMode(mode_byte);
    d.yaw = yaw;
    d.pitch = pitch;
    d.yaw_vel = aux;
    d.seq = seq;
  } else {
    /* 云台板收 C2G → 更新 chassis_data_ */
    ChassisData& d = chassis_data_;
    d.online = online;
    d.ready = ready;
    d.estop = estop;
    d.mode = UnpackChassisMode(mode_byte);
    d.gimbal_mode = UnpackGimbalMode(mode_byte);
    d.yaw_des = yaw;
    d.pitch_des = pitch;
    d.w_des = aux;
    d.seq = seq;
  }

  last_rx_tick_ = hello_world::tick::GetTickMs();
  ++rx_count_;
}

uint32_t robot::GimbalChassisComm::offlineTicks() const
{
  /* 处理 tick 回绕：GetTickMs() 是 uint32，减法本身即环形语义 */
  const uint32_t now = hello_world::tick::GetTickMs();
  return static_cast<uint32_t>(now - last_rx_tick_);
}

bool robot::GimbalChassisComm::isOffline() const
{
  return offlineTicks() > kCommOfflineThresMs;
}

/* Private function definitions ----------------------------------------------*/
