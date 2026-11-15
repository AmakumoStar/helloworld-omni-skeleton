/**
 *******************************************************************************
 * @file      : pid.hpp
 * @brief     : 可复用 PID 控制器（位置式，带积分限幅、输出限幅、抗积分饱和）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      Hello World     1. 初版：位置式 + 微分先行 + 条件积分
 *******************************************************************************
 * @attention :
 *  1. 本文件为纯算法，不依赖 HAL / 电机 / 时间基准，可在 PC 上用 g++ 直接编译测试。
 *  2. 时间步长 dt 由调用方给定（单位 s），本类自己不取时间。
 *  3. Update() 的微分作用在「测量值」上（微分先行），目标阶跃时不会产生微分冲击；
 *     需要「误差的微分」时用 UpdateErr()。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_PID_PID_HPP_
#define HW_COMPONENTS_ALGORITHMS_PID_PID_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

namespace hello_world
{
namespace pid
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Pid
{
 public:
  /** PID 参数与限幅 */
  struct Config {
    float kp = 0.0f;  ///< 比例系数
    float ki = 0.0f;  ///< 积分系数（乘在误差上，单位：输出/误差/秒）
    float kd = 0.0f;  ///< 微分系数（单位：输出·秒/误差）
    float out_min = -1.0f;  ///< 输出下限
    float out_max = 1.0f;   ///< 输出上限
    float int_min = -1.0f;  ///< 积分项下限（输出单位）
    float int_max = 1.0f;   ///< 积分项上限（输出单位）
  };

  Pid(void) = default;
  explicit Pid(const Config& config) { Init(config); }
  ~Pid(void) = default;

  /**
   * @brief       初始化：写入参数并清空内部状态
   * @param       config: 参数与限幅
   * @retval      None
   */
  void Init(const Config& config);

  /** 清空积分与微分历史（状态切换 / 急停时调用，防止残留指令突变） */
  void Reset(void);

  /** 运行期改参数，不清状态（调参用） */
  void set_config(const Config& config) { config_ = config; }

  const Config& config(void) const { return config_; }

  /**
   * @brief       按「目标值 / 测量值」更新一次
   * @param       target: 目标值
   * @param       actual: 测量值
   * @param       dt:     本周期时长，单位 s（<= 0 时直接返回上次输出，不更新状态）
   * @retval      限幅后的输出
   * @note        微分为「微分先行」：d = −kd·(actual − prev_actual)/dt
   */
  float Update(float target, float actual, float dt);

  /**
   * @brief       按「误差」更新一次（没有独立测量值时使用，如底盘跟随环）
   * @param       error: 本周期误差
   * @param       dt:    本周期时长，单位 s
   * @retval      限幅后的输出
   * @note        微分为误差的微分：d = kd·(error − prev_error)/dt
   */
  float UpdateErr(float error, float dt);

  /* 数据获取 */
  float out(void) const { return out_; }
  float integral(void) const { return integral_; }
  /** 上周期输出是否被限幅（调试用） */
  bool saturated(void) const { return saturated_; }

 private:
  /**
   * @brief       公共的「比例 + 积分 + 微分」合成
   * @param       error: 误差
   * @param       d_term: 已经算好的微分项
   * @param       dt: 周期
   * @retval      限幅后的输出
   */
  float Step(float error, float d_term, float dt);

  Config config_{};
  float integral_ = 0.0f;   ///< 积分项（已是「输出」量纲）
  float prev_meas_ = 0.0f;  ///< 上次测量值（微分先行用）
  float prev_error_ = 0.0f; ///< 上次误差（误差微分用）
  float out_ = 0.0f;        ///< 上次输出
  bool first_ = true;       ///< 首次调用时不做微分
  bool saturated_ = false;  ///< 上次输出是否饱和
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace pid
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_PID_PID_HPP_ */
