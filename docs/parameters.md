# 待实测参数清单

> 这份表是「代码能跑」与「实车正确」之间的差距。
> **A 类不测就是错的，B 类不测只是手感差，C 类是确认后必须收回的调试件。**

---

## A 类：安全相关，必须先测

| # | 参数 | 当前值 | 位置 | 为什么要测 | 怎么测 | 状态 |
|---|---|---|---|---|---|---|
| A1 | 云台 pitch 机械限位 | `-0.35 / +0.60` rad | `Gimbal/Task/component/main_task.cpp` | 估值。超限会顶结构、烧电机 | 手推到上下极限，读 `angle()`，各留 5° 余量 | ☐ |
| A2 | 四个轮子方向 `kWheelDir` | 全 `kDirRev` | `Chassis/Instance/component/ins_motor.cpp` | 反了车会原地打转 | 手转轮子看 `vel()` 符号；或低速试跑 | ☐ |
| A3 | δ 符号 `kFrameDeltaSign` | `1.0` | `RobotComponents/component/module_state.hpp` | 反了「前推」变「后退」 | 转云台 90° 后前推，看车往哪走 | ☐ |
| A4 | 自旋符号 `kSpinSign` | `1.0` | `Chassis/RobotModules/component/chassis_ik_solver.hpp` | 拨轮正转可能车反转 | 小陀螺模式拨轮试 | ☐ |
| A5 | 云台 yaw 方向 / 零位 | `kDirFwd / -1.36920` | `Gimbal/Instance/component/ins_motor.cpp` | 拆装后零位会变 | 上电读 `angle()`，对齐机械零位重算 | ☐ |
| A6 | 云台 pitch 方向 / 零位 | `kDirRev / 0.02480` | 同上 | 同上（且 pitch 目前无反馈） | 同上 | ☐ |
| A7 | M3508 减速比 | 代码注释标「待确认」 | `Chassis/Instance/component/ins_motor.cpp` | 拓扑图写 **14**，出厂 **19.2**，差 37% | 手转轮子，对比 `angle()` 与轮子实际转角 | ☐ |
| A8 | 电调 ID 与总线归属 | 0x200+0x1FF、CAN1+CAN2 全打 | `Chassis/Task/component/comm_task.cpp` | 不确定才全打，浪费带宽 | 看嗅探表出现 `0x201~0x204` 还是 `0x205~0x208` | ☐ |
| A9 | 遥控器档位映射 | 假设 1/3/2 = 上/中/下 | `protocol.md` 第五节 | Phantom 遥控器标注完全不同 | 拨到每档读 `rc_l_switch_ / rc_r_switch_` | ☐ |

---

## B 类：性能与手感，需要整定

| # | 参数 | 当前值 | 位置 | 备注 | 状态 |
|---|---|---|---|---|---|
| B1 | 云台 yaw 环 `kp/ki/kd` | **2.0 / 0 / 0.35** | `pid_tune.hpp`（可在线改） | 只验过"能动"，未整定 | ☐ |
| B2 | 云台 pitch 环 `kp/kd` | **5.0 / 0.12** | `main_task.cpp` | **完全没验证过** | ☐ |
| B3 | 轮速环 `kp/ki` | **200 / 20** | `ins_pid.cpp` | 教科书起始值，从未实车跑过 | ☐ |
| B4 | 跟随环 `kp/kd` | **4.0 / 0.05**，输出 ±6 | `main_task.cpp` | 太大会绕着云台转圈 | ☐ |
| B5 | 摇杆死区 | **0.05** | `module_state.hpp` | 实测遥控器 `rh` 有 −0.055 偏置，可能漏过死区 → 需中位标定 | ☐ |
| B6 | 云台 `max_torque` | **1.0** N·m（调试值） | `main_task.cpp` | 目标 3.0；多大不会甩飞结构要试 | ☐ |
| B7 | 底盘限幅 `max_vx/vy/w/spin_w` | **0.6/0.6/2.0/2.0**（调试值） | `chassis.hpp` | 目标 3.0/3.0/6.0/8.0；8 rad/s 自旋会不会翻车要试 | ☐ |
| B8 | 单轮速度上限 `kMaxWheelVelMps` | **1.0**（调试值） | `main_task.cpp` | 目标 4.5 | ☐ |
| B9 | `max_yaw_rate / max_pitch_rate` | **6.0 / 3.0** rad/s | `main_task.cpp` | 纯手感 | ☐ |

### B 类整定顺序（不能反）

```
轮速环 → 云台 yaw → 云台 pitch → 跟随环
```

反了的话，震荡时你分不清是哪个环的问题。

---

## C 类：验证过的假设，需要复核

| # | 参数 | 当前值 | 什么时候需要改 |
|---|---|---|---|
| C1 | 电机反馈超时 | 50 ms | 按实际帧率确认（C620 是 1 kHz，50 ms 很宽松） |
| C2 | 遥控 / 双板离线 | 100 ms | 实测丢帧率 |
| C3 | IWDG 超时 | 300 / 600 ms | 实测最坏控制周期后确认余量 |
| C4 | 电机就绪超时 | 2000 ms | 达妙使能实测耗时 |
| C5 | IMU 零漂 | 窗口 1.5 s、容差 0.05 rad/s | 实测静止噪声；太严会一直不收敛 |
| C6 | Mahony 增益 | kp=0.5 / ki=0 | 收敛慢会晃，快会跟着振动抖 |
| C7 | 云台绝对角漂移速率 | 未测 | 决定要不要加磁力计或角速度前馈（加分项） |

---

## D 类：确认后必须收回的调试件

| 位置 | 现在是 | 确认后应改为 |
|---|---|---|
| `Chassis/Task/component/comm_task.cpp` | 0x200+0x1FF × CAN1+CAN2（4 帧/ms） | 只留确定的那一组 |
| `Chassis/Task/component/comm_task.cpp` | `kDbusCapture = true`（DBUS 原始流抓取） | 抓完改回 false，否则 `g_dbus_ev/g_dbus_ok` 恒 0 |
| `Chassis/RobotModules/component/robot.cpp` | `g_bench_enable` 运行期开关（写魔数 0x5A5A5A5A 才生效，上电默认关） | 保持"上电默认关"即可，不要改成编译期 true |
| `Gimbal/RobotModules/component/robot.cpp` | `kBenchTest = false` | 保持 false（自测开关） |
| `Gimbal/inc/pid_tune.hpp` | 三个 `volatile float` 在线调参 | 定稿后写回 `MakeYawConfig()` 并注明 |
| 三处调试限幅 | 0.6 / 1.0 / 1.0 | 恢复正式值（B6~B8） |

---

## 记录表（填完再上电）

```
【几何】R = ______ m   半轴距 = ______ m   半轮距 = ______ m
【云台限位】pitch ∈ [______, ______] rad    余量 = ______°
【电机 ID】LF=__ LB=__ RB=__ RF=__   yaw=__  pitch=__
【减速比】M3508 = ______（14 / 19.2 / 实测）
【CAN 拓扑】底盘板电调=__路  双板=__路   云台板电机=__路
【嗅探结果】CAN1 出现: ______________________
            CAN2 出现: ______________________
【遥控档位】左: 上=__ 中=__ 下=__    右: 上=__ 中=__ 下=__
【整定结果】轮速 kp=__ ki=__    yaw kp=__ kd=__    pitch kp=__ kd=__
            跟随 kp=__ kd=__
```

---

## 现在就能做的（不需要电调通电）

| 项 | 需要什么 |
|---|---|
| A1 云台 pitch 限位 | 角度尺 + 手推 |
| 几何三个尺寸 | 尺子 |
| 嗅探表 | 板子上电即可（不需要电机转） |
| A9 遥控档位 | 接收机有数据时 |

其余都要等**电调通电**或**接收机修好**。
