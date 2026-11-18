# 待实测参数清单

> 这份表是「代码能跑」与「实车正确」之间的差距。
> **A 类不测就是错的，B 类不测只是手感差，C 类是确认后必须收回的调试件。**

---

## A 类：安全相关，必须先测

| # | 参数 | 当前值 | 位置 | 为什么要测 | 怎么测 | 状态 |
|---|---|---|---|---|---|---|
| A1 | 云台 pitch 机械限位 | `-0.35 / +0.60` rad | `Gimbal/Task/component/main_task.cpp` | 估值。超限会顶结构、烧电机 | 手推到上下极限，读 `angle()`，各留 5° 余量 | ☐ |
| A2 | 四个轮子方向 `kWheelDir` | `LF/LB = kDirRev`，`RB/RF = kDirFwd` | `Chassis/Instance/component/ins_motor.cpp` | 反了车会原地打转 | 手转轮子看 `vel()` 符号；或低速试跑 | ✅ 2026-11-17 实车标定 |
| A3 | δ 符号 `kFrameDeltaSign` | `-1.0` | `RobotComponents/component/module_state.hpp` | 反了「前推」变「后退」 | 转云台 90° 后前推，看车往哪走 | ✅ 2026-11-17 实车标定 |
| A4 | 自旋符号 `kSpinSign` | `1.0` | `Chassis/RobotModules/component/chassis_ik_solver.hpp` | 拨轮正转可能车反转 | 小陀螺模式拨轮试 | ☐ |
| A5 | 云台 yaw 方向 / 零位 | `kDirFwd / -1.36920` | `Gimbal/Instance/component/ins_motor.cpp` | 拆装后零位会变 | 上电读 `angle()`，对齐机械零位重算 | ☐ |
| A6 | 云台 pitch 方向 / 零位 | `kDirRev / 0.02480` | 同上 | 同上（且 pitch 目前无反馈） | 同上 | ☐ |
| A7 | M3508 减速比 | 代码用 **19.2**（`3591/187`），注释标「待确认」 | `Chassis/Instance/component/ins_motor.cpp` | 拓扑图写 **14**，出厂 **19.2**，差 37%。**2026-11-17 实测：四轮 3.18 rad/s（目标 3.13）时车实际转 1.370 rad/s，而按现有常数只能转 0.807 → 整体差 1.70 倍**，这一项是最可疑的来源（1.37 倍） | ①卷尺量轮径/轴距/轮距 ②看减速箱铭牌 ③直行 2 s 量距离（标称 0.30 m）④手转轮子 5 圈读 `angle()`（一圈应 6.283 rad） | ☐ 未解决，见 `known_issues.md` 7.5 |
| A8 | 电调 ID 与总线归属 | 0x200+0x1FF、CAN1+CAN2 全打 | `Chassis/Task/component/comm_task.cpp` | 不确定才全打，浪费带宽 | 看嗅探表出现 `0x201~0x204` 还是 `0x205~0x208` | ☐ |
| A9 | 遥控器档位映射 | 假设 1/3/2 = 上/中/下 | `protocol.md` 第五节 | Phantom 遥控器标注完全不同 | 拨到每档读 `rc_l_switch_ / rc_r_switch_` | ☐ |

---

## B 类：性能与手感，需要整定

| # | 参数 | 当前值 | 位置 | 备注 | 状态 |
|---|---|---|---|---|---|
| B1 | 云台 yaw 环 `kp/ki/kd` | **1.5 / 0 / 0.10** | `Gimbal/Task/component/main_task.cpp`（可在线改 `g_yaw_*`） | ki 恒 0（红线）；kp≥6 会震荡（1 N·m 时 err 0.17 就饱和） | ☑ 可用 |
| B2 | 云台 pitch 环 `kp/kd` | **10.0 / 0.25** | 同上（可在线改 `g_pitch_*`） | 原 5.0/0.12"手拨不够硬"；ki 固定 0（重力静差靠 kp 压） | ☑ 可用 |
| B3 | 轮速环 `kp/ki/ff` | **400 / 300 / 700**（ff **随速度衰减**，`knee=1.0 rad/s`） | `ins_pid.cpp`（可在线改 `g_wheel_*`） | 2026-11-18 架空阶跃整定：常数 ff=700 是"一开始快然后变慢"的元凶（超调 106%）；改 400/300/衰减后超调 ~26%（含 ±12% 纹波）、稳态 −1~6%。**落地需复核破静摩擦的 ff**（架空无载不代表地面） | ☑ 架空整定完，待落地复核 |
| B4 | 跟随环 `kp/kd` | **1.2 / 0.20**，输出 ±6 | `main_task.cpp`（可在线改 `g_follow_*`） | kp=2.0 进极限环；提速要调 `max_w` 而不是 kp | ☑ 可用 |
| B5 | 摇杆死区 | **0.05** | `module_state.hpp` | 实测遥控器 `rh` 有 −0.055 偏置，可能漏过死区 → 需中位标定 | ☐ |
| B6 | 云台 `max_torque` | **1.0** N·m | `Gimbal/Task/component/main_task.cpp` | **红线**：11-16 飞转事故后定死；目标 3.0 需证明不会甩飞结构 | ☑ 红线，勿动 |
| B7 | 底盘限幅 `max_vx/vy/w/spin_w` | **0.20 / 0.20 / 1.50 / 1.50**，小陀螺档平移再 ×`kSpinTransBoostMax=1.5` | `chassis.hpp` | 阶梯：0.15 → **0.20（当前）** → 0.25 → 0.6 → 3.0；每次提速前先架空确认方向。spin×1.5 已用 A/B 读数验证（spin 0.30 / 分离 0.20） | ☑ 已落地验证 |
| B8 | 单轮速度上限 `kMaxWheelVelMps` | **1.10**（= 1.5×0.4 + 0.30676×1.5 = 1.060，留余量） | `main_task.cpp` | 目标 4.5。逆解后的最终闸门；`static_assert` 已把"平移倍率"算进去 | ☑ 已落地验证 |
| B9 | `max_yaw_rate / max_pitch_rate` | **2.0 / 1.0** rad/s | `Gimbal/Task/component/main_task.cpp` | 纯手感，pitch 1.0 未确认 | ☐ |

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
| `Chassis/Task/component/comm_task.cpp` | `kDbusCapture = false`（抓包已完成，保持关闭） | 需要重新抓包时临时改 true |
| `Chassis/RobotModules/component/robot.cpp` | `g_bench_enable` 运行期开关（写魔数 0x5A5A5A5A 才生效，上电默认关） | 保持"上电默认关"即可，不要改成编译期 true |
| `Chassis/RobotModules/component/robot.cpp` | `g_force_mode / g_estop_force`（调试用强制模式/急停） | 保持 0；正式使用前确认复位后为 0 |
| `Chassis/RobotModules/component/robot.cpp` | `g_ignore_delta = 1`（把 δ 当 0） | **必须为 0**，否则"平移随枪口"这个验收点失效 |
| `Chassis/Instance/component/ins_pid.cpp` | `g_wheel_kp/ki/ff` 在线整定 | 定稿后写回 `MakeWheelConfig()` 并同步本节 B3 |
| `Gimbal/RobotModules/component/robot.cpp` | `kBenchTest = false` | 保持 false（自测开关） |
| `Gimbal/inc/pid_tune.hpp` | 三个 `volatile float` 在线调参 | 定稿后写回 `MakeYawConfig()` 并注明 |
| 三处调试限幅 | 0.15 / 0.3 / 1.0 | 按 B7/B8 阶梯恢复到正式值（3.0 / 4.5 / 3.0） |

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
