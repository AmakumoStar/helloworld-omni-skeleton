# 参数与待办

> 只列**当前生效值**和**还没定稿的项**。每个值的来历、踩过的坑见 [known_issues.md](known_issues.md)。

---

## 一、当前生效参数（代码默认值，2026-11-18，提交 47dce8f）

### 云台

| 项 | 值 | 位置 |
|---|---|---|
| yaw `kp / ki / kd` | **3.5 / 0.02 / 0.35** | `Gimbal/Task/component/main_task.cpp`（`g_yaw_*`，可在线改） |
| yaw D 项来源 | `Imu::yawRate()`（已取负，与 euler yaw 同号） | `Gimbal/inc/imu.hpp` |
| yaw 积分限幅 / 清积分 | **±0.15 N·m** / \|err\|>34° 清 | `main_task.cpp` / `gimbal.hpp` |
| yaw `max_torque` | **1.0 N·m** | `main_task.cpp`（红线） |
| pitch `kp / ki / kd` | **10.0 / 0 / 0.25** | `main_task.cpp` |
| pitch 重力前馈 | **0.60 N·m** | `main_task.cpp`（`g_pitch_grav_ff`） |
| pitch `max_torque_pitch` | **2.0 N·m** | `main_task.cpp`（操作手批准的放宽） |
| pitch 软件限位 | **−0.008 / +0.583 rad** | `main_task.cpp`（机械 −0.111/+0.687，两端余量 6°） |
| `max_yaw_rate / max_pitch_rate` | **2.0 / 1.0 rad/s** | `main_task.cpp` |

### 底盘

| 项 | 值 | 位置 |
|---|---|---|
| 轮速环 `kp / ki / ff` | **400 / 300 / 700**（ff 随速度衰减，`knee=1.0 rad/s`） | `Chassis/Instance/component/ins_pid.cpp` |
| 跟随环 `kp / kd` | **1.2 / 0.20** | `Chassis/RobotModules/component/robot.cpp` |
| 分离航向保持 `kp / kd` | **1.2 / 0.15** | 同上 |
| `max_vx / max_vy` | **0.20 / 0.20 m/s** | `chassis.hpp` |
| `max_w / max_spin_w` | **1.50 / 1.50 rad/s** | 同上 |
| 单轮闸门 `kMaxWheelVelMps` | **1.10 m/s**（`static_assert` 与上限耦合） | `Chassis/Task/component/main_task.cpp` |
| `g_frame_lead` | **0.19 s**（代码默认；在线试到 0.10 未定稿） | `robot.cpp` |
| `g_delta_offset / g_front_yaw` | **1.8900 / 0** | `robot.cpp` |
| `g_ignore_delta` | **0** | 正式值，必须 0 |

### 通信 / 状态机

| 项 | 值 |
|---|---|
| 电机反馈超时 | 50 ms |
| 遥控 / 双板离线 | 100 ms |
| IWDG | 底盘 300 ms / 云台 600 ms |
| 电机就绪超时 | 2000 ms |
| IMU 零漂窗口 / 容差 | 1.5 s / 0.05 rad/s |
| Mahony | kp=0.5 / ki=0 |

---

## 二、还没定稿的项（按优先级）

| # | 项 | 现状 | 下一步 |
|---|---|---|---|
| 1 | **1.71 倍比例** | 自旋/平移实际≈标称的 1.7 倍（轮速 3.18 时车实际 1.370 rad/s，按现有常数只能 0.807） | 卷尺量轮径/轴距/轮距 + 看减速箱铭牌（拓扑图 14 / 出厂 19.2）+ 直行 2 s 量距离 + 手转 5 圈读 `angle()`；**先量再改**，别直接改 R |
| 2 | `g_frame_lead` | 代码 0.19；实车试 0.10 更直，但未最终确认 | 定稿后改 `robot.cpp` + 重编烧录（底盘） |
| 3 | 速度阶梯 | 0.20 m/s / 1.50 rad/s / 闸门 1.10 | 必须先解决 1.71，再 0.25 → 0.6 → 3.0，每档架空验方向 |
| 4 | 轮速 ~9 Hz 纹波 | 峰峰值 1.1~1.5 rad/s（100 Hz 采样，可能混叠） | 1 kHz 录制定性；别急着给轮速环加 D |
| 5 | `0x201` 帧数是别人 2 倍 | 184,818 vs ~92,400 | 查 C620 拨码数量/接线（硬证据） |
| 6 | 看门狗 / 归零验收 | 未做 | `jlink.ps1 -Action wdg-demo / zero-check` |
| 7 | `max_pitch_rate=1.0` 手感 | 未确认 | 小幅度试，手不离急停 |
| 8 | 摇杆死区 0.05 / 中位偏置 | `rh` 实测有 −0.055 偏置 | 中位标定后再定死区 |

---

## 三、调试开关清单（正式使用前必须收回）

| 位置 | 项 | 应保持 |
|---|---|---|
| `Chassis/RobotModules/component/robot.cpp` | `g_ignore_delta` | **0** |
| 同上 | `g_force_mode / g_estop_force` | **0**（上电默认） |
| 同上 | `g_bench_enable` | 运行期魔数开关，上电默认关 |
| `Chassis/Task/component/comm_task.cpp` | `kDbusCapture` | **false** |
| `Gimbal/RobotModules/component/robot.cpp` | `kBenchTest` | **false** |

> 运行期用 J-Link 写的值**断电即丢**；定稿值必须写回代码默认值并重编烧录。

---

## 四、几何 / 标定记录（待填）

```
【几何】R = ______ m   半轴距 = ______ m   半轮距 = ______ m
【减速比】M3508 = ______（14 / 19.2 / 实测）
【云台限位】pitch ∈ [______, ______] rad
【δ 零点】g_delta_offset = ______
【整定结果】yaw kp=3.5 ki=0.02 kd=0.35    pitch kp=10 kd=0.25
           轮速 kp=400 ki=300 ff=700      跟随 kp=1.2 kd=0.20
```
