# 单位约定、双板协议与关键参数表

> 本文件是「动手前先画一张字段对照表」的落地版本。
> 两块板、三个层次（RobotModules / RobotComponents / Instance）都以此为准，
> 改动任何一项都必须同步改另一块板。

---

## 一、单位约定（全工程唯一口径）

| 物理量 | 符号 | 单位 | 说明 |
|---|---|---|---|
| 线速度 | vx, vy | m/s | 平移，定义在**云台坐标系** |
| 角速度 | w | rad/s | 绕 z 轴，**逆时针为正**（z 向上，右手法则） |
| 关节角度 | yaw, pitch | rad | yaw 可连续旋转（超 ±π），pitch 受限位约束 |
| 角度差 | δ, error | rad | 一律先经 `robot::WrapToPi()` 归一化到 [-π, π) |
| 电机角度 | angle() | rad | 电机输出端（已过减速比） |
| 电机角速度 | vel() | rad/s | 电机输出端 |
| 轮子线速度 | — | m/s | 由轮半径换算成角速度后再进速度环 |
| 轮子角速度 | — | rad/s | 速度环 PID 的输入/目标 |
| PID 输出（轮速环） | — | 报文原始量 | ±16384 对应 ±20 A |
| PID 输出（云台环） | — | N·m | DM-J4310 上限 7.5 N·m |

**易错点**：文档里的「半轮距 L/2」指左右方向（y），「半轴距 W/2」指前后方向（x），
本工程分别命名为 `half_track` 与 `half_wheelbase`，二者相加就是自旋力臂 k。

---

## 二、坐标系约定（考核说明 3.2 的硬性要求）

- **云台系**：x = 云台朝向（枪管方向），y = 云台左方，z = 上
- **车体系**：x = 车头朝向，y = 车体左方，z = 上
- **约定**：所有平移指令 (vx, vy) 一律定义在云台系，`vx > 0` 永远是「操作手面对的方向」

变换（`robot::GimbalToChassis()`）：

```
δ  = 云台朝向 − 车头朝向        （取云台 yaw 关节角，经 G2C 帧送到底盘板）
vx_c = vx_g·cos δ + vy_g·sin δ
vy_c = −vx_g·sin δ + vy_g·cos δ
```

**符号是唯一需要实车确认的地方**：如果推杆发现平移方向反了，只改
`module_state.hpp` 里的 `kFrameDeltaSign`（取 −1），不要动调用点。

---

## 三、双板 CAN 帧（0x210 C2G / 0x211 G2C）

### 3.1 CAN ID 分配

| 用途 | ID | 方向 | 说明 |
|---|---|---|---|
| M3508 控制（ID 1~4） | `0x200` | 底盘板 → 电调 | 4 台电机一帧，每台 2 字节 |
| M3508 控制（ID 5~8） | `0x1FF` | 底盘板 → 电调 | 本车未使用 |
| M3508 反馈 | `0x201`~`0x208` | 电调 → 底盘板 | = 0x200 + id |
| DM-J4310 发送 | `0x01`~`0x0F` | 云台板 → 电机 | = 电机 ID |
| DM-J4310 反馈 | `0x11`~`0x1F` | 电机 → 云台板 | = 电机 ID + 0x10 |
| **C2G（底盘→云台）** | **`0x210`** | 跨板 | 本工程选定 |
| **G2C（云台→底盘）** | **`0x211`** | 跨板 | 本工程选定 |

### 3.2 字节布局（两个方向共用同一结构）

| 字节 | 字段 | 类型 | 缩放 | 说明 |
|---|---|---|---|---|
| 0 | flags | uint8 | — | bit0 在线 / bit1 就绪 / bit2 急停 |
| 1 | mode | uint8 | — | bit0~1 底盘模式，bit2~3 云台模式 |
| 2~3 | yaw | int16 小端 | ×1000 | C2G: 期望 yaw；G2C: 实际 yaw |
| 4~5 | pitch | int16 小端 | ×1000 | C2G: 期望 pitch；G2C: 实际 pitch |
| 6 | aux | int8 | ×20 | C2G: `w_des`；G2C: `yaw_vel` |
| 7 | seq | uint8 | — | 帧序号，判断丢帧 |

量程与精度：

- 角度：±32767 / 1000 = ±32.7 rad（≈ ±5.2 圈），分辨率 0.001 rad ≈ 0.057°
- 辅助量：±127 / 20 = ±6.35 rad/s，分辨率 0.05 rad/s

### 3.3 枚举取值（两板必须一致）

| 枚举 | 取值 |
|---|---|
| `LifeState` | 0=Dead, 1=Resurrection, 2=Working |
| `ChassisMode` | 0=分离, 1=跟随, 2=小陀螺 |
| `GimbalMode` | 0=关节空间 |
| `ControlSource` | 0=无, 1=遥控器 |

### 3.4 离线判断

阈值 `kCommOfflineThresMs = 100 ms`。
`isOffline()` = 距离 `last_rx_tick_` 的时间差 > 100 ms。
拔线后约 100 ms 置位，另一板进 Dead 并全零输出。

---

## 四、遥控器映射（验收按此演示）

| 输入 | 作用 |
|---|---|
| 左摇杆竖直 | 前后平移（云台系 vx） |
| 左摇杆水平 | 左右平移（云台系 vy，右推为 −y） |
| 右摇杆水平 | 分离模式原地转向；云台模式下控制 yaw 转速 |
| 右摇杆竖直 | 云台模式下控制 pitch 转速 |
| 拨轮 | 小陀螺模式的自旋角速度（必须限幅） |
| **左拨杆 Down** | **急停：所有电机立即归零** |
| 右拨杆 Up / Mid / Down | 分离 / 跟随 / 小陀螺 |

---

## 五、待填参数（必须实机测量，不要照抄代码里的占位值）

| 参数 | 代码位置 | 当前值 / 来源 |
|---|---|---|
| 轮半径 R | `Chassis/Task/component/main_task.cpp` | **0.07786 m（已补全版 6.2）** |
| 半轴距 W/2 | 同上 | **0.15338 m（已补全版 6.2）** |
| 半轮距 L/2 | 同上 | **0.15338 m（已补全版 6.2）** |
| 单轮线速度上限 | 同上 | 4.5 m/s |
| 4 个轮电机 ID | `Chassis/Instance/component/ins_motor.cpp` | **1,2,3,4（电路拓扑.png 已确认）** |
| 轮电机减速比 | 同上 | **待确认**：图写「减速比 14」，出厂 19.2 |
| 4 个轮电机方向 | 同上 | **全部 `kDirRev`（反向，已补全版 6.2）** |
| 4 个轮角度偏移 | 同上 | **全 0（已补全版 6.2）** |
| yaw 电机 ID | `Gimbal/Instance/component/ins_motor.cpp` | **2（电路拓扑.png 已确认）** |
| pitch 电机 ID | 同上 | **1（电路拓扑.png 已确认）** |
| yaw / pitch 方向与偏移 | 同上 | **yaw 正向 / −1.36920，pitch 反向 / 0.02480（已补全版 6.2）** |
| pitch 软件限位 | `Gimbal/Task/component/main_task.cpp` | −0.35 ~ +0.60 rad（**待实测**） |
| BMI088 片选引脚 | 两板 `ins_imu.cpp` | **PA4（加速度计）/ PB0（陀螺仪）—— 已由 C_project 样例确认** |
| Mahony 增益 | 两板 `inc/imu.hpp` | **kp = 0.5 / ki = 0（与 C_project 一致）** |
| CAN 波特率 | `Core/Src/can.c`（CubeMX 生成） | **1 Mbps（C_project.ioc 同为 1 Mbps）** |
| 轮速环 PID | `Chassis/Instance/component/ins_pid.cpp` | kp=200, ki=20 |
| yaw 位置环 PID | `Gimbal/Task/component/main_task.cpp` | kp=6, kd=0.15 |
| pitch 位置环 PID | 同上 | kp=5, kd=0.12 |
| 跟随环 PID | `Chassis/Task/component/main_task.cpp` | kp=4, kd=0.05 |
| CAN 通道分配 | 两板 `comm_task.cpp` | 电机=hcan1 / 双板=hcan2（**待按实车接线确认**） |

> `电路拓扑.png` 是**系统拓扑**（板卡、滑环、航插、电机的连接关系），不是 PCB 原理图，
> 因此只能确认「电机 ID 与减速比」这一层，引脚级（BMI088 片选、CAN 收发器接哪路）
> 必须查 PCB 原理图或用万用表量。

---

## 六、无硬件验证清单

```bash
# 1) 纯逻辑自测（PC 上跑，不需要硬件）
cmake -S host_tests -B host_tests/build -G Ninja -DCMAKE_CXX_COMPILER=D:/mingw64/bin/g++.exe
cmake --build host_tests/build
./host_tests/build/host_tests.exe

# 2) 两板固件编译（只验语法/链接，不运行）
cmake -S Chassis -B Chassis/build/w -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build Chassis/build/w
cmake -S Gimbal  -B Gimbal/build/w  -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build Gimbal/build/w
```

上板前必须全绿；上板后按 `docs/bringup_checklist.md` 的顺序逐项点亮。
