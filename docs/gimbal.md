# 云台（Gimbal）实现说明

> 云台板负责：yaw / pitch 两轴位置环、**绝对指向保持**、与底盘板的双板通信。
> 底盘板负责逆解和轮速环，两者通过 CAN1 的 8 字节帧交换状态。

---

## 一、硬件拓扑

```
                    ┌──────────────────────────────┐
   遥控器(可选) ────►│  USART3 / DBUS               │
                    │                              │
   ┌────────────────┤  云台板 STM32F407IG          │
   │  CAN1 ────────►│                              │
   │                │  SPI1 ──► BMI088 (IMU)       │
   │                └──────────────────────────────┘
   │                        │ CAN1
   │                        ├──► DM-J4310 yaw  (ID 2，反馈 0x12)
   │                        ├──► DM-J4310 pitch(ID 1，反馈 0x11 期望)
   │                        └──► 底盘板（0x210/0x211 双板帧）
   │
   └── 底盘板
```

| 项目 | 配置 |
|---|---|
| 主控 | STM32F407IGH6 |
| IMU | BMI088，SPI1，CS：加速度计 PA4 / 陀螺仪 PB0 |
| yaw 电机 | DM-J4310，CAN ID **2**，反馈 ID `0x12` |
| pitch 电机 | DM-J4310，CAN ID **1**，反馈 ID `0x11`（期望值） |
| 控制周期 | 1 ms（TIM6） |
| 看门狗 | IWDG ≈ **600 ms** |

---

## 二、控制结构

### 2.1 一条指令的路

```
摇杆(右摇杆横向/纵向)
   │  归一化 [-1,1]，先过死区
   ▼
目标角积分：target += 归一化值 × 最大角速度 × dt
   │            （yaw 6 rad/s，pitch 3 rad/s）
   ▼
位置环 PID：err = target − feedback
   │
   ▼
限幅（max_torque）
   │
   ▼
MIT 通道纯转矩输出 → 达妙电机
```

源码位置：

| 环节 | 文件 |
|---|---|
| 摇杆 → 步长 | `Gimbal/RobotModules/component/robot.cpp` `GenerateCommands()` |
| 目标积分 / 限位 | `Gimbal/RobotModules/component/gimbal.hpp` `MoveBy()` |
| 位置环 | 同上 `Gimbal::Update()` |
| 输出 | `Robot::OutputMotors()` |

### 2.2 反馈量：yaw 用绝对角，pitch 用关节角

这是云台最关键的一个设计：

```cpp
err_.yaw   = AngleDiff(target_.yaw,   imu_->yaw());   // 绝对角（IMU）
err_.pitch = AngleDiff(target_.pitch, pitch_motor_->angle());  // 关节角
```

| 轴 | 反馈 | 原因 |
|---|---|---|
| **yaw** | **IMU 绝对角**（相对开机姿态） | 关节角只能让云台「相对车体不动」。底盘一转，关节角不变 → δ 不减小 → 跟随模式永远追不上、小陀螺枪口跟着车转 |
| **pitch** | 关节角 | 俯仰是相对车体的量，与车体偏航无关，用关节角最直接 |

⚠️ **代价**：Mahony 没有磁力计，IMU 的 yaw 是「相对开机姿态」，会缓慢漂移（度/分钟级）。
演示时长内可接受；若要更稳，可叠加底盘角速度前馈（考核说明的加分项）。

### 2.3 输出方式：MIT 通道 + 纯转矩

```cpp
AsDaMiao(yaw)->setMitInput(angle, 0.0f, /*kp=*/0, /*kd=*/0, torque);
```

- `mit_kp = mit_kd = 0` → 达妙内部位置环关闭，退化成**纯转矩**
- 位置环完全由我们自己写的 PID 负责（便于整定、便于在 PC 上单测）
- `SetMitParams()` 从未被调用，所以不存在「双环打架」

### 2.4 限位

| 量 | 值 | 说明 |
|---|---|---|
| `pitch_min` | −0.35 rad | **待实测**（约 −20°） |
| `pitch_max` | +0.60 rad | **待实测**（约 +34°） |
| 越界处理 | 到限位后该方向误差强制置 0 | 顶住而不是抖 |

yaw 不限位（可连续旋转），靠 `AngleDiff` 做 ±π 归一化。

---

## 三、与底盘板的配合

### 3.1 遥控从哪来

```
方案 A：接收机接在云台板 → 本地读 DBUS
方案 B：接收机接在底盘板 → 底盘把归一化摇杆值塞进 yaw_des/pitch_des 转发
        （本车就是这种情况）
```

代码里：

```cpp
if (local_rc_online) {
    yaw_norm = -ApplyStickDeadzone(rc_->rc_rh());
} else if (comm 在线) {
    yaw_norm = Clamp(comm_->chassis_data().yaw_des, -1.0f, 1.0f);
}
```

`yaw_des/pitch_des` 名义上是「期望目标角」，实际传的是**归一化摇杆速率**。
这是有意偏离，答辩时要说明。

### 3.2 云台回报给底盘的东西

| 字段 | 内容 | 底盘拿它干什么 |
|---|---|---|
| `yaw` | **yaw 关节角**（不是绝对角） | 作为 δ = 云台朝向 − 车头朝向 |
| `pitch` | pitch 关节角 | 仅显示 |
| `ready` | 是否 Working | 联调状态 |
| `flags.online` | 本板活着 | `peer_online` |

⚠️ 注意：云台**内部目标**是绝对角，但**对外报告的是关节角**——因为底盘要的是「云台相对车体」的偏航。

### 3.3 双板互保

- 底盘报急停 → 云台也 `estop_ = true` → 全零输出
- 任一方离线 100 ms → 另一方进 Dead

---

## 四、参数表

| 参数 | 当前值 | 位置 | 状态 |
|---|---|---|---|
| `max_yaw_rate` | **2.0 rad/s** | `main_task.cpp` | 实车手感 |
| `max_pitch_rate` | **1.0 rad/s** | 同上 | 实车手感 |
| `max_torque`（yaw） | **1.0 N·m** | 同上 | **红线，不改** |
| `max_torque_pitch` | **2.0 N·m** | 同上 | 重力轴，操作手批准的放宽 |
| `pitch_min/max` | **−0.008 / +0.583 rad** | 同上 | 机械 −0.111/+0.687，两端余量 6° |
| yaw 环 `kp/ki/kd` | **3.5 / 0.02 / 0.35** | `main_task.cpp`（`g_yaw_*`，可在线改） | 已整定：0.5 rad 斜坡超调 0.8° |
| pitch 环 `kp/kd` | **10.0 / 0.25** | 同上 | 已整定；ki 固定 0 |
| 重力前馈 `g_pitch_grav_ff` | **0.60 N·m** | 同上 | 实测 G=0.60 |
| 摇杆死区 | 0.05 | `module_state.hpp` | 实测遥控器有 −0.055 偏置，可能需要中位标定 |
| 反馈超时 | 50 ms | `module_state.hpp` | 超时即判离线、输出归零 |
| 姿态解算 | Mahony kp=0.5 / ki=0 | `imu.hpp` | 与参考工程一致 |

---

## 五、在线整定 PID（不用重烧）

yaw 的三个增益放在 `volatile float` 全局里，每 1 ms 写进 PID：

```powershell
# 1) 查地址
& arm-none-eabi-nm -C Gimbal\build\omni_gimbal.elf | Select-String "g_yaw_k"

# 2) 写值（mem32 写 IEEE-754 位模式）
#    1.5f=0x3FC00000  2.0f=0x40000000  3.0f=0x40400000  0.35f=0x3EB33333
```

流程：

1. 手推云台给一个阶跃，读 `err_` / `output_`
2. 先加 `kp` 到「刚好不抖」
3. 再加 `kd` 压超调
4. 有稳态误差再补少量 `ki`
5. 定稿后写回 `MakeYawConfig()`

判据：超调 < 5%、无高频抖动、松手后能稳住。

---

## 六、已知问题

| 问题 | 现象 | 状态 |
|---|---|---|
| **pitch 无反馈** | `angle=0`、温度 `0/0` | ✅ 已修：CAN 滤波器/中断 bug，不是硬件（known_issues §3） |
| 绝对角漂移 | 长时间运行枪口缓慢偏移 | Mahony 无磁力计，已知限制（度/分钟级） |
| 关节角 vs 绝对角混用 | 内部目标是绝对角，对外报关节角 | 有意设计，但容易读错 |
| 大角度超调的硬下限 | 0.5 rad 斜坡仍有 ~0.8° 超调 | 1 N·m 限幅决定，kp/kd 再调无效；见 known_issues §14 |
| 陀螺 z 与 euler yaw 反号 | D 项曾因此变成正反馈（越加 D 越振） | ✅ 已修：`yawRate()` 取负（known_issues §13） |

---

## 七、验证清单（云台单独就能做）

| # | 验证项 | 判据 |
|---|---|---|
| 1 | 板子活着 | `uwTick` 持续增长，`RCC_CSR` 无 IWDG 位 |
| 2 | IMU 收敛 | 上电 1.5 s 后 `imu_ready=1` |
| 3 | yaw 电机在线 | `dm_status_` 温度非 0、`is_enabled=1` |
| 4 | 状态机 | `peer`、`rc` 都满足后 `life_state_=2` |
| 5 | **绝对角闭环** | **用手转云台，松手后回到原角度** ← 已验证 |
| 6 | 摇杆驱动 | 推右摇杆横向，`target` 变化、云台跟着转 |
| 7 | 松手保持 | 松杆后停住，不漂不抖 |
| 8 | 失效保护 | 拔双板 CAN → 100 ms 内归零 |

> 第 5 项是判断「绝对角闭环是否真的生效」的关键：
> 如果闭环在关节角上，云台被转走后会停在原地，**不会回位**。
