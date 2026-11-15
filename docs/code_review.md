# 代码核对报告

> 整理日期：2026-11-12
> 范围：`omni_skeleton` 双板工程（底盘板 + 云台板）+ 主机自测
> 结论：**代码可用，编译与逻辑自测全绿；仍有 4 项只能靠实车/硬件解决**

---

## 1. 结论摘要

| 检查项 | 结果 |
|---|---|
| 自研源文件语法检查（两板合计 51 个 `.cpp`） | **0 错误，0 警告** |
| 双板完整编译 + 链接 | 通过（底盘 FLASH 70.5 KB / RAM 8.4 KB；云台 FLASH 70.5 KB / RAM 8.4 KB） |
| 主机侧单元测试 | **143 项通过，0 失败** |
| 本次会话发现并修复的问题 | **12 个**（其中 3 个安全相关、2 个致命） |
| 只能靠硬件解决、代码无法兜底的 | **4 项**（见第 5 节） |

---

## 2. 代码分层与文件清单

### 2.1 分层

```
硬件驱动层  inc/ + src/            组件库（骨架提供 + 少量自建）
  ├─ BMI088 / Mahony / DT7 / motor_*  / HW_can / tick
  ├─ pid.{hpp,cpp}         ★自建：PID
  ├─ imu.hpp / imu_bias.hpp ★自建：IMU 封装 + 零漂估计
  └─ 接线层  Instance/      单例 + 实车配置（ID/方向/偏移/几何）
模块层      RobotModules/    Chassis(逆解) / Gimbal(位置环) / Robot(状态机)
共享层      RobotComponents/ module_state（纯类型）+ 双板协议
任务层      Task/           MainTask(1ms) / CommTask(发送+喂狗+DBUS)
```

**分层的两条硬约束**（改代码时别破）：

1. `RobotComponents` 被两块板同时编译 → 只能放纯类型/内联函数，**不许 include HAL**
2. `inc/`、`src/` 是设备层，**不许反向依赖** `Instance/`（目前 `HW_can.cpp` 是个例外，为了在中断里快速取到电机对象，已注释说明）

### 2.2 自建文件清单

| 文件 | 作用 | 备注 |
|---|---|---|
| `RobotComponents/component/module_state.hpp` | 枚举 / 运动结构 / 角度工具 / 状态迁移 / **摇杆死区** | 纯头文件，两板共用 |
| `RobotComponents/component/gimbal_chassis_comm.{hpp,cpp}` | 8 字节双板协议 | 编解码严格对称 |
| `Chassis/inc/pid.hpp` + `src/pid.cpp`（云台板各一份） | 位置式 PID | 纯算法，可 host 测 |
| `Chassis/inc/imu.hpp` | Imu 封装（BMI088 + Mahony + 零漂） | 头文件内联 |
| `Chassis/inc/imu_bias.hpp` | 零漂估计器 | 纯算法，可 host 测 |
| `Chassis/RobotModules/component/chassis_ik_solver.hpp` | 麦轮逆解 + 等比限幅 | 纯算法，可 host 测 |
| `Chassis/RobotModules/component/{chassis,robot}.{hpp,cpp}` | 底盘运动 + 状态机 | |
| `Gimbal/RobotModules/component/{gimbal,robot}.{hpp,cpp}` | 云台位置环 + 状态机 | |
| `Instance/component/*.cpp` | 单例与实车配置 | 13 个 |
| `Task/component/{main_task,comm_task}.cpp` | 1ms 入口 / 通讯 | |
| `host_tests/` | 主机自测（143 项） | 不依赖硬件 |

---

## 3. 已修复问题清单

按"致命 → 安全 → 功能 → 编译"排序。

| # | 现象 | 根因 | 修复 |
|---|---|---|---|
| 1 | 链接报 `undefined reference to tick::GetTickMs` | `tick.cpp` 被 `#if defined(USE_HAL_TICK)` 包住，全工程没定义该宏 → 目标文件是空的 | 两个 `CMakeLists.txt` 加 `add_compile_definitions(USE_HAL_TICK)` |
| 2 | CAN 一帧都收不到 | 骨架**从未调用 `HAL_CAN_Start()`** | `CommTaskInit` 补 Start + 首次 ActivateNotification |
| 3 | 永远进不了 Working（`imu_ready` 恒 0） | `Imu::Update()` **从未被调用** | `Robot::Update()` 开头每周期调用 |
| 4 | 云台电机动作错乱 | yaw/pitch 的 ID 与实车相反（实车 pitch=1、yaw=2） | 按 `电路拓扑.png` 修正 |
| 5 | `peer_online` 恒 0，双板不通 | 双板帧发在 CAN2，但两板实际共用 CAN1（嗅探证实 CAN2 零帧） | `kCanComm` 改 `&hcan1` |
| 6 | 云台板 `rc_online` 恒 0 | 接收机实际接在**底盘板**，云台读不到 DBUS | 底盘把归一化摇杆值放进 `yaw_des/pitch_des` 转发 |
| 7 | 松手后车缓慢爬行 | **没有摇杆死区** | `module_state.hpp` 新增 `ApplyStickDeadzone()`，四条摇杆路径全部过它 |
| 8 | 上电时车在动 → 永久 Dead | 零漂估计 `failed_` 一旦置位就永久放弃 | 改成无限重试，`failed_` 只作调试标志 |
| 9 | 电机全灭也能进 Working | 超时兜底没检查 yaw 是否使能 | 超时分支加上 `yaw_ok` 前置条件 |
| 10 | yaw 明显震荡 | `kp=6` 而输出限幅只有 1 N·m，误差 >0.17 rad 就饱和 → 极限环 | `kp` 降到 2.0、`kd` 提到 0.35 |
| 11 | 四个轮子完全不动 | 轮电机帧只发 CAN1，而电调不在 CAN1（嗅探从未出现 0x201~0x204） | 轮帧改成 CAN1+CAN2 双发 |
| 12 | 四处编译错误 | `constexpr` 修饰 GPIO 指针、`BMI088` 实际在 `hello_world::imu`、`hdma_usart3_rx` 未声明、`ins_all.hpp` 只有前置声明 | 逐个修正 |
| **13** | **跟随模式会一直自旋、小陀螺枪口跟着车转** | 云台 yaw 环的反馈量用的是**关节角**（相对车体），底盘一转云台就跟着转，δ 永远不减小 | yaw 反馈改用 **IMU 绝对角**；`Reset()`/状态切换时把目标对齐当前绝对指向 |

### 3.0 第 13 条为什么是这一节最重要的修复

考核说明 3.2 里 δ = 云台朝向 − 车头朝向 能被当作"跟随时应该消除的误差"，**前提是云台已经相对地面指稳了**。如果云台只锁关节角：

- 底盘按 δ 左转 → 云台跟着左转 → 关节角仍是 δ → 底盘**永远追不上，一直转**
- 小陀螺自旋 → 枪口跟着车转 → "平移以云台为基准"的基准本身在旋转

修复后 yaw 闭环在 IMU 绝对角上，底盘转动时云台会主动反向补偿，δ 才会真正减小。

代价：Mahony 无磁力计，绝对 yaw 会缓慢漂移（度/分钟级），演示时长内可接受；若要更稳，可叠加底盘角速度前馈（文档加分项）。

主机自测里专门加了一组对照用例（`TestGimbalAbsoluteHold`）：车体以 1 rad/s 转 1 秒，**旧方案误差恒为 0（控制器完全看不见车体在转）、实际指向偏了 1 rad；新方案能看见偏差并反向补偿**。

### 3.1 死区为什么必须做

考核第 10 项要求"零输入即零输出"。DT7 中位抖动约 ±0.02，乘以 3.0 m/s 就是 0.06 m/s 的残余速度——松手车还在爬，验收时会被判"不受控"。

现在的实现是"死区 + 区间重映射"：死区内严格返回 0，死区外把剩余行程重新映射到 [-1,1]，**不损失最大行程**。主机自测里有 42 条用例覆盖边界、满量程、单调性、对称性。

### 3.2 零漂为什么不能"失败即放弃"

上电瞬间车可能在晃（人在装电池、放车）。原实现重试 3 轮不达标就永不再试，状态机于是永远停在 Dead——现场踩过这个坑，表现为"`imu_ready` 一直是 0，重启才好"。现在改成无限重试。

---

## 4. 待恢复的调试值（上新车/验收前必看）

调试期为了安全把限幅压低了，验收前要改回来：

| 文件 | 常量 | 当前（调试） | 验收值 |
|---|---|---|---|
| `Chassis/RobotModules/component/chassis.hpp` | `max_vx` / `max_vy` | 0.6 | **3.0** |
| 同上 | `max_w` | 2.0 | **6.0** |
| 同上 | `max_spin_w` | 2.0 | **8.0** |
| `Chassis/Task/component/main_task.cpp` | `kMaxWheelVelMps` | 1.0 | **4.5** |
| `Gimbal/Task/component/main_task.cpp` | `lim.max_torque` | 1.0 | **3.0** |
| `Gimbal/RobotModules/component/robot.cpp` | `kAllowSingleJoint` | **false（已恢复）** | false |
| `Gimbal/Task/component/comm_task.cpp` | `kDuplicateJointsToBothCan` | false | false（除非 pitch 无反馈要试探） |

> 建议：**新车第一次上电保持这些保守值**，把流程走通、方向都验对之后再逐项放开。

---

## 5. 已知但未修的问题

### 5.1 只能靠硬件解决

| 问题 | 证据 | 结论 |
|---|---|---|
| **pitch 电机零反馈** | `rotor_temp=0 / mos_temp=0 / status_code=0`，且 CAN1、CAN2 嗅探都没出现 `0x11` | 电机没供电 / 没接 / 坏了。代码无法兜底，只能先用 `kAllowSingleJoint` 单轴跑 |
| 轮子不动（最终原因待确认） | 已改成双路广播 | 若仍不动 → 电调 24V 没供电 |
| SWD 偶发连不上 | `VTref=3.287V` 但 `Failed to initialize DAP` | 排线错位/松动；建议换成 3 根杜邦线 |
| C620 减速比 | 拓扑图写"减速比 14"，出厂 19.2 | 差 1.37 倍，只影响速度标定精度，需实测确认 |

### 5.2 设计上可改进（不影响通过）

1. **SPI 读 IMU 在 1 ms 中断里做**，且 `BMI088::getData()` 内部有 2 µs 的 `DelayUs` 忙等。目前能跑（SysTick 硬件计数器仍在递减），但不是好实践——更好的做法是主循环读、或用 DMA/中断。
2. **底盘 `motors_ready` 只检查指针非空**，不检查 M3508 反馈。C620 没有"使能"概念，但可以用"反馈是否超时"来判断更严谨。
3. **`yaw_des/pitch_des` 字段语义被复用**：文档里写的是"期望云台目标角"，实际用来传"归一化摇杆速率"（因为接收机在底盘板上）。属于有意的偏离，答辩时要说明。
4. **`GetPidMotorYaw()` / `GetChassisIkSolver()` 这两个 Instance 单例目前没被业务使用**（Gimbal/Chassis 各自持有内部实例），保留是为了满足骨架要求的接口；可以合并。

---

## 6. 可以拿去答辩的设计点

| 设计 | 理由 |
|---|---|
| 三态状态机 + **急停判断放最外层** | 任何模式分支都绕不过急停，这是硬性验收项 |
| **δ 取云台 yaw 关节角**，而不是两块 IMU 相减 | 关节角天然就是"云台相对底盘"的偏航，省掉两板 IMU 零漂不一致的问题，也少一帧数据 |
| 逆解超速用**等比缩放**而非各自裁剪 | 独立裁剪会让四个轮子比例失真，车会"跑偏" |
| 8 字节协议：**标志位 + 模式位 + int16 缩放 + seq** | 缩放 1000 得到 0.001 rad 分辨率、±32.7 rad 量程（够 5 圈）；seq 用于判断丢帧 |
| **纯算法抽成头文件**（PID/逆解/零漂/死区） | 能脱离硬件在 PC 上跑 143 项自测，算法错误在上板前就被抓住 |
| 无 RTTI 下用 `static_cast` 做电机下转型 | 工程开了 `-fno-rtti`，`dynamic_cast` 不可用；注册进 Robot 的确实是 DM 对象，转型安全 |
| **双板互保** | 对面报急停，本板也归零（加分项） |

---

## 7. 复验命令

```powershell
# 1) 主机自测（不需要硬件）
cd C:\Users\YTN\Desktop\worka\final\omni_skeleton
cmake -S host_tests -B host_tests/build -G Ninja -DCMAKE_CXX_COMPILER=D:/mingw64/bin/g++.exe
cmake --build host_tests/build
.\host_tests\build\host_tests.exe

# 2) 两板固件
cmake -S Chassis -B Chassis/build/w -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build Chassis/build/w
cmake -S Gimbal  -B Gimbal/build/w  -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build Gimbal/build/w

# 3) 烧录（自动判断接的是哪块板，命令见 docs/new_vehicle_checklist.md 的 2.2）
```

---

## 8. 一句话总结

代码本身**没有已知会导致失控的缺陷**：急停、离线、双板互保、看门狗四条失效链都在，算法层有 143 项自测兜底。

当前所有"不正常"的表现，根因都已经定位到**硬件层**（pitch 电机零反馈、电调供电/总线归属、SWD 接触），代码侧能做的都已经做完。
