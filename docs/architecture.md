# 代码结构与设计决策

> 回答三件事：**代码怎么分层**、**为什么这么设计**、**踩过哪些坑**。

---

## 一、分层

```
硬件驱动层   inc/ + src/
  ├─ 组件库（骨架提供）：BMI088 / Mahony / DT7 / motor_* / HW_can / tick / offline_checker
  ├─ pid.{hpp,cpp}          ★自建：位置式 PID
  ├─ imu.hpp / imu_bias.hpp ★自建：IMU 封装 + 陀螺零漂估计
  └─ bench_test.hpp         ★自建：台架自测方波（默认关闭）

接线层       Instance/      单例 + 实车配置（ID / 方向 / 偏移 / 引脚）

模块层       RobotModules/  Chassis(逆解) / Gimbal(位置环) / Robot(状态机)

共享层       RobotComponents/
  ├─ module_state.hpp       ★纯类型 + 角度工具 + 死区 + 状态迁移 + 帧校验
  └─ gimbal_chassis_comm.*  ★8 字节双板协议

任务层       Task/           MainTask(1 ms) / CommTask(发送 + 喂狗 + DBUS)
```

### 两条硬约束

1. **`RobotComponents/` 被两板同时编译** → 只能放纯类型和内联函数，不许 include HAL
2. **`inc/` `src/` 是设备层**，不许反向依赖 `Instance/`
   （`HW_can.cpp` 是唯一例外：为了在中断里快速拿到电机对象，已注释说明）

### 文件清单（自建部分）

| 文件 | 作用 | 可 host 测 |
|---|---|---|
| `RobotComponents/component/module_state.hpp` | 枚举 / 运动结构 / 角度工具 / 死区 / 状态迁移 / DBUS 帧校验 | ✅ |
| `RobotComponents/component/gimbal_chassis_comm.{hpp,cpp}` | 双板 8 字节协议编解码 | ✅ |
| `Chassis/inc/pid.hpp` + `src/pid.cpp` | PID（两板各一份，内容一致） | ✅ |
| `Chassis/inc/imu_bias.hpp` | 零漂估计器 | ✅ |
| `Chassis/inc/imu.hpp` | BMI088 + Mahony 封装 | ❌ 依赖 HAL |
| `Chassis/RobotModules/component/chassis_ik_solver.hpp` | 麦轮逆解 + 等比限幅 | ✅ |
| `Chassis/RobotModules/component/{chassis,robot}.{hpp,cpp}` | 底盘运动 + 状态机 | 部分 |
| `Gimbal/RobotModules/component/{gimbal,robot}.{hpp,cpp}` | 云台位置环 + 状态机 | 部分 |
| `Gimbal/inc/bench_test.hpp` | 台架自测目标发生器 | ✅ |
| `host_tests/` | 主机侧自测（**328 项**） | — |

---

## 二、一条指令走完的路

```
遥控器 ──DBUS──► 底盘板 CommTask
                    │ decode + 帧校验
                    ▼
            Robot::UpdateInputs      ← 离线判定、急停、模式、δ
                    ▼
            Robot::GenerateCommands  ← 摇杆 → 云台系 (vx,vy,w)
                    ▼
            Chassis::Update
                    │ GimbalToChassis(v, δ)   ← 云台系 → 车体系，只转一次
                    │ ChassisIkSolver::SolveAndLimit  ← 逆解 + 等比限幅
                    ▼
            Robot::OutputWheels
                    │ m/s → rad/s → 速度环 PID → raw
                    │ 反馈看门狗：不新鲜就清积分 + 输出 0
                    ▼
            CommTask ──CAN──► C620 ×4
```

云台侧多一层：`IMU 绝对 yaw` 作为反馈 → `PID` → `N·m` → MIT 通道。

---

## 三、关键设计决策

| # | 决策 | 理由 | 代价 |
|---|---|---|---|
| 1 | **云台 yaw 闭环在 IMU 绝对角上**（不是关节角） | 关节角闭环只能让云台「相对车体不动」。底盘一转 δ 不减小 → 跟随永远追不上、小陀螺枪口跟车转 | Mahony 无磁力计，yaw 会缓慢漂移（度/分钟）；可加角速度前馈缓解 |
| 2 | **δ 直接取云台 yaw 关节角**（不用双 IMU 相减） | 关节角天然就是「云台相对底盘」，省掉两板零漂不一致的问题，也少一帧数据 | 依赖关节角本身准确（电机绝对编码器，没问题） |
| 3 | **逆解超速用等比缩放**，不是各自裁剪 | 独立裁剪会破坏四轮速度比例 → 车跑偏 | 限速时整体变慢（正确行为） |
| 4 | **摇杆死区 + 区间重映射** | DT7 中位抖动 ±0.02，×3 m/s 就是 0.06 m/s 的爬行，违反「零输入即零输出」 | 无（重映射后不损失满行程） |
| 5 | **三态状态机 + 急停判断放最外层** | 任何模式分支都绕不过急停，这是硬性验收项 | 无 |
| 6 | **「进自检」与「进工作」前提拆开** | 自检本来就是去等电机使能，若也要求电机就绪 → 死锁（实测踩过） | 需要两个谓词 `prerequisitesOk()` / `allReady()` |
| 7 | **电机就绪超时判定与状态无关** | 曾写成「只在 Resurrection 生效」，导致进 Working 后又失效 → 2 s 循环 | 需要记录 `boot_tick_` |
| 8 | **DBUS 校验帧内容，不只看长度** | 接收机掉线时缓冲全 0，解出 −1.55 → 满量程指令 → 云台自己匀速转（实测踩过） | 多一层判断；非法帧不刷新在线状态 |
| 9 | **电机反馈看门狗**（`IsFeedbackFresh`） | 反馈断了 `vel()` 恒 0，速度环积分涨到限幅，电调恢复瞬间猛冲（实测 `out_` 涨到 3500 还在涨） | 超时阈值需按实际帧率定（现 50 ms） |
| 10 | **纯算法抽成头文件** | 能脱离硬件在 PC 上跑 328 项自测，算法错误在上板前就被抓住 | 需要维护 host_tests |
| 11 | **无 RTTI 下用 `static_cast` 下转型** | 工程开了 `-fno-rtti`；注册进 Robot 的确是 DM 对象 | 转型不安全时不会报错，靠约定 |
| 12 | **双板互保** | 对面报急停，本板也归零 | 需要 `peer_online` 先成立 |

---

## 四、已修复的问题（按严重度）

| # | 问题 | 根因 | 修法 |
|---|---|---|---|
| 1 | 链接报 `tick::GetTickMs` 未定义 | `tick.cpp` 被 `USE_HAL_TICK` 包住，全工程没定义该宏 | 两个 CMakeLists 加 `add_compile_definitions` |
| 2 | CAN 一帧收不到 | 骨架从未调用 `HAL_CAN_Start()` | `CommTaskInit` 补启动 |
| 3 | 永远进不了 Working | `Imu::Update()` 从未被调用 | 控制周期开头调用 |
| 4 | **云台固件烧进了底盘板** | 换线后没确认板子身份 | 烧录前先读 UID + 比对 Reset 入口 |
| 5 | 跟随失效 / 小陀螺枪口跟车转 | yaw 闭环在关节角上 | 改为 IMU 绝对角 |
| 6 | 状态机死锁：一颗电机坏了永不 Working | `Dead→Resurrection` 也要求 `motors_ready` | 拆出 `prerequisitesOk()` |
| 7 | 每 2 s 循环一次（进 Working 立刻被打回） | 超时判定依赖 `life_state_` | 改成上电计时，与状态无关 |
| 8 | 云台自己匀速转 | DBUS 空帧 → 满杆 | 校验帧内容 |
| 9 | 推杆后电流一直涨 | 轮速环无反馈看门狗 | `IsFeedbackFresh` + 清积分 + 输出 0 |
| 10 | 松手后车缓慢爬行 | 没有摇杆死区 | `ApplyStickDeadzone`（4 条摇杆路径全覆盖） |
| 11 | 零漂失败后永久 Dead | `failed_` 一旦置位不再重试 | 改无限重试，只做调试标志 |
| 12 | 双板不通 | 双板帧发在 CAN2，实车共用 CAN1 | 改 `kCanComm = &hcan1` |
| 13 | 轮子不动（部分原因） | 轮电机帧只发一路 CAN | 双路 + 双 ID 广播（确认后应收回） |
| 14 | yaw 明显震荡 | `kp=6` 配 1 N·m 限幅，误差 0.17 rad 就打满 | `kp=2.0`、`kd=0.35` |
| 15 | 编译错误若干 | `constexpr` 修饰 GPIO 指针、`BMI088` 命名空间、`hdma_usart3_rx` 未声明、前置声明缺失 | 逐个修正 |
| 16 | 死区函数可能除零 | `deadzone >= 1` 时 `(1-dz)` 除零 | 钳到 0.99 |

---

## 五、已知问题

### 5.1 需要硬件解决（代码兜不住）

| 问题 | 证据 | 结论 |
|---|---|---|
| 轮电机无任何反馈 | 嗅探表两路 CAN 全 0 帧（连 `0x201~0x208` 都没有） | 电调没供电 / CAN 没接 / ID 不对 |
| 遥控器无数据 | `g_dbus_rx_events = 0`（串口零字节） | 接收机没接 / 没电 / 引脚不对 |
| pitch 电机无反馈 | 云台报 `pitch=0` 且 `dm_status_` 全 0 | 待查 ID 配置（CAN 上出现 `0x001/0x002`，可疑） |
| SWD 偶发连不上 | `VTref=3.3V` 但 `Failed to initialize DAP` | 排线错位，建议换 3 根杜邦线 |
| M3508 减速比 | 拓扑图写 14，出厂 19.2 | 差 37% 速度标定，需实测 |

### 5.2 设计上可改进（不影响通过）

1. **IMU 的 SPI 读在 1 ms 中断里**，`getData()` 内部还有 2 µs 忙等。能跑，但不是最佳实践
2. **底盘 `motors_ready` 只看指针非空**，电调全死也判就绪（现由输出看门狗兜底）
3. **CAN1 负载**：现在 0x200 + 0x1FF + 0x211 三帧/ms，约 39% 负载；确认 ID 后应删掉多余的一半
4. **`yaw_des/pitch_des` 字段语义被复用**：文档写「期望云台目标角」，实际传「归一化摇杆速率」。
   属有意偏离（接收机在底盘板上），答辩要说明
5. **两个 Instance 单例未被业务使用**（`GetPidMotorYaw` / `GetChassisIkSolver`），保留为骨架接口

---

## 六、复验命令

```powershell
$repo = '<你的仓库路径>'

# 1) 主机自测（不需要硬件，期望 328 项全过）
cmake -S "$repo\host_tests" -B "$repo\host_tests\build" -G Ninja -DCMAKE_CXX_COMPILER=D:/mingw64/bin/g++.exe
cmake --build "$repo\host_tests\build"
& "$repo\host_tests\build\host_tests.exe"

# 2) 两板固件
cmake --build "$repo\Chassis\build\w"
cmake --build "$repo\Gimbal\build\w"
```

---

## 七、答辩可讲的设计点

| 讲什么 | 怎么讲 |
|---|---|
| **坐标系** | 平移定义在云台系，解算前按 δ 旋转到车体系；δ 每 1 ms 刷新，所以小陀螺下方向不漂 |
| **为什么 yaw 用绝对角** | 关节角只能锁「相对车体」，底盘一转 δ 就不变；跟随会一直转、小陀螺枪口跟车转 |
| **为什么 δ 用关节角** | 关节角本身就是相对量，省去双 IMU 零漂不一致 |
| **等比限幅** | 独立裁剪会让四轮比例失真导致跑偏，等比缩放保证方向不变 |
| **三条失效链** | 急停（拨杆）/ 离线（100 ms 超时）/ 看门狗（300·600 ms），任一条都归零 |
| **四层防御** | 状态机、输出看门狗、帧内容校验、PID 积分钳位 |
| **可测性** | 纯算法抽到共享头文件，328 项 PC 自测，上板前抓出算法错误 |
