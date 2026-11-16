# 上板 Bring-up 手册

> 原则：**先能看见 → 先能停住 → 一次只改一个变量。**
> 每一步都有判据，不满足不许进下一步。

---

## 0. 准备（不上电）

- [ ] 两板固件能编过（见 [README](README.md) 的快速开始）
- [ ] 主机自测 328 项全绿
- [ ] [parameters.md](parameters.md) 里的**安全相关参数**已按实车改过（至少云台 pitch 限位）
- [ ] 四轮离地，云台周围无干涉
- [ ] 遥控器急停位放好，手放在电源开关/拨杆上

⚠️ **本工程默认是「调试保守值」**：`max_vx=0.6`、`max_torque=1.0`、`kMaxWheelVelMps=1.0`。
正式验收前按 [parameters.md](parameters.md) 恢复。

---

## 1. 保活（第一件事）

- [ ] `CommTask()` 里 `HAL_IWDG_Refresh(&hiwdg)` 已就位
- [ ] 板子能连续跑 > 1 s 不复位

判据：读 `RCC_CSR`（`0x40023874`），**bit29 (IWDG) 必须为 0**。
不为 0 说明在反复看门狗复位，什么都调不了（底盘 0.3 s / 云台 0.6 s 一轮）。

---

## 2. 建立观测通道

两条路，按需要选：

| 方式 | 优点 | 缺点 |
|---|---|---|
| **J-Link / Ozone 读内存** | 不需要改代码，能看任意变量 | 必须有调试器连着 |
| **串口打印**（需自行加） | 脱机可看 | 阻塞发送，1 ms 中断里用要小心 |

⚠️ **打硬件断点前必须断电或四轮离地**：断点会冻结 TIM6 的 1 ms 中断，
电机保持最后一条电流指令继续转。

---

## 3. 认车（只采集信息，不动控制逻辑）

先把结果填进 [parameters.md](parameters.md) 的记录表，**填完再往下走**。

### 3.1 CAN 拓扑（最关键）

必须搞清三件事：

1. 两块板之间用哪一路 CAN 连
2. 4 个 M3508 挂在哪一路
3. 2 个 DM-J4310 挂在哪一路

用固件内置**嗅探表**（`g_rx_total / g_rx_ids / g_rx_cnt / g_rx_uniq`，两板都有）：

```powershell
$repo = '<你的仓库路径>'
$elf  = "$repo\Chassis\build\omni_chassis.elf"
& arm-none-eabi-nm -C $elf | Select-String "g_rx_"
```

读到地址后 `mem32` 读表。判读：

| 看到的 ID | 含义 |
|---|---|
| `0x201`~`0x204` | M3508 电调 ID = 1~4 |
| `0x205`~`0x208` | M3508 电调 ID = **5~8**（要改 `kWheelId`） |
| `0x11`/`0x12` | DM-J4310 反馈（ID + 0x10） |
| `0x210`/`0x211` | 双板帧 |
| **一个都没有** | 电机没供电 / CAN 线没接 |

### 3.2 电机 ID

| 电机 | 查什么 |
|---|---|
| M3508 ×4 | C620 拨码；反馈 ID = `0x200 + id` |
| DM-J4310 yaw / pitch | 达妙上位机读 CAN ID；反馈 ID = `id + 0x10` |

### 3.3 几何参数（拿尺子量）

- 轮半径 `R`
- 半轴距 `W/2`（前后）
- 半轮距 `L/2`（左右）

填进 `Chassis/Task/component/main_task.cpp`。

### 3.4 云台限位

手推到 pitch 上下机械极限，读 `angle()`，各留 5° 余量填进
`Gimbal/Task/component/main_task.cpp` 的 `lim.pitch_min / pitch_max`。

### 3.5 遥控器档位实测

⚠️ **不要假设**：DT7 协议只传「开关值 1/3/2」，不传语义。
不同遥控器（如 DJI Phantom 系列的 `OFF/CL/HL`）标注完全不同。

做法：拨到每一档，读 `rc_l_switch_ / rc_r_switch_`，把映射写进文档。

代码里的默认映射：

| DBUS 值 | 代码解释 |
|---|---|
| 1 | 上 |
| 3 | 中 |
| 2 | 下 |
| 左 = 2 | **急停** |
| 右 = 1/3/2 | 分离 / 跟随 / 小陀螺 |

---

## 4. 通讯打通

### 4.1 SWD（三线接法）

排线最容易错位一格或反插，症状是「有 VTref 但 `Failed to initialize DAP`」。

```
J-Link 20pin 7 脚 → SWDIO
J-Link 20pin 9 脚 → SWCLK
J-Link 20pin 4 脚 → GND
```

3.3V 不用接。连不上先**断电重上电**，再换杜邦线。

### 4.2 烧录

```powershell
$repo = '<你的仓库路径>'
$jl   = 'C:\Program Files\SEGGER\JLink_V978\JLink.exe'
$f    = "$env:TEMP\flash.jlink"

Set-Content -Path $f -Encoding ASCII -Value @(
  'r','h',
  ('loadfile "{0}"' -f "$repo\Chassis\build\omni_chassis.elf"),
  'r','g','qc')
& $jl -device STM32F407IG -if SWD -speed 1000 -autoconnect 1 -ExitOnError 1 -CommanderScript $f
```

云台板把路径换成 `Gimbal\build\omni_gimbal.elf`。

**烧前先确认是哪块板**：

```powershell
$repo = '<你的仓库路径>'
$jl   = 'C:\Program Files\SEGGER\JLink_V978\JLink.exe'
$f    = "$env:TEMP\id.jlink"
Set-Content $f -Encoding ASCII -Value @('si SWD','speed 1000','connect','mem32 0x1FFF7A10 3','mem32 0x08000000 2','q')
& $jl -device STM32F407IG -CommanderScript $f
```

- UID 第三个字不同的两块板 → 各自记下来
- `0x08000000` 的第二个字 = Reset 入口，**和 ELF 的 entry 一致才算新固件**

### 4.3 CAN 嗅探

见 3.1。**这是判断「为什么电机不动」最快的手段**。

### 4.4 DBUS

- [ ] `g_dbus_rx_events` 增长 → 串口在收数据
- [ ] `g_dbus_rx_valid` 增长 → 帧内容合法
- [ ] 摇杆拉满，`rc_lv` 到 ±1

判据：**两个计数都为 0** = 接收机根本没接上（没电/没插/引脚不对）。

---

## 5. 只读验证（不驱动电机）

- [ ] 手转一个轮子，对应 ID 的 `angle()` 连续变化，且只有那个 ID 涨
- [ ] `imu_ready` 在上电 1.5 s 后变 1
- [ ] 双板 `peer_online` 变 1，`rx_count` 持续增长

---

## 6. 单电机与方向（最危险阶段，逐个来）

- [ ] 只给一个轮子 `raw = +500`，持续 2 s，记录实际转向
- [ ] 方向不符 → 改 `ins_motor.cpp` 的 `dir`
- [ ] 四轮正方向一致（推车前进时四轮同号）
- [ ] 云台：确认 DM 的 `is_enabled()` 变 true（上电失能，不发指令不反馈）
- [ ] 云台先给 ±5° 小阶跃，`kp` 从小加，`kd` 必须非 0

方向对照：

| 症状 | 改哪里 |
|---|---|
| 前推车后退 | `ins_motor.cpp` 的 `kWheelDir` |
| 右推车左移 | `robot.cpp` 的 `vy_cmd` 负号 |
| 右推车逆时针 | 同上 `w_stick` 负号 |
| 转云台后前推方向不对 | `module_state.hpp` 的 `kFrameDeltaSign` |
| 拨轮方向反 | `chassis_ik_solver.hpp` 的 `kSpinSign` |

---

## 7. 状态机与失效链

- [ ] 急停档 → 所有电机输入为 0
- [ ] 拔遥控 → 约 100 ms 后 `rc_online=0` → Dead → 全零
- [ ] 拔双板 CAN → 约 100 ms 后 `peer_online=0` → Dead
- [ ] 切状态时 PID 积分被 `Reset()`，切回 Working 不突跳

⚠️ 注意 `reset` 与 `Dead` 的区别：**Dead 只是不输出，不断电**。

---

## 8. 双板通信

- [ ] 先单向：底盘板只发，云台板看 `rx_count` 增长
- [ ] 再双向：同一已知常数（如 1.234 rad）两板解出来差 ≤ 1 LSB
- [ ] 拔任一板 CAN → 另一板约 100 ms 后进 Dead

判据：两端缩放比例一致，否则会「看着通了其实数值差几个量级」。

---

## 9. 三种模式（先离地）

| 模式 | 右拨杆 | 验收动作 |
|---|---|---|
| 分离 | Up | 各方向平移/转向符合直觉，松手立刻停 |
| 跟随 | Mid | **转云台 90°，车头自动追过去并停住**（不是一直转） |
| 小陀螺 | Down | 拨轮控自旋；**平移方向仍随枪口**（核心验收点） |

⚠️ 跟随/小陀螺成立的前提是**云台绝对指稳**（见 [architecture.md](architecture.md) 设计决策 1）。

---

## 10. 看门狗（最后做）

- [ ] 加调试标志跳过喂狗，或用 `__BKPT()` 卡在喂狗前
- [ ] 底盘板约 **300 ms** 复位、云台板约 **600 ms** 复位
- [ ] 复位后能自动恢复（不要求「不死」，只要求「不失控」）

⚠️ **喂狗只能有一个入口**。放中断回调里会掩盖主循环卡死，验收会判「谎称已实现」。

---

## 11. PID 整定

### 11.1 云台 yaw（可在线调，不用重烧）

增益放在三个 `volatile float` 全局里（`Gimbal/inc/pid_tune.hpp`），
每 1 ms 写进 yaw 环：

```powershell
# 查地址
& arm-none-eabi-nm -C "$repo\Gimbal\build\omni_gimbal.elf" | Select-String "g_yaw_k"
# 写值（mem32 写的是 IEEE-754 位模式，不是十进制）
#   1.5f = 0x3FC00000   2.0f = 0x40000000   0.35f = 0x3EB33333
```

步骤：

1. 手推云台给阶跃，读 `err_` / `output_` / yaw 曲线
2. 先加 `kp` 到「刚好不抖」，再加 `kd` 压超调
3. 有稳态误差再补少量 `ki`
4. 定稿后把值写回 `main_task.cpp` 的 `MakeYawConfig()`

### 11.2 其它环

| 环 | 位置 | 起步值 |
|---|---|---|
| 轮速环 | `ins_pid.cpp` | kp=200, ki=20 |
| yaw 位置环 | `main_task.cpp` / `pid_tune.hpp` | kp=2.0, kd=0.35 |
| pitch 位置环 | `main_task.cpp` | kp=5.0, kd=0.12 |
| 跟随环 | `main_task.cpp` | kp=4.0, kd=0.05, 输出 ±6 |

判据：阶跃超调 < 5%、无高频抖动、无稳态漂移。

⚠️ yaw 注意 **359°→1° 过零**（必须过 `WrapToPi`）；
pitch 到限位应「顶住」而不是抖。

---

## 附录 A：常用 J-Link 命令

```powershell
$jl = 'C:\Program Files\SEGGER\JLink_V978\JLink.exe'
$f  = "$env:TEMP\a.jlink"

# 连接测试
Set-Content $f -Encoding ASCII -Value @('si SWD','speed 1000','connect','q')

# 读 UID（区分板子）         mem32 0x1FFF7A10 3
# 读复位原因                 mem32 0x40023874 1
#   bit29=IWDG  bit28=软件  bit27=POR  bit26=PIN  bit25=BOR
# 读向量表（确认固件版本）    mem32 0x08000000 2
#   [0]=初始 SP  [1]=Reset 入口
# 读故障寄存器               mem32 0xE000ED28 1   (CFSR)
#                            mem32 0xE000ED2C 1   (HFSR)

& $jl -device STM32F407IG -CommanderScript $f
```

## 附录 B：符号地址怎么查

```powershell
# 全局/静态变量
& arm-none-eabi-nm -C "$repo\Gimbal\build\omni_gimbal.elf" | Select-String "GetRobot|g_rx_"

# 类成员偏移（DWARF）
& arm-none-eabi-readelf --debug-dump=info "$repo\Gimbal\build\omni_gimbal.elf" |
  Select-String "life_state_" -Context 0,6
```

⚠️ 两个坑：

1. **每次重新编译地址都会变**，不要复用旧地址
2. `nm` 里会有 `guard variable for GetRobot()::robot`，那是编译器的静态初始化保护变量，
   **不是对象本身**。要用精确匹配（正则结尾加 `$`）

## 附录 C：故障对照表

| 症状 | 最可能原因 | 一句话验证 |
|---|---|---|
| 烧不进去 `Missing parameter` | 命令被换行截断 | 先 `Get-Content $f` 看行数 |
| `VTref` 有但 `DAP` 失败 | SWDIO/SWCLK 没通 | 换 3 根杜邦线 |
| 板子周期性复位 | 没喂狗 | 读 `RCC_CSR` bit29 |
| 收不到任何 CAN 帧 | 没调 `HAL_CAN_Start()` | 读嗅探表是否全 0 |
| 只能收到一帧 DBUS | 回调里没重启接收 | 看 `g_dbus_rx_events` 是否只涨 1 |
| `rc_online` 恒 0 | 接收机没数据 / 帧非法 | `g_dbus_rx_events` 与 `g_dbus_rx_valid` |
| 轮子不动 | 电调没电 / CAN 没接 / ID 不对 | 嗅探表有没有 `0x201~0x208` |
| 静止时云台自己匀速转 | 摇杆空帧被当成满杆 | 看 `yaw_des` 是否恒为 ±1 |
| 推杆后电流一直涨 | 电调无反馈，速度环积分饱和 | 看 `pids[i].out_`；现有看门狗兜底 |
| 跟随模式越转越快 | 云台没绝对指稳，δ 不减小 | 转云台看 `delta_` 有没有变 |
| 云台 359° 附近来回甩 | 角度差没过 `WrapToPi` | 直接读 `err_.yaw` |
| 双板「通了」但车乱跑 | 两端缩放比不一致 | 对同一常数比数值 |

## 附录 D：踩过的坑

1. **骨架没调 `HAL_CAN_Start()`** —— 滤波器配好了、回调也挂了，但 CAN 没启动，一帧收不到
2. **`tick.cpp` 缺 `USE_HAL_TICK` 宏** —— 整个文件被预处理掉，链接报 undefined reference
3. **`Imu::Update()` 忘了在控制周期里调用** —— 零漂永远采不到，`imu_ready` 永远 0
4. **把云台固件烧进了底盘板** —— 轮子必然不动，因为云台固件里没有轮电机代码。
   **每次烧录前先读 UID + 比对 Reset 入口**
5. **状态机死锁**：`Dead→Resurrection` 原本也要求 `motors_ready`，
   而“超时兜底”只在 Resurrection 里生效 → 一颗电机坏了就永远停在 Dead。
   修法：把「进自检」和「进工作」的前提拆开（`prerequisitesOk` / `allReady`）
6. **超时兜底依赖状态变量**：修完上面那条后，超时判定写成 `life_state_==Resurrection`，
   结果进 Working 的那一拍 `ready` 又变 false → 被打回 Dead → 每 2 s 循环一次。
   修法：改成与状态无关的「上电计时」
7. **DBUS 空帧被当成满杆**：接收机掉线时缓冲区全 0，解码成 −1.55，
   死区钳位后变成满量程指令 → 云台自己匀速转。修法：校验帧内容（非全 0/FF + 拨杆 1~3）
8. **轮速环没有反馈看门狗**：反馈断了 `vel()` 恒 0，积分涨到限幅，
   电调一恢复就猛冲。修法：`IsFeedbackFresh()` + 清积分 + 输出 0
9. **`kp` 太大 + 转矩限幅太小** = 极限环震荡，看着像程序 bug，其实是增益取舍
10. **Ozone 里写死的 J-Link 序列号** —— 换机器必然 `Connection failed`，改 `SetHostIF` 那行
