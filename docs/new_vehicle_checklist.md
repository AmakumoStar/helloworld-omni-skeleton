# 拿到一辆新车之后的操作手册

> 这份手册假设你已经有一份能编译通过的固件（`omni_skeleton`），现在换了一台**新的车**。
> 核心原则：**先认车（采集信息）→ 再通通讯 → 再单机验证 → 最后整车**。
> 每一步都有明确判据，不满足不进下一步。

---

## 阶段 0：准备（不上电，约 30 分钟）

### 0.1 工具

| 工具 | 用途 |
|---|---|
| J-Link（SWD 3 根线即可） | 烧录 / 读内存 / 嗅探 |
| Ozone | 实时看变量、调 PID（用 J-Link） |
| 遥控器 + 接收机 | 必须有，没有就永远进不了 Working |
| 万用表 | 量供电、量 CAN 通断 |
| 垫块 / 支架 | 四轮离地 |
| CAN 分析仪（有更好） | 没有就用固件内置的嗅探（见 2.3） |

### 0.2 代码状态确认

```powershell
cd C:\Users\YTN\Desktop\worka\final\omni_skeleton
cmake -S host_tests -B host_tests/build -G Ninja -DCMAKE_CXX_COMPILER=D:/mingw64/bin/g++.exe
cmake --build host_tests/build
.\host_tests\build\host_tests.exe
```

**判据**：结尾必须是 `检查项 143 个，失败 0 个 / 结果：PASS`。
不过就别上车，先解决算法层的问题。

### 0.3 确认当前是"保守值"

| 文件 | 常量 | 上车时应为 |
|---|---|---|
| `Chassis/RobotModules/component/chassis.hpp` | `max_vx/max_vy/max_w/max_spin_w` | 0.6 / 0.6 / 2.0 / 2.0 |
| `Chassis/Task/component/main_task.cpp` | `kMaxWheelVelMps` | 1.0 |
| `Gimbal/Task/component/main_task.cpp` | `lim.max_torque` | 1.0 |
| `Gimbal/RobotModules/component/robot.cpp` | `kAllowSingleJoint` | `false` |

> 新车第一次调试**必须**用这套保守值。等方向全验对了再按 `docs/code_review.md` 第 4 节恢复。

### 0.4 车架起来

四轮离地、云台无干涉、急停拨杆放手边。

---

## 阶段 1：认车（只采集信息，不改控制逻辑）

这一步决定了后面所有"为什么不动"的排查速度。**把结果填进 1.5 的表格**。

### 1.1 确定 CAN 拓扑（最关键）

要搞清楚三件事：

1. 两块板之间用哪一路 CAN 连（底盘板 CAN1/CAN2 ↔ 云台板 CAN1/CAN2）
2. 底盘 4 个 M3508 电调挂在哪一路
3. 云台 2 个达妙电机挂在哪一路

**方法**：用固件内置的嗅探（`Gimbal/src/HW_can.cpp` 里的 `g_rx_*` 表）。

先烧云台板，让它跑 3 秒，然后读这几张表：

```powershell
$root='C:\Users\YTN\Desktop\worka\final\omni_skeleton'
$jlink='C:\Program Files\SEGGER\JLink_V978\JLink.exe'
$s="$env:TEMP\sn.jlink"
# 地址用 nm 查：arm-none-eabi-nm -C Gimbal\build\omni_gimbal.elf | findstr g_rx_
Set-Content $s -Encoding ASCII -Value @('si SWD','speed 100','connect','h','mem32 0x20000714 2','mem32 0x2000079C 2','mem32 0x2000071C 8','mem32 0x2000075C 8','g','q')
& $jlink -device STM32F407IG -CommanderScript $s
Remove-Item $s -Force -ErrorAction SilentlyContinue
```

**怎么读**：

| 看到什么 ID | 说明 |
|---|---|
| `0x11` / `0x12` | 达妙 yaw(2)/pitch(1) 的反馈 |
| `0x201`~`0x204` | M3508 反馈（= 0x200 + id） |
| `0x200` / `0x1FF` | 底盘板发出的电机控制帧 |
| `0x210` / `0x211` | 双板通信帧 |

> 我们遇到过的真实情况：CAN1 上只有 `0x12` 和 `0x200`，从没见过 `0x201~0x204`
> → 结论：**电调不在 CAN1**，轮子因此不动。后来把轮帧改成双路广播才解决。

### 1.2 电机 ID

| 电机 | 查什么 | 我们这台的值 |
|---|---|---|
| 底盘 4 轮 | C620 电调拨码，反馈 ID = 0x200 + id | id = 1,2,3,4（LF/LB/RB/RF） |
| 云台 yaw | 达妙上位机读 CAN ID | **2** |
| 云台 pitch | 同上 | **1**（注意和 yaw 是**反的**） |

达妙还要确认上位机参数：`CAN ID = id`、`Master ID = id + 0x10`、`PMAX = 3.141593`、`VMAX = 21`、`TMAX = 7.5`。

### 1.3 几何参数（拿尺子量）

- 轮半径 `R`（麦轮到地面距离）
- 半轴距 `W/2`（前后轮到中心）
- 半轮距 `L/2`（左右轮到中心）

填进 `Chassis/Task/component/main_task.cpp` 的 `kWheelRadiusM / kHalfWheelbaseM / kHalfTrackM`。

> 我们的值：R = 0.07786 m，半轴距 = 半轮距 = 0.15338 m（正方形底盘）。

### 1.4 云台限位

手动把云台推到上下机械极限，读 `yaw`/`pitch` 电机的 `angle()`，各留 5° 余量填进
`Gimbal/Task/component/main_task.cpp` 的 `lim.pitch_min / lim.pitch_max`。

### 1.5 记录表（填完再往下走）

```
CAN 拓扑：底盘板电机=____路，双板=____路；云台板电机=____路，双板=____路
电机 ID：LF=__ LB=__ RB=__ RF=__  yaw=__  pitch=__
几何：  R=______  半轴距=______  半轮距=______
云台限位：pitch ∈ [____, ____] rad
减速比：M3508 = 19.2（出厂）还是 14（需确认）
```

---

## 阶段 2：通讯打通

### 2.1 SWD 连接（三线接法）

不用的 20pin 排线，直接三根杜邦线：

```
J-Link 7 脚 (SWDIO) → 板子 SWDIO
J-Link 9 脚 (SWCLK) → 板子 SWCLK
J-Link 4 脚 (GND)   → 板子 GND
```

验证：

```powershell
$jlink='C:\Program Files\SEGGER\JLink_V978\JLink.exe'
$s="$env:TEMP\p.jlink"
Set-Content $s -Encoding ASCII -Value @('si SWD','speed 1000','connect','q')
& $jlink -device STM32F407IG -CommanderScript $s
Remove-Item $s -Force -ErrorAction SilentlyContinue
```

| 输出 | 含义 |
|---|---|
| `Cortex-M4 identified.` | ✅ 可以烧了 |
| `VTref=3.2xx V` 但 `Failed to initialize DAP` | 有电但 SWDIO/SWCLK 没通 → 查线、查排线错位 |
| `VTref=0.000V` | 板子没供电 |

### 2.2 烧录（自动识别板子，不会烧错）

```powershell
$root='C:\Users\YTN\Desktop\worka\final\omni_skeleton'
$jlink='C:\Program Files\SEGGER\JLink_V978\JLink.exe'
$chk="$env:TEMP\chk.jlink"; $f="$env:TEMP\f.jlink"

cmake --build "$root\Chassis\build\w"
cmake --build "$root\Gimbal\build\w"

Set-Content $chk -Encoding ASCII -Value @('si SWD','speed 1000','connect','mem32 0x1FFF7A10 1','q')
$out = & $jlink -device STM32F407IG -CommanderScript $chk 2>&1 | Out-String
Remove-Item $chk -Force -ErrorAction SilentlyContinue
if($out -notmatch 'Cortex-M4 identified'){ throw '连不上板子' }

if($out -match '00400020'){ $name='底盘板'; $elf="$root\Chassis\build\omni_chassis.elf" }
else                      { $name='云台板'; $elf="$root\Gimbal\build\omni_gimbal.elf" }
Write-Host "检测到：$name" -ForegroundColor Green

Set-Content $f -Encoding ASCII -Value @("r","h",('loadfile "{0}"' -f $elf),"r","g","qc")
& $jlink -device STM32F407IG -if SWD -speed 1000 -autoconnect 1 -ExitOnError 1 -CommanderScript $f
```

芯片 UID 可以用来区分板子（`00400020...` = 底盘板，`0054004D...` = 云台板）。
**新车的两块板 UID 会不同，需要重新记录**：读一次 `mem32 0x1FFF7A10 3` 存下来。

**判据**：输出里有 `O.K.`。

### 2.3 CAN 嗅探（判断电机在不在、在哪一路）

见 1.1。这一步能直接回答"为什么轮子不动 / pitch 没反应"。

### 2.4 遥控器

1. 接收机插在**哪块板的 USART3** 上——这一步必须确认（我们这台在**底盘板**上）
2. 遥控器开机、对频
3. 读 `rc_online`（见 3.1 的读法）应该为 1

> 如果接收机在底盘板而云台板读不到，代码里已经做了**摇杆值转发**（`yaw_des/pitch_des`），
> 不用改接线。

---

## 阶段 3：单机验证（四轮离地）

### 3.1 读状态的最快方式

不用 Ozone 也能读。用 DWARF 算偏移，再用 J-Link 读内存：

```powershell
$root='C:\Users\YTN\Desktop\worka\final\omni_skeleton'
$elf="$root\Chassis\build\omni_chassis.elf"     # 或 omni_gimbal.elf
$syms=& arm-none-eabi-nm -C $elf 2>$null
$l = $syms | Where-Object { $_ -match "^[0-9A-Fa-f]{8}\s+\S\s+GetRobot\(\)::robot\s*$" } | Select-Object -First 1
$base=[Convert]::ToUInt32((($l -split '\s+')[0]),16)

$info=& arm-none-eabi-readelf --debug-dump=info $elf 2>$null | Out-String
$m=[regex]::Match($info,"DW_AT_name\s*:\s*$([regex]::Escape('life_state_'))\s*\r?\n(?:.*\r?\n){0,8}?.*DW_AT_data_member_location:\s*(\d+)")
$off=[int]$m.Groups[1].Value

$jlink='C:\Program Files\SEGGER\JLink_V978\JLink.exe'
$s="$env:TEMP\st.jlink"
Set-Content $s -Encoding ASCII -Value @('si SWD','speed 100','connect','h',("mem8 0x{0:X} 8" -f ($base+$off)),'g','q')
& $jlink -device STM32F407IG -CommanderScript $s
Remove-Item $s -Force -ErrorAction SilentlyContinue
```

读回的 8 个字节依次是：

```
[life_state_] [mode_] [imu_ready] [motors_ready] [rc_online] [peer_online] [estop] [--]
```

**判据**：`life_state_ = 2` 且前四项全为 1，才说明这块板就绪。

### 3.2 逐项点亮

| 顺序 | 验什么 | 判据 | 不通过查什么 |
|---|---|---|---|
| 1 | 板子活着 | Ozone 连 10 秒不断 | 喂狗没生效 / TIM6 中断没进 |
| 2 | IMU | `imu_ready = 1`（约 1.5 s 后） | SPI 接线、BMI088 片选（`ins_imu.cpp`） |
| 3 | CAN 反馈 | 手转轮子，`angle_` 连续变化 | 电调供电、CAN 归属（1.1）、`HAL_CAN_Start` |
| 4 | 遥控 | 摇杆拉满，`rc_lv` 到 ±1 | 接收机插哪块板、波特率 100k/9bit 偶校验 |
| 5 | 电机使能 | 达妙 `is_enabled = 1`、`status_code = 1` | 电机供电、ID、上位机参数 |
| 6 | 单轴转动 | 推摇杆，云台平滑转、松手停 | PID 参数（见 6） |
| 7 | 轮子转向 | 前推 → 四轮同向、车轮底部朝后 | `ins_motor.cpp` 的 `kWheelDir` |

### 3.3 方向标定速查

| 症状 | 改哪里 |
|---|---|
| 前推，车往后退 | `Chassis/Instance/component/ins_motor.cpp` → `kWheelDir` 全改 `kDirRev` |
| 右推，车往左走 | `Chassis/RobotModules/component/robot.cpp` → `vy_cmd` 负号 |
| 右推，车逆时针转 | 同上 → `w_stick` 负号 |
| 转动云台后前推，方向不对 | `module_state.hpp` → `kFrameDeltaSign` 改 −1 |
| 拨轮方向反 | `chassis_ik_solver.hpp` → `kSpinSign` 改 −1 |
| 右摇杆右推，云台往左转 | `Gimbal/RobotModules/component/robot.cpp` → yaw 步长负号 |
| 右摇杆上推，pitch 反而下压 | 同上 → pitch 步长符号 |

---

## 阶段 4：双板联调

| 验什么 | 判据 | 不通过 |
|---|---|---|
| 两块板都上电、CAN 接好 | 两边 `peer_online = 1` | 双板帧的 CAN 通道选错（`kCanComm`） |
| 帧在流动 | `comm->rx_count_` 持续增长 | 同上；或对方板没跑 |
| 拔任一板 CAN | 约 100 ms 后**两板都** `peer_online = 0`、电机归零 | 离线阈值/路由 |
| 双板互保 | 底盘报急停 → 云台也 `estop = 1` | `Gimbal/robot.cpp` 里的互保逻辑 |

---

## 阶段 5：三模式验收动作（轮子可以着地）

| 模式 | 右拨杆 | 动作 | 预期 |
|---|---|---|---|
| 分离 | Up | 推左摇杆 / 推右摇杆 | 平移 / 原地转，松手停 |
| 跟随 | Mid | **转动云台** | 车头自动追着云台转 |
| 小陀螺 | Down | 拨轮给自旋 + 推左摇杆前 | 车在转，但**前进方向始终朝云台** |
| **核心验收** | 任意 | **转云台 90°，再推左摇杆前** | 车朝云台指的方向走，不是朝车头 |

---

## 阶段 6：失效演示（硬性验收项）

| 动作 | 预期 | 怎么看 |
|---|---|---|
| 打急停（左拨杆 Down） | ≤1 ms 内所有电机指令归零 | 读电机 `raw_input_` 或听声音 |
| 松急停 | 重新走 Dead→Resurrection→Working（几十 ms） | `life_state_` 跳变 |
| 关遥控器 | 约 100 ms 后归零、进 Dead | `rc_online` 变 0 |
| 重开遥控器 | 自动恢复 | 同上 |
| **屏蔽喂狗** | 底盘约 300 ms、云台约 600 ms 复位 | 用 Ozone 在 `MainTask()` 下断点让 CPU 停住即可，不用改代码 |

> ⚠️ 喂狗只能在 `CommTask()` 一处。放到中断回调里会掩盖主循环卡死，验收判"谎称已实现"。

---

## 阶段 7：整定与恢复正式值

1. **PID 整定**（Ozone Timeline 画"目标/实际/输出"）
   - 云台位置环：先 P 后 D，`kp` 从 2 开始加，`kd` 保证不抖
   - 轮速环：`kp=200, ki=20` 起步
2. **恢复限幅**（见 `docs/code_review.md` 第 4 节）
3. **恢复 `kAllowSingleJoint = false`**
4. 更新 `docs/units_and_protocol.md` 里的参数表
5. 跑一遍完整的主机自测

---

## 附录 A：常用 J-Link 命令

```powershell
# 连接测试
$s="$env:TEMP\p.jlink"; Set-Content $s -Encoding ASCII -Value @('si SWD','speed 1000','connect','q')
& 'C:\Program Files\SEGGER\JLink_V978\JLink.exe' -device STM32F407IG -CommanderScript $s

# 读芯片 UID（区分板子）
#   mem32 0x1FFF7A10 3

# 读复位原因（判断是不是看门狗复位）
#   mem32 0x40023874 1     bit29=IWDG, bit28=软件复位, bit27=POR, bit26=PIN, bit25=BOR

# 读故障寄存器（判断有没有 HardFault/UsageFault）
#   mem32 0xE000ED28 1     CFSR
#   mem32 0xE000ED2C 1     HFSR

# 读向量表（确认烧的是不是新固件）
#   mem32 0x08000000 2     第一个字=初始SP，第二个=Reset入口
```

## 附录 B：符号地址怎么查

```powershell
# 全局/静态变量地址
arm-none-eabi-nm -C Gimbal\build\omni_gimbal.elf | findstr "GetRobot g_rx_"

# 类成员偏移（DWARF）
arm-none-eabi-readelf --debug-dump=info Gimbal\build\omni_gimbal.elf | findstr /N "life_state_" 
# 再往上看几行找 DW_AT_data_member_location
```

**注意**：每次重新编译，`.bss` 里的地址都可能变，**每次都要重新查**，不要用旧地址。

## 附录 C：故障对照表

| 症状 | 最可能原因 | 一句话验证 |
|---|---|---|
| 烧不进去，`Missing command line parameter` | 命令被换行截断 / 变量名打错 | 先 `Get-Content $f` 看 6 行在不在 |
| `VTref=3.3V` 但 `Failed to initialize DAP` | SWDIO/SWCLK 没通（错位/松） | 换 3 根杜邦线 |
| 板子每 0.3/0.6 s 复位 | 喂狗没生效 | 读 `RCC_CSR` bit29 |
| `life_state_` 永远 0 | 看 `inputs_` 哪个是 0 | 读 8 字节状态 |
| `imu_ready` 恒 0 | 片选引脚错 / SPI 没通 | 转车看 `euler_[2]` 变不变 |
| `peer_online` 恒 0 | 双板帧发错 CAN | 嗅探看有没有 `0x210/0x211` |
| 轮子不动 | 电调不在那路 CAN / 没供电 | 嗅探有没有 `0x201~0x204` |
| 云台抖 | kp 太大被限幅打成极限环 | 降 kp、加 kd |
| 松手车缓慢爬 | 没有摇杆死区 | 看 `ApplyStickDeadzone` 是否生效 |
| pitch 零反馈 | 电机没供电/没接/ID 错 | 看 `rotor_temp` 是否为 0 |

## 附录 D：这台车踩过的坑（让新人少走弯路）

1. **骨架没有调用 `HAL_CAN_Start()`** —— 滤波器和回调都配好了，但 CAN 没启动，一帧收不到。
2. **`tick.cpp` 的 `USE_HAL_TICK` 宏没定义** —— 整个文件被预处理掉，链接报一堆 undefined reference。
3. **`Imu::Update()` 忘了在控制周期里调用** —— 零漂永远采不到，`imu_ready` 永远 0。
4. **两块板共用同一路 CAN**，但双板帧发在另一路 —— `peer_online` 恒 0。
5. **遥控接收机接在底盘板**，云台板读不到 —— 需要转发摇杆值。
6. **`kp` 太大 + 转矩限幅太小** = 极限环震荡，看着像"程序有问题"，其实是增益取舍。
7. **`.jdebug` 里写死了别人 J-Link 的序列号** —— Ozone 一直 `Connection failed`，改 `SetHostIF` 那行。
8. **`comm_seq_` 是 uint8** —— 别拿它当"运行毫秒数"看，它会 256 回绕。
