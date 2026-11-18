# omni_skeleton — 双板全向轮步兵

RoboMaster 电控考核项目的完整实现：**底盘板 + 云台板**双 MCU 架构，
STM32F407IGH6 ×2、C++20、1 ms 控制周期。

| 项 | 值 |
|---|---|
| 仓库 | `github.com/AmakumoStar/helloworld-omni-skeleton` |
| 当前提交 | `47dce8f`（本地领先 origin 4 个提交，push 由操作手执行） |
| 编译 | CMake + Ninja + `arm-none-eabi-g++` |
| 主机自测 | 328 项 |
| 阶段 | **两板实车闭环已打通**：三模式可用、云台两轴整定完成、小陀螺自旋+平移已验证 |

---

## 文档导航

| 文档 | 什么时候看 |
|---|---|
| **[bringup.md](bringup.md)** | 拿到硬件、上电、调试、验收 —— 按阶段走的操作手册 |
| **[known_issues.md](known_issues.md)** | 出问题先查这里：每条都有现象、判据、真因、修法（§13/§14 是 2026-11-18 的 yaw 整定） |
| **[parameters.md](parameters.md)** | 当前生效的参数、仍未实测的项、调试开关清单 |
| **[architecture.md](architecture.md)** | 代码结构、设计取舍 |
| **[protocol.md](protocol.md)** | 改双板通信、对单位/坐标系有疑问 |
| **[gimbal.md](gimbal.md)** | 云台环的细节（绝对角、限位、前馈） |
| **[dbus_capture.md](dbus_capture.md)** | 需要重新抓 DBUS 原始流时 |
| **[tools.md](tools.md)** | `tools/` 各脚本（jlink.ps1、spin_trace.py、wheel_step.py…）的用途与数据流 |
| **[history/handoff_2026-11-17.md](history/handoff_2026-11-17.md)** | 历史交接（内容已并入 known_issues，仅存档） |

---

## 架构速览

```
        遥控器(DR16)
             │ DBUS
     ┌───────▼────────┐        CAN1         ┌────────────────┐
     │   底盘板        │◄──── 0x210/0x211 ──►│    云台板       │
     │  Chassis       │      8 字节双板帧    │   Gimbal       │
     └───┬────────┬───┘                     └───────┬────────┘
         │CAN1/2  │                                  │ CAN1
         ▼        ▼                                  ▼
    M3508 ×4   （BMI088）                     DM-J4310 ×2
```

| 板 | 职责 | 关键点 |
|---|---|---|
| **底盘板** | 麦轮逆解、三模式状态机、轮速环、双板协议发起 | 平移指令在**云台系**，解算前按 δ + 超前补偿旋转到车体系 |
| **云台板** | yaw/pitch 位置环、绝对指向保持 | yaw 闭环在 **IMU 绝对角**上，不是关节角 |

---

## 快速开始

```powershell
$repo = 'C:\Users\YTN\Desktop\worka\final\omni_skeleton'

# 编译（只能操作手执行；agent 沙盒里 cmake 会挂死）
cmake --build "$repo\Chassis\build\w"
cmake --build "$repo\Gimbal\build\w"

# 烧录（必须给 ExpectedUid，防止烧串；SWD 不稳就加 -Speed 200）
pwsh -File "$repo\tools\jlink.ps1" -Action flash -Board chassis -ExpectedUid 00380028 -Speed 200
pwsh -File "$repo\tools\jlink.ps1" -Action flash -Board gimbal  -ExpectedUid 0024003D -Speed 200

# 看板上跑的是什么、活着没有
pwsh -File "$repo\tools\jlink.ps1" -Action uid   -Board gimbal -Speed 200
pwsh -File "$repo\tools\jlink.ps1" -Action alive -Board chassis -Speed 200
```

板子：底盘 `00380028`、云台 `0024003D`。详细流程见 [bringup.md](bringup.md)。

---

## 当前状态（2026-11-18）

| 功能 | 状态 | 关键参数 / 备注 |
|---|---|---|
| 状态机 + 失效链 | ✅ 实车验过 | 急停=左上拨杆上推；云台板先判 `local_rc_online` 再读本地拨杆 |
| 双板 CAN | ✅ 实车验过 | 0x210/0x211 双向，拔任一端 100 ms 进 Dead |
| 遥控 DBUS | ✅ 实车验过 | 18 字节、无帧头、71 帧/s，`ReceiveToIdle` 切块 |
| 云台 yaw | ✅ 整定完成 | **kp 3.5 / ki 0.02 / kd 0.35**；D 用 IMU `yawRate()`（已修符号）；0.5 rad 斜坡超调 0.8° |
| 云台 pitch | ✅ 整定完成 | kp 10 / kd 0.25 / ki 0；重力前馈 0.60 N·m；限位 −0.008 / +0.583 |
| 底盘四轮 | ✅ 实车验过 | 轮速环 400/300/ff 700（随速度衰减）；方向已标定 |
| 三种模式 | ✅ 实车验过 | 分离 / 跟随 / 小陀螺；小陀螺平移随枪口 |
| 小陀螺自旋+平移 | ✅ 方向正确 | `g_frame_lead` 在线试到 0.10（代码默认仍 0.19，待定稿） |
| **1.71 倍比例** | ❌ **最大隐患** | 自旋/平移实际速度约为标称 1.7 倍；先量轮径/轴距/减速比，别直接改 R（known_issues 7.5） |
| 轮速 ~9 Hz 纹波 | ⏳ 未根治 | 需 1 kHz 录制定性 |
| `0x201` 帧数 2 倍 | ⏳ 待核对 | 查 C620 拨码/接线 |
| 看门狗/归零验收 | ⏳ 待做 | `jlink.ps1 -Action wdg-demo / zero-check` |

---

## 安全边界（改代码前必读）

| 项 | 值 | 说明 |
|---|---|---|
| yaw `max_torque` | **1.0 N·m** | 11-16 飞转事故后定死，不要动 |
| yaw `ki` | **0.02** | 2026-11-18 操作手批准从 0 放宽；积分限幅 ±0.15、|err|>34° 清积分、条件积分三条保护不变 |
| pitch `max_torque_pitch` | **2.0 N·m** | 操作手批准的放宽（重力轴），yaw 仍 1.0 |
| `g_ignore_delta / g_force_mode / g_estop_force` | **0** | 正式使用前必须为 0；运行期变量断电即丢 |
| 参数改动流程 | 静置 30 s 看输出 → 再谈响应 | 落地测试：架空验方向 → 落地低速 → 手不离急停 |

---

## 三条硬约束（改代码前必读）

1. **`RobotComponents/` 被两块板同时编译** → 只能放纯类型与内联函数，不许 include HAL
2. **平移指令永远定义在云台系** → 解算前必须按 δ 旋转，且只转一次
3. **喂狗只在 `CommTask()` 一处** → 放中断回调里会掩盖主循环卡死，验收直接判不合格

---

## 常用调试入口

```powershell
# 符号地址每次编译都会变，先 nm；正则用「名称$」并排除 guard
arm-none-eabi-nm -C -S Gimbal\build\omni_gimbal.elf | Select-String 'g_yaw_kd$'

# 结构体成员偏移（加字段后必须重查，否则会把 yaw 当 pitch）
arm-none-eabi-gdb -batch -ex "ptype /o 'robot::Gimbal'" Gimbal\build\omni_gimbal.elf

# 小陀螺录制（跑的时候 SWD 可以拔掉）
pwsh tools\jlink.ps1 -Action dump-trace -Board chassis -Speed 200
python tools\spin_trace.py --dump spin_trace.bin --plot spin_trace.png
```

⚠️ 用 J-Link 前先让操作手断开 Ozone（FPB 断点 + 复位即 halt 会让板子表现为"烧完不动"）；
J-Link 会话结尾必须发 `g`，否则 CPU 停在复位后不跑。
