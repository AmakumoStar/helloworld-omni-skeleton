# omni_skeleton — 双板全向轮步兵

RoboMaster 电控考核项目的完整实现：**底盘板 + 云台板**双 MCU 架构，
STM32F407IGH6 ×2、C++20、1 ms 控制周期。

| 项 | 值 |
|---|---|
| 仓库 | `github.com/AmakumoStar/helloworld-omni-skeleton` |
| 编译 | CMake + Ninja + `arm-none-eabi-g++` (15.3.1) |
| 主机自测 | **328 项全过** |
| 两板语法检查 | **51 文件，0 错 0 警告** |
| 当前阶段 | 云台绝对角闭环已验证；底盘电机待通电 |

---

## 文档导航

| 文档 | 什么时候看 |
|---|---|
| **[bringup.md](bringup.md)** | 拿到硬件、上电、调试、验收 —— 按阶段走的操作手册 |
| **[architecture.md](architecture.md)** | 想理解代码结构、设计取舍、踩过哪些坑 |
| **[protocol.md](protocol.md)** | 改双板通信、对单位/坐标系有疑问 |
| **[parameters.md](parameters.md)** | 要实测标定参数（限位、方向、PID、限幅） |

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
| **底盘板** | 麦轮逆解、三模式状态机、轮速环、双板协议发起 | 平移指令在**云台系**，解算前按 δ 旋转到车体系 |
| **云台板** | yaw/pitch 位置环、绝对指向保持 | yaw 闭环在 **IMU 绝对角**上，不是关节角 |

---

## 快速开始

```powershell
# 仓库根目录
$repo = '<你的仓库路径>'

# 1) 主机自测（不需要硬件）
cmake -S "$repo\host_tests" -B "$repo\host_tests\build" -G Ninja -DCMAKE_CXX_COMPILER=D:/mingw64/bin/g++.exe
cmake --build "$repo\host_tests\build"
& "$repo\host_tests\build\host_tests.exe"

# 2) 两板固件
cmake --build "$repo\Chassis\build\w"
cmake --build "$repo\Gimbal\build\w"
```

烧录见 [bringup.md](bringup.md) 第 4 节。

---

## 三条硬约束（改代码前必读）

1. **`RobotComponents/` 被两块板同时编译** → 只能放纯类型与内联函数，不许 include HAL
2. **平移指令永远定义在云台系** → 解算前必须按 δ 旋转，且只转一次
3. **喂狗只在 `CommTask()` 一处** → 放中断回调里会掩盖主循环卡死，验收直接判不合格

---

## 已知状态（2026-11 中）

| 功能 | 状态 |
|---|---|
| 状态机（Dead/Resurrection/Working）+ 失效链 | ✅ 代码完成，实车验过 |
| 双板 CAN 通信 | ✅ 实车验过（`rx_count` 持续增长） |
| 遥控链路 | ⚠️ 接收机当前无数据，待排查硬件 |
| 云台 yaw 绝对角闭环 | ✅ 实车验过（手推回位、摇杆驱动） |
| 底盘轮电机 | ❌ 电调无任何反馈，轮子未转 |
| pitch 电机 | ❌ 无反馈（可能 ID 配置问题） |
| 三模式实际动作 | ❌ 依赖底盘能动，尚未验证 |
