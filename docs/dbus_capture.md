# DBUS 原始流抓取 —— 一页纸操作手册

> 目的：一次拿到三个数（**帧长 / 帧头 / 帧间隔**），把「DBUS 解码失败」从猜变成算。
> 背景与已排除的假设见 [known_issues.md](known_issues.md) 第 1 节。

---

## 0. 为什么不能再用「遥控有没有反应」当判据

11-16 试过的两个方案（ReceiveToIdle 流式 / 上一帧+本帧滑动搜索）都是拿
「推杆后云台转不转」来判断有没有成功的。这个判据**不等价**：

- 解码没对上 → 遥控没反应
- 状态机卡 Dead / 云台 CAN 没通 / 电调没使能 → 遥控同样没反应

同一个现象有两个原因，就没法用它定位。而且当时还叠加了「J-Link halt 导致读到全零」
的假象，于是把可能有用的代码回退了。

这次改成：**只采集，判据是字节本身**。

---

## 1. 这次改了什么（只动底盘板，云台板不用重编）

| 文件 | 改动 | 影响 |
|---|---|---|
| `Chassis/Task/component/comm_task.cpp` | 新增 `kDbusCapture`：USART3 改用 `ReceiveToIdle` 按空闲切块，每块记 `tick + len + 前 64 字节`，存 `g_dbus_cap`（128 条） | 抓包期间 `g_dbus_ev/ok` 恒 0（那条定长路径没跑），**解码逻辑一个字没改** |
| `Chassis/RobotModules/component/robot.cpp` | 台架模式改成运行期开关 `g_bench_enable`（写 `0x5A5A5A5A` 才开，上电默认关）；**修掉台架分支漏调 `chassis_->SetCommand()` 的 bug**；实现 `g_bench_open_duty` 开环输出 | 默认关闭时行为与改前一致；开了才能测轮子 |
| `Chassis/src/HW_can.cpp` | 新增 `g_can_tx_ok[2]` / `g_can_tx_fail[2]` | 以前 `HAL_CAN_AddTxMessage` 的返回值被丢掉，发不出去完全静默 |

> 云台板这次**不用重编**（它的 DBUS 路径没动）。

---

## 2. 编译（必须你来，agent 沙盒里 cmake 会挂死）

```powershell
cmake --build C:\Users\YTN\Desktop\worka\final\omni_skeleton\Chassis\build\w
```

编完 ELF 在 `Chassis\build\omni_chassis.elf`（注意：不是 `build\w\`，那里只有 hex/bin）。

---

## 3. 烧录（agent 可以代做，也可你自己跑）

```powershell
pwsh -File tools\jlink.ps1 -Action uid   -Board chassis        # 先看是哪块板
pwsh -File tools\jlink.ps1 -Action flash -Board chassis -ExpectedUid 00380028
```

脚本会做两件事：烧前比对 UID（防止把片子烧串），烧后比对 `Reset 向量 == ELF 入口`
（确认板上跑的就是这个 ELF）。

**烧完先验活**：

```powershell
pwsh -File tools\jlink.ps1 -Action alive -Board chassis
```

判读：`uwTick` 两次读数差值 > 300 ms = 程序在跑。
差值为 0 时**先排除「CPU 被 halt 没恢复」**（断电重上电再测一次），不要直接判卡死。

---

## 4. 抓数据（10 秒）

1. 遥控器开机、接收机接好，**手离开摇杆**（后面对齐检查需要中位 1024）
2. 上电，等 10 秒（128 条记录 ≈ 0.9 秒的帧，上电就存满了）
3. 导出：

```powershell
pwsh -File tools\jlink.ps1 -Action dump-dbus -Board chassis -OutFile dbus_cap.bin
```

脚本会校验文件头 magic `0xDB05CA90`，并打印「已存 n 条 / 总块数 / 重挂失败次数」。

**导出后板子会被复位（脚本里带了 `r; g`）。**如果它没恢复运行，断电重上电。

---

## 5. 分析

```powershell
python tools\dbus_capture.py --dump dbus_cap.bin --hexdump 64
```

输出五段：块长分布、块间隔、前 2 字节、解码候选表、结论。

**结论长什么样**：

```
==> 结论：帧长 20 字节，DT7::decode 的 data[0] 应指向连续流的第 2 字节（即每帧跳过前 2 字节）
```

或者

```
==> 结论：帧长 18 字节，DT7::decode 的 data[0] 应指向连续流的第 0 字节
```

候选表里看 `合法率`：**>= 90% 才算真的对上了**（判据是拨杆只能 1/2/3 + 四个摇杆
11 bit 原始值在 364~1684）。20% 左右那种是巧合，不要信。

---

## 6. 三种结果，三种动作

| 结论 | 说明 | 下一步 |
|---|---|---|
| `帧长 = 18，phase = 0` | 说明帧就是标准 18 字节、无帧头。那问题不在对齐，而在**别的**（校验全 0/FF 判定、`data[5]` 的位掩码、或丢字节） | 把 `dbus_cap.bin` 发我，看 hexdump 里 `acc == 0x00/0xFF` 和拨杆位到底怎样 |
| `帧长 = 20，phase = 2` | 帧头 `0x20 0x00` + 18 字节体，与 known_issues 的推断一致 | 按 20 字节接收、跳过 2 字节；`IsDbusFrameValid` 与 `DT7::decode` 三处一起改 |
| 块长恒为 64（上限） | 没检测到空闲间隔：接收机是连续流，或波特率/接法不对 | 改用「流式接收 + 帧头搜索」，判据仍是候选表的合法率 |

> ⚠️ 无论哪种结论，`IsDbusFrameValid`、`DT7::decode`、接收长度这三处**必须一起改**。
> 只改一处 = 换一个位置继续错。

---

## 7. 轮子测试（和 DBUS 无关，可并行）

台架开关是运行期的，不用再重编译：

```
# 打开台架模式（绕过 rc/peer/imu/motors/estop 全部五个前提）—— 车必须架空！
w4 <g_bench_enable> 5A5A5A5A

# 开环占空比：绕过速度环，四轮直给同一电流（0.1 = 10% 满量程）
w4 <g_bench_open_duty> 3DCCCCCD      # 0.1f

# 关掉
w4 <g_bench_enable> 0
```

地址用 `jlink.ps1` 里的 `Get-SymbolAddress`（或 `arm-none-eabi-nm -C ... | Select-String 'g_bench_'`）查。

判读树：

| 现象 | 结论 |
|---|---|
| 开环能转 | 电调/接线没问题 → 回去查指令链（`g_bench_vx` 路径、PID） |
| 开环不转，`g_can_tx_fail` 在涨 | 发送层（邮箱满/总线错误），还没到电调 |
| 开环不转，`g_can_tx_fail == 0`，`g_rx_cnt` 有明显反馈 | 电调收到了但不执行 → 看 C620 小灯、量 24V、确认 ID |

> 11-16 的「五个前提全强制为真，轮子仍不转」是因为台架分支漏了 `SetCommand()`，
> 底盘拿到的指令恒为 0。**那次实验不能用来排除电调**。

---

## 8. 收尾（务必做）

- [ ] `kDbusCapture` 改回 `false`（或按分析结论直接改成正确的解码实现）
- [ ] `g_bench_enable` 写 0、断电重启，确认上电轮子不动
- [ ] 把 `dbus_cap.bin` 和结论补进 [known_issues.md](known_issues.md)
