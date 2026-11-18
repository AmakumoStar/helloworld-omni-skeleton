# tools/ 调试脚本说明

> 这些是**上板调试与数据分析**用的脚本，全部入库、各自带 docstring。
> 它们不参与固件编译，删改不会影响两板构建，但 `docs/` 里的引用要同步改。

## 一览

| 文件 | 作用 | 什么时候用 |
|---|---|---|
| **jlink.ps1** | J-Link 操作封装：读 UID、判活（两次独立会话读 `uwTick`）、导出 DBUS 抓包、导出小陀螺录制、烧录（带 UID + Reset 入口校验） | **每次上板都用** |
| **spin_trace.py** | 解析固件内置的 100 Hz×7 s 小陀螺录制（21 列），直接给 4 条判据：指令是否衰减 / 轮子是否跟住 / 输出是否饱和 / 是否有反馈跳变 | 小陀螺“转不匀、走不直”时 |
| **spin_direction.py** | 在上面的数据上拆分方向误差来源：坐标系角（δ/ω 噪声被 `g_frame_lead` 放大）/ 轮速跟不上 / 9 Hz 纹波 | 专门查“小陀螺平移走不直” |
| **wheel_step.py** | 轮速环阶跃判读：t10~t90、超调、稳定时间、稳态误差、输出饱和度、振荡次数 | 调轮速环 kp/ki/ff 时；架空拨轮做阶跃 |
| **dbus_capture.py** | 离线解析 DBUS 原始抓包：帧长、间隔、帧头、各通道，判断解码对不对 | 遥控链路出问题时 |
| **make_fake_capture.py** | 生成**假**的 DBUS dump，验证 `dbus_capture.py` 的解析自洽 | 改了解析脚本、手上没硬件时 |

## jlink.ps1 的 Action

| Action | 用途 |
|---|---|
| `uid` | 读 UID + 向量表（确认是哪块板、烧的是不是这份 ELF） |
| `alive` | 两次独立会话读 `uwTick`，判断程序是否在跑（避免“halt 住读冻结值”） |
| `flash` | 烧录；**必须给 `-ExpectedUid`**，烧完校验 Reset 向量 == ELF 入口 |
| `dump-dbus` | 导出 `g_dbus_cap`（固件需先打开 `kDbusCapture`） |
| `dump-trace` | 导出小陀螺录制 `g_trace_buf`，并清 `g_trace_state/n` 以便重录 |
| `bench` | 台架点动（`g_bench_enable` 魔数开关，上电默认关） |
| `zero-check` | 归零差分验收 |
| `wdg-demo` | 看门狗复位演示 |

```powershell
$repo = 'C:\Users\YTN\Desktop\worka\final\omni_skeleton'
pwsh -File "$repo\tools\jlink.ps1" -Action uid   -Board gimbal -Speed 200
pwsh -File "$repo\tools\jlink.ps1" -Action flash -Board gimbal -ExpectedUid 0024003D -Speed 200
pwsh -File "$repo\tools\jlink.ps1" -Action dump-trace -Board chassis -Speed 200
```

## 数据流

```
固件内置录制 (g_trace_buf，100 Hz × 7 s，只写 RAM)
        │  jlink.ps1 -Action dump-trace
        ▼
   spin_trace.bin ──► spin_trace.py     （自旋 4 条判据）
                  ├► spin_direction.py  （方向误差拆分）
                  └► wheel_step.py      （轮速环阶跃指标）
```

DBUS 链路则是：

```
comm_task.cpp (kDbusCapture=true) → g_dbus_cap → jlink.ps1 -Action dump-dbus → dbus_capture.py
```

## 使用注意（都踩过）

- **SWD 一律 `-Speed 200`**；1 MHz 常失败。用 J-Link 前先让操作手断开 Ozone。
- J-Link 会话结尾必须发 `g`，否则 CPU 停在复位后不跑（“烧完就不动”多半是这个）。
- `connect` 会 halt CPU，读到的 RAM 是冻结值；判“在跑”要用两次独立会话读 `uwTick`。
- 符号地址每次编译都变：先 `arm-none-eabi-nm -C -S <elf>`，正则用 `名称$` 并排除 `guard`。
- 台架读台账要在**同一会话里先读后清零**（CPU 在跑，先清会把轮子清掉读到 0）。
- 录制里 `tgt` 是轮子线速度 m/s、`vel` 是电机输出端 rad/s，不换算会算出 85361% 这种假数。

## 与固件开关的对应

| 脚本 | 需要的固件开关 |
|---|---|
| `dump-trace` / `spin_*.py` / `wheel_step.py` | `g_trace_*`（已内置） |
| `dump-dbus` / `dbus_capture.py` | `Chassis/Task/component/comm_task.cpp` 的 `kDbusCapture = true`（**用完改回 false**） |
| `bench` | `g_bench_enable = 0x5A5A5A5A`（运行期写，上电默认关） |
