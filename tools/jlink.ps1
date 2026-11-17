<#
.SYNOPSIS
    J-Link 操作封装：读 UID / 判断程序在跑 / 导出 DBUS 抓包 / 烧录（带 UID 与入口校验）

.DESCRIPTION
    为什么要有这个脚本：手敲 J-Link Commander 容易踩两个坑
      1) `connect` 会把 CPU halt 住，此时读到的 RAM 是冻结值，看起来像"固件卡死"
      2) halt 期间 IWDG 还在数（除非 DBG_IWDG_STOP=1），dump 到一半板子会被复位
    所以这里统一先写 DBGMCU_APB1_FZ bit12 冻结看门狗，并且把"判断在跑"做成
    「两次独立会话读 uwTick」而不是「同一会话里 Sleep 后再读」。

.EXAMPLE
    # 看这块板是谁（UID + 当前固件入口）
    pwsh -File tools\jlink.ps1 -Action uid -Board chassis

.EXAMPLE
    # 判断程序是否在跑（uwTick 是否增长）
    pwsh -File tools\jlink.ps1 -Action alive -Board chassis

.EXAMPLE
    # 导出 DBUS 抓包（固件需已烧入 kDbusCapture 版本），然后用 python 分析
    pwsh -File tools\jlink.ps1 -Action dump-dbus -Board chassis
    python tools\dbus_capture.py --dump dbus_cap.bin

.EXAMPLE
    # 烧录（必须显式给出 UID，防止把云台固件烧进底盘板）
    pwsh -File tools\jlink.ps1 -Action flash -Board chassis -ExpectedUid 00380028
#>
[CmdletBinding()]
param(
    [ValidateSet('uid', 'alive', 'dump-dbus', 'dump-trace', 'flash', 'bench', 'zero-check', 'wdg-demo')]
    [string]$Action = 'uid',

    [ValidateSet('chassis', 'gimbal')]
    [string]$Board = 'chassis',

    # 仓库根目录（默认由脚本位置推断：tools\ 的上一级）
    [string]$Repo = '',

    # J-Link 速度，单位 kHz。1 MHz 经常 Failed to initialize DAP；
    # 这块板 200 kHz 通常能通（见 docs/bringup.md 4.1 与交接文档"常用坑"）。
    [int]$Speed = 1000,

    # dump-dbus 的输出文件；空则写到当前目录下的 dbus_cap.bin
    [string]$OutFile = '',

    # flash 时必须给：先跑 -Action uid 看清楚是哪块板
    [string]$ExpectedUid = '',

    # bench：开环占空比（-1~1）。开环直接给电流，空载转速很高：
    #   0.005 ≈ 0.1 A（最慢，推荐先试这个）
    #   0.01  ≈ 0.2 A
    #   0.02  ≈ 0.4 A
    #   0.05  ≈ 1 A  ← 昨天用这个，轮子实测 20+ rad/s（≈1.7 m/s），太快
    [double]$Duty = 0,

    # bench：关掉台架模式并把 duty 归零
    [switch]$BenchOff
)

$ErrorActionPreference = 'Stop'

# ---------------------------------------------------------------- 环境
if ([string]::IsNullOrWhiteSpace($Repo)) {
    $Repo = Split-Path -Parent $PSScriptRoot
}
if ([string]::IsNullOrWhiteSpace($OutFile)) {
    # dump-trace 默认写 spin_trace.bin，别把自旋数据盖到 DBUS 抓包文件名上
    $leaf = if ($Action -eq 'dump-trace') { 'spin_trace.bin' } else { 'dbus_cap.bin' }
    $OutFile = Join-Path (Get-Location) $leaf
}

$JLink = 'C:\Program Files\SEGGER\JLink_V978\JLink.exe'
if (-not (Test-Path $JLink)) {
    throw "找不到 J-Link: $JLink"
}

function Resolve-Tool([string]$name, [string[]]$candidates) {
    foreach ($c in $candidates) {
        if (Test-Path $c) { return $c }
    }
    $cmd = Get-Command $name -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    return $null
}

$Nm = Resolve-Tool 'arm-none-eabi-nm' @(
    'C:\Users\YTN\Desktop\helloworld\arm-gnu-toolchain\bin\arm-none-eabi-nm.exe')
$ReadElf = Resolve-Tool 'arm-none-eabi-readelf' @(
    'C:\Users\YTN\Desktop\helloworld\arm-gnu-toolchain\bin\arm-none-eabi-readelf.exe')

$Elf = if ($Board -eq 'chassis') {
    Join-Path $Repo 'Chassis\build\omni_chassis.elf'
} else {
    Join-Path $Repo 'Gimbal\build\omni_gimbal.elf'
}
if (-not (Test-Path $Elf)) {
    throw ("找不到 ELF: {0}`n注意：ELF 在 build\ 下，build\w\ 里只有 hex/bin。请先编译。" -f $Elf)
}

# 台架/抓包相关符号在匿名命名空间里，nm -C 显示为 "(anonymous namespace)::xxx"，
# 所以用「结尾匹配」，不能用精确名（不然还会撞上 guard variable）。
function Get-SymbolAddress([string]$name) {
    if (-not $Nm) { throw '找不到 arm-none-eabi-nm，无法查符号' }
    $pattern = [regex]::Escape($name) + '$'
    $line = & $Nm -C $Elf | Where-Object { $_ -match $pattern } | Select-Object -First 1
    if (-not $line) { throw "ELF 里没有符号 $name（是不是没编译进去？）" }
    return ('0x' + ($line -split '\s+')[0])
}

function Invoke-JLink([string[]]$lines, [string]$tag = 'cmd') {
    $f = Join-Path $env:TEMP ("jlink_{0}_{1}.jlink" -f $tag, [guid]::NewGuid().ToString('N').Substring(0, 6))
    Set-Content -Path $f -Encoding ASCII -Value $lines
    Write-Verbose ("JLink script ({0}):`n{1}" -f $f, ($lines -join "`n"))
    # 速度只在命令行给一次：各 Action 的脚本里不再写 'speed'，否则会盖掉 -Speed 200
    $out = & $JLink -device STM32F407IG -if SWD -speed $Speed -autoconnect 1 `
        -ExitOnError 1 -CommanderScript $f 2>&1 | Out-String
    Remove-Item $f -ErrorAction SilentlyContinue
    return $out
}

# connect 之后 CPU 就停了：先冻结 IWDG，否则 dump 到一半会被看门狗复位
$FreezeIwdg = 'w4 0xE0042008 0x1000'

function Get-WordLine([string]$text, [string]$addrPrefix) {
    ($text -split "`r?`n" | Where-Object { $_ -match "^$addrPrefix" } | Select-Object -First 1)
}

# ---------------------------------------------------------------- uid
function Do-Uid([switch]$Quiet) {
    $out = Invoke-JLink @(
        'si SWD', 'connect',
        'mem32 0x1FFF7A10 3',
        'mem32 0x08000000 2',
        'q'
    ) 'uid'

    $uidLine = Get-WordLine $out '1FFF7A10'
    $vecLine = Get-WordLine $out '08000000'
    if (-not $uidLine -or -not $vecLine) {
        Write-Host $out
        throw '读 UID / 向量表失败，看上面的 J-Link 输出'
    }
    $uidWords = ($uidLine -split '=')[1].Trim() -split '\s+'
    $vecWords = ($vecLine -split '=')[1].Trim() -split '\s+'
    $uid0 = $uidWords[0].ToUpper()
    $reset = $vecWords[1].ToUpper()

    if (-not $Quiet) {
        Write-Host ("UID      : {0} {1} {2}" -f $uidWords[0], $uidWords[1], $uidWords[2])
        Write-Host ("向量表   : SP={0}  Reset={1}" -f $vecWords[0], $vecWords[1])
        $known = @{ '00380028' = '新车底盘'; '0024003D' = '新车云台';
                    '00400020' = '旧车底盘(SWD 已坏)'; '0054004D' = '旧车云台' }
        $who = if ($known.ContainsKey($uid0)) { $known[$uid0] } else { '未知板子' }
        Write-Host ("判读     : {0}（{1}）" -f $uid0, $who)
    }
    return @{ Uid = $uid0; Reset = $reset; Raw = $out }
}

# ---------------------------------------------------------------- alive
function Do-Alive() {
    $tickAddr = Get-SymbolAddress 'uwTick'
    Write-Host ("uwTick @ {0}" -f $tickAddr)

    function Read-Tick {
        $out = Invoke-JLink @(
            'si SWD', 'connect',
            $FreezeIwdg,
            ('mem32 {0} 1' -f $tickAddr),
            'g',
            'q'
        ) 'tick'
        $line = Get-WordLine $out ([regex]::Escape($tickAddr.Substring(2).ToUpper()))
        if (-not $line) { Write-Host $out; throw '读 uwTick 失败' }
        return [Convert]::ToUInt32((($line -split '=')[1].Trim()), 16)
    }

    $a = Read-Tick
    Start-Sleep -Milliseconds 700
    $b = Read-Tick
    $delta = [int64]$b - [int64]$a
    Write-Host ("uwTick   : 第一次 {0}，700ms 后 {1}，差值 {2} ms" -f $a, $b, $delta)
    if ($delta -gt 300) {
        Write-Host '判读     : 程序在跑（uwTick 正常增长）'
    } else {
        Write-Host '判读     : 读不到增长。注意先排除「CPU 被 halt 没恢复」：'
        Write-Host '           断电重上电，再用 -Action alive 复测一次；'
        Write-Host '           仍然不涨才可能是真卡死（见 docs/known_issues.md 4.1）'
    }
    return $delta
}

# ---------------------------------------------------------------- dump-dbus
function Do-DumpDbus() {
    $addr = Get-SymbolAddress 'g_dbus_cap'
    # ⚠️ J-Link Commander 的数字参数**一律按十六进制解析**：写 9364 会被当成 0x9364=37732 字节。
    #    所以长度必须写成 0x2494（= 9364 十进制，与 comm_task.cpp 里的 static_assert 对齐）。
    $size = 0x2494
    Write-Host ("g_dbus_cap @ {0} （结构体 {1} 字节 = 0x{1:X}）" -f $addr, $size)
    # 先把已有的文件删掉：J-Link 不会覆盖已存在文件时会静默保留旧内容
    Remove-Item $OutFile -ErrorAction SilentlyContinue
    $out = Invoke-JLink @(
        'si SWD', 'connect',
        $FreezeIwdg,
        ('savebin "{0}", {1}, 0x{2:X}' -f $OutFile, $addr, $size),
        'r', 'g', 'q'
    ) 'dump'
    if (-not (Test-Path $OutFile)) {
        Write-Host $out
        throw "savebin 没有生成文件：$OutFile"
    }

    $bytes = [System.IO.File]::ReadAllBytes($OutFile)
    Write-Host ("导出     : {0}（{1} 字节）" -f $OutFile, $bytes.Length)
    if ($bytes.Length -lt 16) { throw '文件太短，savebin 长度或地址不对' }
    $magic = [BitConverter]::ToUInt32($bytes, 0)
    # 按 16 进制字符串比：0xDB05CA90 超过 Int32 范围，直接写数字字面量会变成负数而比较失败
    if (('{0:X8}' -f $magic) -ne 'DB05CA90') {
        Write-Host ("!! 头部 magic = 0x{0:X8}，预期 0xDB05CA90" -f $magic)
        Write-Host '   → 符号地址不对，或固件不是 kDbusCapture 版本'
        Write-Host ('   前 16 字节: ' + (($bytes[0..15] | ForEach-Object { $_.ToString('x2') }) -join ' '))
        return
    }
    $n = [BitConverter]::ToUInt32($bytes, 4)
    $nTotal = [BitConverter]::ToUInt32($bytes, 8)
    $armFail = [BitConverter]::ToUInt32($bytes, 12)
    Write-Host ("抓包     : 已存 {0}/128 条，总块数 {1}，重挂失败 {2}" -f $n, $nTotal, $armFail)
    if ($nTotal -eq 0) {
        Write-Host '判读     : 一条都没收到 —— 检查接收机供电/信号线，或 USART3 是不是真的那一路'
    } else {
        Write-Host '下一步   : python tools\dbus_capture.py --dump dbus_cap.bin'
    }
}

# ---------------------------------------------------------------- flash
function Do-Flash() {
    if ([string]::IsNullOrWhiteSpace($ExpectedUid)) {
        throw '必须显式给 -ExpectedUid（先跑 -Action uid 看清楚是哪块板）。见 docs/bringup.md 附录 D-4'
    }
    $uid = Do-Uid
    if ($uid.Uid -ne $ExpectedUid.ToUpper()) {
        throw ("UID 不匹配：读到 {0}，预期 {1}。拒绝烧录。" -f $uid.Uid, $ExpectedUid)
    }
    Write-Host 'UID 校验通过，开始烧录…'
    # ⚠️ loadfile 之前必须冻结 IWDG：擦除+写入要几百毫秒到几秒，而 IWDG 在 CPU 被 halt 时
    #    仍然计数（底盘 300 ms）。中途复位会让 flash 写一半 —— 轻则校验失败，重则开不了机。
    #    DBGMCU 属于调试域，写一次管到下次掉电。
    $out = Invoke-JLink @(
        'si SWD', 'connect',
        $FreezeIwdg,
        ('loadfile "{0}"' -f $Elf),
        'r',
        'mem32 0x08000000 2',
        'g', 'qc'
    ) 'flash'
    $vec = Get-WordLine $out '08000000'
    if (-not $vec) { Write-Host $out; throw '烧录后读向量表失败' }
    $reset = (($vec -split '=')[1].Trim() -split '\s+')[1].ToUpper()
    Write-Host ("烧录完成   : Reset = 0x{0}" -f $reset)

    if ($ReadElf) {
        $entry = (& $ReadElf -h $Elf | Select-String 'Entry point address' | Select-Object -First 1)
        if ($entry) {
            # readelf 打印 "0x8006cc9"，mem32 打印 "08006CC9"，按数值比而不是按字符串比
            $entryHex = ($entry.Line -split '0x')[1].Trim()
            $entryVal = [Convert]::ToUInt32($entryHex, 16)
            $resetVal = [Convert]::ToUInt32($reset, 16)
            Write-Host ("ELF 入口   : 0x{0:X8}" -f $entryVal)
            if ($entryVal -eq $resetVal) {
                Write-Host '判读       : 板上固件与源码一致（Reset 向量 == ELF 入口）'
            } else {
                Write-Host '判读       : ✗ 不一致！板上跑的不是这个 ELF，重新烧'
            }
        }
    }
}

# ---------------------------------------------------------------- dump-trace
# 导出「小陀螺自旋诊断」录制区（Chassis/RobotModules/component/robot.cpp 里的 g_trace_*）。
# 数据是固件在 1 kHz 控制周期里按 100 Hz 自己记的，跑的时候 SWD 可以拔掉；
# 停下来再接上导出即可（connect 会 halt CPU，这里先冻结 IWDG 再 savebin）。
#
# 为什么要录而不是"边跑边读"：J-Link 的 connect 会 halt CPU，读到的都是冻结值，
# 单个快照看不出"逐渐变慢"这个过程；而且车在转的时候挂着 SWD 排线容易被绞。
function Get-SymbolSize([string]$name) {
    if (-not $Nm) { throw '找不到 arm-none-eabi-nm，无法查符号大小' }
    $pattern = [regex]::Escape($name) + '$'
    $line = & $Nm -C -S $Elf | Where-Object { $_ -match $pattern } | Select-Object -First 1
    if (-not $line) { return $null }
    $parts = $line -split '\s+'
    return ('0x' + $parts[1])
}

function Do-DumpTrace() {
    $addrBuf   = Get-SymbolAddress 'g_trace_buf'
    $addrN     = Get-SymbolAddress 'g_trace_n'
    $addrState = Get-SymbolAddress 'g_trace_state'
    $addrT0    = Get-SymbolAddress 'g_trace_t0_ms'
    $addrMode  = Get-SymbolAddress 'g_trace_mode'

    # 700 行 × 21 列 × 4 字节 = 58800 = 0xE5B0；从 nm -S 现取，改代码后不会对不上
    $sizeHex = Get-SymbolSize 'g_trace_buf'
    if (-not $sizeHex) { throw '取不到 g_trace_buf 的大小（固件里没有这个符号？先烧带 trace 的版本）' }
    Write-Host ("g_trace_buf @ {0}，大小 {1}" -f $addrBuf, $sizeHex)

    Remove-Item $OutFile -ErrorAction SilentlyContinue
    $out = Invoke-JLink @(
        'si SWD', 'connect',
        $FreezeIwdg,
        ('savebin "{0}", {1}, {2}' -f $OutFile, $addrBuf, $sizeHex),
        ('mem32 {0} 1' -f $addrN),
        ('mem32 {0} 1' -f $addrState),
        ('mem32 {0} 1' -f $addrT0),
        ('mem32 {0} 1' -f $addrMode),
        'g', 'q'
    ) 'trace'

    if (-not (Test-Path $OutFile)) {
        Write-Host $out
        throw "savebin 没有生成文件：$OutFile"
    }

    # mem32 输出形如 "20001A2C = 000002BC"，按地址前缀取行、取 '=' 右边的第一个字
    function Read-WordAt([string]$addr) {
        $prefix = $addr.Substring(2).ToUpper()
        $line = Get-WordLine $out $prefix
        if (-not $line) { return $null }
        return [Convert]::ToUInt32((($line -split '=')[1].Trim() -split '\s+')[0], 16)
    }
    $n     = Read-WordAt $addrN
    $state = Read-WordAt $addrState
    $t0    = Read-WordAt $addrT0
    $mode  = Read-WordAt $addrMode
    if (($null -eq $n) -or ($null -eq $state)) {
        Write-Host $out
        throw '读 g_trace_n / g_trace_state 失败'
    }

    $bytes = [System.IO.File]::ReadAllBytes($OutFile)
    Write-Host ("导出       : {0}（{1} 字节）" -f $OutFile, $bytes.Length)
    $stateName = @{ 0 = '待触发（还没进小陀螺/拨轮没推）'; 1 = '录制中（没录满就断电了）'; 2 = '录满（正常）' }
    $s = if ($stateName.ContainsKey([int]$state)) { $stateName[[int]$state] } else { '未知' }
    Write-Host ("录制状态   : state={0} {1}" -f $state, $s)
    Write-Host ("样本数     : {0} / 700（100 Hz，最长 7.0 s）" -f $n)
    Write-Host ("触发模式   : {0}（2=小陀螺）" -f $mode)
    if ($n -eq 0) {
        Write-Host '判读       : 一条都没录到 —— 没进小陀螺，或拨轮没推到 |w|>0.2 rad/s'
        return
    }

    # 顺手把状态清零，下一次拨轮就能重新录一遍（不用重新上电）
    $null = Invoke-JLink @(
        'si SWD', 'connect', $FreezeIwdg,
        ('w4 {0} 00000000' -f $addrState),
        ('w4 {0} 00000000' -f $addrN),
        'g', 'q'
    ) 'tracereset'

    $sidecar = [ordered]@{
        n = [int]$n; state = [int]$state; t0_ms = [int]$t0; mode = [int]$mode
        rows = 700; cols = 21; row_bytes = 84; file = (Split-Path -Leaf $OutFile)
    }
    $jsonPath = [System.IO.Path]::ChangeExtension($OutFile, 'json')
    ($sidecar | ConvertTo-Json) | Set-Content -Path $jsonPath -Encoding utf8
    Write-Host ("侧车文件   : {0}" -f $jsonPath)
    Write-Host '下一步     : python tools\spin_trace.py --dump spin_trace.bin'
}

# ---------------------------------------------------------------- bench
# 台架/开环测试：绕 PID 直接给四轮同一个电流，车必须架空！
# 为什么要走脚本：duty 是 float，J-Link 的 w4 只收 IEEE-754 位模式，手算容易错。
function Do-Bench() {
    $addrEnable = Get-SymbolAddress 'g_bench_enable'
    $addrDuty = Get-SymbolAddress 'g_bench_open_duty'

    if ($BenchOff -or ($Duty -eq 0)) {
        Write-Host '关台架模式：g_bench_enable = 0，g_bench_open_duty = 0'
        $out = Invoke-JLink @(
            'si SWD', 'connect', $FreezeIwdg,
            ('w4 {0} 00000000' -f $addrEnable),
            ('w4 {0} 00000000' -f $addrDuty),
            'g', 'q'
        ) 'benchoff'
    } else {
        if ([math]::Abs($Duty) -gt 0.05) {
            throw ("duty={0} 太大了：开环是直接给电流，没有速度限幅。0.05 实测就有 20+ rad/s（≈1.7 m/s）。先试 0.005 / 0.01。" -f $Duty)
        }
        $bits = '{0:X8}' -f [BitConverter]::ToUInt32([BitConverter]::GetBytes([float]$Duty), 0)
        Write-Host ("开台架模式 + 开环 duty={0}（{1}）" -f $Duty, $bits)
        $out = Invoke-JLink @(
            'si SWD', 'connect', $FreezeIwdg,
            ('w4 {0} 5A5A5A5A' -f $addrEnable),
            ('w4 {0} {1}' -f $addrDuty, $bits),
            'g', 'q'
        ) 'benchon'
    }

    Start-Sleep -Milliseconds 400
    $chk = Invoke-JLink @(
        'si SWD', 'connect', $FreezeIwdg,
        ('mem32 {0} 2' -f $addrDuty),   # duty 和 enable 在内存里相邻：先 duty 后 enable
        'q'
    ) 'benchchk'
    ($chk -split "`r?`n") | Where-Object { $_ -match '^[0-9A-F]{8} =' }
    Write-Host ("地址       : enable={0}  duty={1}" -f $addrEnable, $addrDuty)
    Write-Host '判读       : enable 读到 5A5A5A5A = 已开；读到 0 = 已关'
    Write-Host '下一步     : 看四个轮子是否同向慢转；测完用 -Action bench -BenchOff 收工'
}

# ---------------------------------------------------------------- zero-check
# 「急停归零 / 离线归零」的软件判据：读四个轮电机的 raw_input_ 与 vel_。
#   raw_input_（对象 +0x58）= 真正要发给电调的值，0 = 电机不输出（这是验收项）
#   vel_（对象 +0x08）      = 反馈转速，0 = 确实没在转
# 只读一个不够：反馈线拔了 vel_ 也恒 0，必须两者一起看。
# 同时读 uwTick：它在涨 = 板子没死、是软件主动归零，不是掉电。
function Do-ZeroCheck() {
    $addrWheels = Get-SymbolAddress 'GetMotorWheel(int)::wheels'
    $addrTick   = Get-SymbolAddress 'uwTick'
    $addrEstop  = Get-SymbolAddress 'robot::g_estop_force'
    $addrForce  = Get-SymbolAddress 'robot::g_force_mode'

    $base = [Convert]::ToUInt32($addrWheels.Substring(2), 16)
    $addrRaw = @()
    $addrVel = @()
    for ($i = 0; $i -lt 4; $i++) {
        $addrRaw += ('0x{0:X8}' -f ($base + 0x68 * $i + 0x58))
        $addrVel += ('0x{0:X8}' -f ($base + 0x68 * $i + 0x08))
    }

    $lines = @('si SWD', 'connect', $FreezeIwdg)
    foreach ($a in $addrRaw) { $lines += "mem32 $a 1" }
    foreach ($a in $addrVel) { $lines += "mem32 $a 1" }
    $lines += "mem32 $addrTick 1"
    $lines += "mem32 $addrEstop 1"
    $lines += "mem32 $addrForce 1"
    $lines += "mem32 $addrDbOk 3"
    $lines += @('g', 'q')
    $out = Invoke-JLink $lines 'zero'

    function ReadW([string]$addr) {
        $line = Get-WordLine $out $addr.Substring(2).ToUpper()
        if (-not $line) { return $null }
        return [Convert]::ToUInt32((($line -split '=')[1].Trim() -split '\s+')[0], 16)
    }
    function ReadF([string]$addr) {
        $w = ReadW $addr
        if ($null -eq $w) { return [double]::NaN }
        return [BitConverter]::ToSingle([BitConverter]::GetBytes([uint32]$w), 0)
    }

    Write-Host '=== 归零检查（四轮电机对象）==='
    Write-Host ('  {0,-4} {1,12} {2,12}' -f '轮', 'raw_input_', 'vel_')
    $allZero = $true
    $names = @('LF', 'LB', 'RB', 'RF')
    for ($i = 0; $i -lt 4; $i++) {
        $r = ReadW $addrRaw[$i]
        $vRaw = ReadW $addrVel[$i]
        if ($null -eq $r -or $null -eq $vRaw) { Write-Host $out; throw '读电机对象失败' }
        $v = [BitConverter]::ToSingle([BitConverter]::GetBytes([uint32]$vRaw), 0)
        if ([math]::Abs([double]$r) -gt 0.5) { $allZero = $false }
        Write-Host ('  {0,-4} {1,12} {2,12:F2}' -f $names[$i], $r, $v)
    }
    $tick = ReadW $addrTick
    $estop = ReadW $addrEstop
    $force = ReadW $addrForce
    Write-Host ''
    Write-Host ('uwTick       : {0} ms（在涨说明板子活着，是软件归零）' -f $tick)
    Write-Host ('g_estop_force: {0}' -f $estop)
    Write-Host ('g_force_mode : {0}' -f $force)
    Write-Host ''
    if ($allZero) {
        Write-Host '判读：四轮 raw_input_ 全为 0 —— 归零成立（电机确实没收到输出）'
    } else {
        Write-Host '判读：✗ 有轮子 raw_input_ 非 0，归零没成立（看是不是还停在 Working）'
    }
}

# ---------------------------------------------------------------- wdg-demo
# 看门狗复位演示（底盘超时 300 ms / 云台 600 ms，见 Core/Src/iwdg.c 的 prescaler×reload/32 kHz）。
# 原理：IWDG 由 LSI 独立驱动，CPU 被 J-Link halt 住时**照样计数**，除非
# DBGMCU_APB1_FZ 的 DBG_IWDG_STOP(bit12) = 1。所以演示要先把这一位清 0，
# 再故意 halt 超过超时时间，板子就会自己复位。
#
# ⚠️ 复位会清掉所有 RAM 调试变量（g_estop_force / g_force_mode / g_ignore_delta 回默认），
#    所以演示前必须让车处于**物理安全状态**：四轮架空，或左摇杆拉到底（急停）。
#
# 判据：RCC_CSR(0x40023874) bit29 (IWDGRSTF) = 1，且复位后能自动恢复（重新进 Working）。
function Do-WdgDemo() {
    $csr = '0x40023874'
    $dbg = '0xE0042008'
    $tickAddr = Get-SymbolAddress 'uwTick'

    Write-Host '=== 看门狗复位演示（底盘 300 ms）==='
    Write-Host '步骤 0：清 DBG_IWDG_STOP（否则 halt 时狗也停），并清复位标志'
    $out = Invoke-JLink @(
        'si SWD', 'connect',
        ('w4 {0} 00000000' -f $dbg),
        ('w4 {0} 01000000' -f $csr),   # RMVF=1，清掉历史复位标志
        ('mem32 {0} 1' -f $csr),
        ('mem32 {0} 1' -f $dbg),
        'g', 'q'
    ) 'wdgprep'
    ($out -split "`r?`n") | Where-Object { $_ -match '^[0-9A-F]{8} =' } | ForEach-Object { Write-Host ('  ' + $_) }
    Write-Host '  判读：CSR 现在应为 00000000（没有历史复位标志）'

    Write-Host '步骤 1：halt 住 CPU 1.5 s（超过 300 ms 超时），让 IWDG 咬'
    $null = Invoke-JLink @('si SWD', 'connect', 'sleep 1500', 'q') 'wdgbite'
    Start-Sleep -Milliseconds 400

    Write-Host '步骤 2：读复位后的 RCC_CSR 与 uwTick'
    $out2 = Invoke-JLink @(
        'si SWD', 'connect', $FreezeIwdg,
        ('mem32 {0} 1' -f $csr),
        ('mem32 {0} 1' -f $tickAddr),
        'g', 'q'
    ) 'wdgafter'
    $csrLine = Get-WordLine $out2 '40023874'
    $tickLine = Get-WordLine $out2 (($tickAddr.Substring(2)).ToUpper())
    if (-not $csrLine) { Write-Host $out2; throw '读 RCC_CSR 失败' }
    $csrVal = [Convert]::ToUInt32((($csrLine -split '=')[1].Trim()), 16)
    Write-Host ('  RCC_CSR = 0x{0:X8}' -f $csrVal)
    $bits = @{ 29 = 'IWDG 复位'; 28 = '软件复位'; 27 = '上电复位'; 26 = '引脚复位'; 25 = 'BOR 复位' }
    $hit = @()
    foreach ($b in 25..29) { if (($csrVal -band [uint32](1 -shl $b)) -ne 0) { $hit += $bits[$b] } }
    if ($hit.Count -gt 0) { Write-Host ('  置位：' + ($hit -join '、')) } else { Write-Host '  置位：无' }
    if (($csrVal -band 0x20000000) -ne 0) {
        Write-Host '判读：✓ IWDG 复位成立（bit29 = 1）'
        Write-Host '      接着看车有没有自动恢复：摇杆回中 → 应重新进 Working，轮子不动、云台重新指稳'
    } else {
        Write-Host '判读：✗ bit29 没置位 —— 检查 DBG_IWDG_STOP 是否真的清成 0（步骤 0 的回读）'
    }
}

# ---------------------------------------------------------------- main
switch ($Action) {
    'uid'       { $null = Do-Uid }
    'alive'     { $null = Do-Alive }
    'dump-dbus' { Do-DumpDbus }
    'dump-trace' { Do-DumpTrace }
    'flash'     { Do-Flash }
    'bench'     { Do-Bench }
    'zero-check' { Do-ZeroCheck }
    'wdg-demo'  { Do-WdgDemo }
}
