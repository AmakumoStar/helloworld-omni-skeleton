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
    [ValidateSet('uid', 'alive', 'dump-dbus', 'flash')]
    [string]$Action = 'uid',

    [ValidateSet('chassis', 'gimbal')]
    [string]$Board = 'chassis',

    # 仓库根目录（默认由脚本位置推断：tools\ 的上一级）
    [string]$Repo = '',

    # dump-dbus 的输出文件；空则写到当前目录下的 dbus_cap.bin
    [string]$OutFile = '',

    # flash 时必须给：先跑 -Action uid 看清楚是哪块板
    [string]$ExpectedUid = ''
)

$ErrorActionPreference = 'Stop'

# ---------------------------------------------------------------- 环境
if ([string]::IsNullOrWhiteSpace($Repo)) {
    $Repo = Split-Path -Parent $PSScriptRoot
}
if ([string]::IsNullOrWhiteSpace($OutFile)) {
    $OutFile = Join-Path (Get-Location) 'dbus_cap.bin'
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
    $out = & $JLink -device STM32F407IG -if SWD -speed 1000 -autoconnect 1 `
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
        'si SWD', 'speed 1000', 'connect',
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
            'si SWD', 'speed 1000', 'connect',
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
    Write-Host ("g_dbus_cap @ {0} （结构体 9364 字节）" -f $addr)
    # 先把已有的文件删掉：J-Link 不会覆盖已存在文件时会静默保留旧内容
    Remove-Item $OutFile -ErrorAction SilentlyContinue
    $out = Invoke-JLink @(
        'si SWD', 'speed 1000', 'connect',
        $FreezeIwdg,
        ('savebin "{0}", {1}, 9364' -f $OutFile, $addr),
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
    if ($magic -ne 0xDB05CA90) {
        Write-Host ("!! 头部 magic = 0x{0:X8}，预期 0xDB05CA90" -f $magic)
        Write-Host '   → 符号地址不对，或固件不是 kDbusCapture 版本'
        Write-Host '   前 16 字节: ' (($bytes[0..15] | ForEach-Object { $_.ToString('x2') }) -join ' ')
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
    $out = Invoke-JLink @(
        'si SWD', 'speed 1000', 'connect',
        'r', 'h',
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

# ---------------------------------------------------------------- main
switch ($Action) {
    'uid'       { $null = Do-Uid }
    'alive'     { $null = Do-Alive }
    'dump-dbus' { Do-DumpDbus }
    'flash'     { Do-Flash }
}
