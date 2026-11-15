# =====================================================================
#  omni_skeleton 编译 + 烧录（底盘板 → 云台板）
#  用法：整段粘进 PowerShell，或在 omni_skeleton 目录执行 .\flash.ps1
#  若提示禁止运行脚本：powershell -ExecutionPolicy Bypass -File .\flash.ps1
# =====================================================================
$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot

# ---------- 0) 定位 J-Link ----------
$jlink = @(
    'C:\Program Files\SEGGER\JLink_V978\JLink.exe',
    'C:\Program Files\SEGGER\JLink\JLink.exe',
    'C:\Program Files (x86)\SEGGER\JLink\JLink.exe'
) | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $jlink) { throw '找不到 JLink.exe（Ozone 自带的那份不行，需要 SEGGER J-Link 软件包）' }
Write-Host "J-Link: $jlink" -ForegroundColor DarkGray

# ---------- 1) 编译两块板 ----------
function Build-Board([string]$name, [string]$buildDir) {
    Write-Host "`n=== 编译 $name ===" -ForegroundColor Cyan
    cmake --build $buildDir
    if ($LASTEXITCODE -ne 0) { throw "$name 编译失败，先解决编译错误" }
}

Build-Board '底盘板' "$root\Chassis\build\w"
Build-Board '云台板' "$root\Gimbal\build\w"

# ---------- 2) 校验 ELF 是否比源码新（防止烧到旧固件） ----------
function Assert-Fresh([string]$name, [string]$elf, [string]$projDir) {
    if (-not (Test-Path $elf)) { throw "找不到固件：$elf" }
    $elfTime = (Get-Item $elf).LastWriteTime
    $newest = @(
        Get-ChildItem $projDir -Recurse -Include *.cpp, *.hpp, *.c, *.h -ErrorAction SilentlyContinue |
            Where-Object { $_.FullName -notmatch '\\build\\|third_party' }
        Get-ChildItem "$root\RobotComponents" -Recurse -Include *.cpp, *.hpp -ErrorAction SilentlyContinue
    ) | Sort-Object LastWriteTime -Descending | Select-Object -First 1

    Write-Host ("{0}: ELF = {1}" -f $name, $elfTime.ToString('MM-dd HH:mm:ss')) -ForegroundColor DarkGray
    if ($newest -and $elfTime -lt $newest.LastWriteTime) {
        throw ("{0} 的 ELF 比源码旧！最新源码：{1} ({2})" -f $name, $newest.Name, $newest.LastWriteTime.ToString('MM-dd HH:mm:ss'))
    }
}

# ---------- 3) 烧录一块板 ----------
function Flash-Board([string]$name, [string]$elf) {
    $cmdFile = Join-Path $env:TEMP 'flash_board.jlink'
    Set-Content -Path $cmdFile -Encoding ASCII -Value @(
        'r'
        'h'
        ('loadfile "{0}"' -f $elf)
        'mem32 0x08000000 2'
        'r'
        'g'
        'qc'
    )
    Write-Host "`n=== 烧录 $name ===" -ForegroundColor Cyan
    & $jlink -device STM32F407IG -if SWD -speed 1000 -autoconnect 1 -ExitOnError 1 -CommanderScript $cmdFile
    if ($LASTEXITCODE -ne 0) {
        throw "$name 烧录失败（退出码 $LASTEXITCODE）。查：① SWD 接线 ② 板子供电 ③ Ozone 是否还开着占用 J-Link"
    }
}

# ---------- 4) 依次烧两块 ----------
$chassisElf = "$root\Chassis\build\omni_chassis.elf"
$gimbalElf = "$root\Gimbal\build\omni_gimbal.elf"

Assert-Fresh '底盘板' $chassisElf "$root\Chassis"
Flash-Board '底盘板' $chassisElf

Read-Host "`n把 J-Link 的 SWD 线从底盘板换到云台板，然后按回车继续" | Out-Null

Assert-Fresh '云台板' $gimbalElf "$root\Gimbal"
Flash-Board '云台板' $gimbalElf

Write-Host "`n两块板都烧完了。脚本最后执行了 g（运行），板子已在跑，可以拔 J-Link。" -ForegroundColor Green
Write-Host "确认无异常动作后，再放开急停/接双板 CAN。" -ForegroundColor Yellow
