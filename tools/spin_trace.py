#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""小陀螺自旋诊断：解析固件里录的 g_trace_buf，直接给出「不匀速」的判读。

数据来源（不需要调试器在线，SWD 可以先拔掉）：
    pwsh -File tools\\jlink.ps1 -Action dump-trace -Board chassis
    python tools\\spin_trace.py --dump spin_trace.bin

录的是什么（Chassis/RobotModules/component/robot.cpp 的 TraceTick）：
    100 Hz × 7 s，每行 21 个 float：
      t_ms, mode, w_cmd, delta,
      四轮目标 m/s, 四轮实际 rad/s, 四轮速度环输出 raw,
      四轮 1ms 级速度跳变量, 底盘 IMU 绝对 yaw

判据（与 docs/handoff_2026-11-17.md 第四节一致）：
    1) w_cmd 自己在衰减          → 问题在拨轮/遥控，不在闭环
    2) 轮子跟住目标、车却不转    → 麦轮打滑
    3) 轮子没跟住目标 / 输出饱和 → 轮速环权限不够，或被拖住
    4) 某个轮子 jitter 特别大    → 反馈跳变（两颗电调同 ID 的典型特征）
"""

import argparse
import csv
import json
import os
import sys

import numpy as np

WHEELS = ["LF", "LB", "RB", "RF"]

# 与 main_task.cpp 的 kMaxWheelVelMps / ins_pid 的 out_max 对齐（只用于判读，不参与控制）
DEFAULT_MAX_WHEEL_MPS = 0.3
DEFAULT_OUT_MAX_RAW = 3000.0
# 纯自旋时四轮速度 = w · (半轴距 + 半轮距)；实车 0.15338 + 0.15338
DEFAULT_LEVER = 0.30676

COL_T_MS = 0
COL_MODE = 1
COL_WCMD = 2
COL_DELTA = 3
COL_TGT = 4
COL_VEL = 8
COL_OUT = 12
COL_JIT = 16
COL_IMU = 20
NCOLS = 21


class Tee:
    """控制台照常打印，同时把整份报告按 UTF-8 存成 txt。

    为什么要存文件：Windows 控制台默认是 GBK，中文在 cmd.exe / 某些终端里会花屏；
    报告落盘成 UTF-8 就不会因为终端编码把判读结论看丢。"""

    def __init__(self, path):
        self.path = path
        self.f = open(path, "w", encoding="utf-8")

    def write(self, s):
        sys.__stdout__.write(s)
        self.f.write(s)

    def flush(self):
        try:
            sys.__stdout__.flush()
        except Exception:
            pass
        self.f.flush()


def load(dump_path, n_override=None):
    """读 bin + 同名 json 侧车，返回 (n, data[n,21])。"""
    if not os.path.exists(dump_path):
        sys.exit(f"找不到 {dump_path}：先跑 jlink.ps1 -Action dump-trace")

    n = n_override
    meta = {}
    sidecar = os.path.splitext(dump_path)[0] + ".json"
    if os.path.exists(sidecar):
        with open(sidecar, "r", encoding="utf-8") as f:
            meta = json.load(f)
        if n is None:
            n = int(meta.get("n", 0))
    if n is None:
        sys.exit("没有 json 侧车也没给 --n：无法知道录了多少条")

    raw = np.fromfile(dump_path, dtype="<f4")
    if raw.size % NCOLS != 0:
        sys.exit(f"文件长度 {raw.size} 不是 {NCOLS} 的整数倍：dump 大小/地址不对")
    rows = raw.reshape(-1, NCOLS)
    n = max(0, min(n, rows.shape[0]))
    meta.setdefault("state", -1)
    meta.setdefault("mode", -1)
    return n, rows[:n], meta


def unwrap(x):
    return np.unwrap(np.asarray(x, dtype=np.float64))


def rate(x, t):
    """数值微分，端点用前向/后向差分，避免 gradient 边缘效应。"""
    x = np.asarray(x, dtype=np.float64)
    if x.size < 2:
        return np.zeros_like(x)
    dt = np.diff(t)
    dt[dt <= 0] = 1e-3
    d = np.empty_like(x)
    d[0] = (x[1] - x[0]) / dt[0]
    d[-1] = (x[-1] - x[-2]) / dt[-1]
    if x.size > 2:
        d[1:-1] = (x[2:] - x[:-2]) / (dt[1:] + dt[:-1])
    return d


def window_mean(arr, mask):
    if mask.sum() == 0:
        return float("nan")
    return float(np.mean(arr[mask]))


def main():
    ap = argparse.ArgumentParser(description="小陀螺自旋诊断（解析 g_trace_buf 导出）")
    ap.add_argument("--dump", default="spin_trace.bin", help="jlink.ps1 -Action dump-trace 导出的 bin")
    ap.add_argument("--n", type=int, default=None, help="样本数（默认读 json 侧车）")
    ap.add_argument("--radius", type=float, default=0.07786, help="轮半径 R，m")
    ap.add_argument("--lever", type=float, default=DEFAULT_LEVER, help="等效臂 k=半轴距+半轮距，m")
    ap.add_argument("--max-wheel", type=float, default=DEFAULT_MAX_WHEEL_MPS,
                    help="kMaxWheelVelMps（单轮线速度闸门）")
    ap.add_argument("--out-max", type=float, default=DEFAULT_OUT_MAX_RAW,
                    help="轮速环 out_max（raw）")
    ap.add_argument("--plot", default="", help="输出 PNG 路径（可选）")
    ap.add_argument("--csv", default="", help="输出 CSV 路径（可选，默认同目录 .csv）")
    args = ap.parse_args()

    # 报告同时落盘（UTF-8），避免 Windows 控制台编码问题把中文判读吃掉
    report_path = os.path.splitext(args.dump)[0] + "_report.txt"
    sys.stdout = Tee(report_path)

    n, data, meta = load(args.dump, args.n)
    if n <= 0:
        sys.exit("录到 0 条：没进小陀螺，或拨轮没推到 |w|>0.2 rad/s（脚本已自动清零，可重录）")

    t = data[:, COL_T_MS].astype(np.float64) / 1000.0
    t = t - t[0]
    mode = int(round(float(np.median(data[:, COL_MODE]))))
    w_cmd = data[:, COL_WCMD].astype(np.float64)
    delta = unwrap(data[:, COL_DELTA])
    imu = unwrap(data[:, COL_IMU])
    tgt = data[:, COL_TGT:COL_TGT + 4].astype(np.float64)       # m/s
    vel = data[:, COL_VEL:COL_VEL + 4].astype(np.float64)       # rad/s
    out = data[:, COL_OUT:COL_OUT + 4].astype(np.float64)       # raw
    jit = data[:, COL_JIT:COL_JIT + 4].astype(np.float64)       # rad/s per 10ms

    tgt_rad = tgt / max(args.radius, 1e-6)
    err = tgt_rad - vel

    w_imu = rate(imu, t)
    # δ = 云台关节角 - 车头角；云台保绝对朝向时 dδ/dt = -ω_车
    w_delta = -rate(delta, t)

    dur = float(t[-1] - t[0]) if n > 1 else 0.0
    print("=" * 78)
    print(f"自旋诊断  {os.path.basename(args.dump)}")
    print(f"样本 {n} 条 / {dur:.2f} s   mode={mode} (2=小陀螺)   录制状态 state={meta.get('state')}")
    print("=" * 78)

    # ---------------- 时间轴摘要（每 ~0.5 s 一行）
    step = max(1, int(round(0.5 / max(np.median(np.diff(t)) if n > 1 else 0.01, 0.005))))
    print("\n[时间轴]  t(s)  w_cmd  ω_imu  ω_δ   |  目标rad/s LF/LB/RB/RF   |  实际rad/s LF/LB/RB/RF  |  out最大")
    idxs = list(range(0, n, step))
    if idxs[-1] != n - 1:
        idxs.append(n - 1)
    for i in idxs:
        print(f"  {t[i]:5.2f} {w_cmd[i]:6.3f} {w_imu[i]:6.3f} {w_delta[i]:6.3f} | "
              f"{tgt_rad[i,0]:6.2f} {tgt_rad[i,1]:6.2f} {tgt_rad[i,2]:6.2f} {tgt_rad[i,3]:6.2f} | "
              f"{vel[i,0]:6.2f} {vel[i,1]:6.2f} {vel[i,2]:6.2f} {vel[i,3]:6.2f} | "
              f"{np.max(np.abs(out[i])):6.0f}")

    # ---------------- 早/晚两段对比（"逐渐变慢"就看这个）
    t_early = (t >= 0.3) & (t <= 1.3)
    t_late = t >= max(0.0, t[-1] - 1.5)
    if t_early.sum() < 3:
        t_early = t <= max(0.3, t[-1] * 0.4)
    if t_late.sum() < 3:
        t_late = t >= t[-1] * 0.6

    def blk(mask):
        return {
            "w_cmd": window_mean(np.abs(w_cmd), mask),
            "w_imu": window_mean(np.abs(w_imu), mask),
            "o_ratio": window_mean(np.abs(w_imu), mask) / max(window_mean(np.abs(w_cmd), mask), 1e-6),
            "err": float(np.mean(np.abs(err[mask]))) if mask.sum() else float("nan"),
            "out": float(np.max(np.abs(out[mask]))) if mask.sum() else float("nan"),
            "jit": float(np.max(np.mean(jit[mask], axis=0))) if mask.sum() else float("nan"),
        }

    early = blk(t_early)
    late = blk(t_late)
    print("\n[早/晚对比]      |w_cmd|   |ω_imu|   ω/指令   轮速跟踪误差   max|out|   max jitter")
    for name, b in (("早段", early), ("晚段", late)):
        print(f"  {name:6s}      {b['w_cmd']:7.3f}  {b['w_imu']:7.3f}  {b['o_ratio']:7.3f}  "
              f"{b['err']:10.3f}  {b['out']:9.0f}  {b['jit']:9.2f}")

    # ---------------- 几何自检：纯自旋时 |target| / |w_cmd| 应 ≈ k
    spin_mask = np.abs(w_cmd) > 0.2
    k_eff = float("nan")
    if spin_mask.sum() > 0:
        ratio = np.max(np.abs(tgt[spin_mask]), axis=1) / np.abs(w_cmd[spin_mask])
        k_eff = float(np.median(ratio))
    gate = args.max_wheel / max(args.radius, 1e-6)
    gate_hit = float(np.max(np.abs(tgt))) >= 0.995 * args.max_wheel

    print(f"\n[几何自检] 纯自旋 目标轮速/角速度 = {k_eff:.4f} m/rad（期望 {args.lever:.4f}，"
          f"偏差 {100.0*(k_eff/args.lever-1.0):+.1f}%）")
    print(f"           单轮闸门 {args.max_wheel:.2f} m/s = {gate:.2f} rad/s，"
          f"本次最大目标 {np.max(np.abs(tgt)):.3f} m/s → {'被闸门削到' if gate_hit else '没触发闸门'}")

    # ---------------- 判据（handoff 第四节的三条 + 同 ID 反馈跳变）
    print("\n[判读]")
    verdicts = []
    decay = (late["w_cmd"] / early["w_cmd"]) if early["w_cmd"] > 1e-6 else 1.0
    if decay < 0.9:
        verdicts.append(("1 指令在衰减", f"晚段 |w_cmd| 只剩早段的 {decay*100:.0f}% "
                                        f"→ 先查拨轮/DBUS，不是闭环问题", "warn"))
    else:
        verdicts.append(("1 指令稳定", f"晚段/早段 = {decay*100:.0f}%，拨轮指令没有掉", "ok"))

    track_ok = late["err"] < 0.3
    sat_pct = 100.0 * float(np.mean(np.abs(out[t_late]) > 0.95 * args.out_max)) if t_late.sum() else 0.0
    if track_ok and sat_pct < 5.0:
        verdicts.append(("2 轮子跟住了目标", f"跟踪误差 {late['err']:.2f} rad/s，输出饱和占比 {sat_pct:.0f}%", "ok"))
        if late["o_ratio"] < 0.85:
            verdicts.append(("3 车体没跟上轮子", f"实际转速只有指令的 {late['o_ratio']*100:.0f}% "
                                               f"→ 麦轮打滑（轮子空转、车转得慢）", "bad"))
        else:
            verdicts.append(("3 车体跟得上", f"实际/指令 = {late['o_ratio']*100:.0f}%，不是打滑", "ok"))
    else:
        verdicts.append(("2 轮子没跟住目标", f"跟踪误差 {late['err']:.2f} rad/s，"
                                            f"输出饱和占比 {sat_pct:.0f}% → 轮速环权限不够 / 被拖住", "bad"))

    jitter_bad = [WHEELS[i] for i in range(4) if late["jit"] > 5.0]
    if jitter_bad:
        verdicts.append(("4 反馈跳变", f"{'/'.join(jitter_bad)} 的 jitter={late['jit']:.1f} "
                                      f"→ 疑两颗电调同 ID（0x201 帧数 2 倍），去核对 C620 拨码", "bad"))
    else:
        verdicts.append(("4 反馈平稳", f"最大 jitter={late['jit']:.2f}，不像同 ID 交叠", "ok"))

    sym = {"ok": "  OK ", "warn": "  ?? ", "bad": "  !! "}
    for name, detail, kind in verdicts:
        print(f"{sym[kind]}{name}：{detail}")

    # ---------------- CSV
    csv_path = args.csv or os.path.splitext(args.dump)[0] + ".csv"
    header = (["t_s", "mode", "w_cmd", "delta", "imu_yaw", "w_imu", "w_delta"]
              + [f"tgt_{w}_mps" for w in WHEELS] + [f"tgt_{w}_rads" for w in WHEELS]
              + [f"vel_{w}_rads" for w in WHEELS] + [f"err_{w}_rads" for w in WHEELS]
              + [f"out_{w}_raw" for w in WHEELS] + [f"jit_{w}" for w in WHEELS])
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(header)
        for i in range(n):
            wr.writerow([f"{t[i]:.4f}", mode, f"{w_cmd[i]:.4f}", f"{data[i,COL_DELTA]:.4f}",
                         f"{data[i,COL_IMU]:.4f}", f"{w_imu[i]:.4f}", f"{w_delta[i]:.4f}"]
                        + [f"{v:.4f}" for v in tgt[i]] + [f"{v:.3f}" for v in tgt_rad[i]]
                        + [f"{v:.3f}" for v in vel[i]] + [f"{v:.3f}" for v in err[i]]
                        + [f"{v:.0f}" for v in out[i]] + [f"{v:.2f}" for v in jit[i]])
    print(f"\nCSV: {csv_path}")
    print(f"报告(UTF-8): {report_path}")

    if args.plot:
        try:
            plot_png(args.plot, t, w_cmd, w_imu, w_delta, tgt_rad, vel, out)
            print(f"PNG: {args.plot}")
        except Exception as exc:  # 画图失败不影响判读
            print(f"画图失败（不影响结论）：{exc}")


def plot_png(path, t, w_cmd, w_imu, w_delta, tgt_rad, vel, out):
    """用 Pillow 画 4 联图（仓库只保证有 numpy/pandas/Pillow）。"""
    from PIL import Image, ImageDraw

    W, H = 1100, 760
    left, right, top, bottom = 70, 20, 40, 40
    panel_h = (H - top - bottom - 3 * 26) // 4
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)

    def px(tt):
        return left + (W - left - right) * tt

    panels = [
        ("w_cmd / omega", [(w_cmd, (200, 0, 0)), (w_imu, (0, 90, 200)), (w_delta, (0, 160, 90))]),
        ("wheel target rad/s", [(tgt_rad[:, i], c) for i, c in
                                enumerate([(200, 0, 0), (230, 130, 0), (0, 130, 60), (0, 60, 200)])]),
        ("wheel actual rad/s", [(vel[:, i], c) for i, c in
                                enumerate([(200, 0, 0), (230, 130, 0), (0, 130, 60), (0, 60, 200)])]),
        ("wheel out raw", [(out[:, i], c) for i, c in
                           enumerate([(200, 0, 0), (230, 130, 0), (0, 130, 60), (0, 60, 200)])]),
    ]
    t0, t1 = float(t[0]), float(t[-1]) if t[-1] > t[0] else 1.0
    for pi, (title, series) in enumerate(panels):
        y0 = top + pi * (panel_h + 26)
        y1 = y0 + panel_h
        d.rectangle([left, y0, W - right, y1], outline=(180, 180, 180))
        d.text((left + 6, y0 + 3), title, fill=(0, 0, 0))
        allv = np.concatenate([s[np.isfinite(s)] for s, _ in series]) if series else np.array([0.0])
        lo, hi = float(np.min(allv)), float(np.max(allv))
        if hi - lo < 1e-6:
            hi, lo = hi + 1.0, lo - 1.0
        pad = 0.08 * (hi - lo)
        lo, hi = lo - pad, hi + pad
        yzero = y1 - (y1 - y0) * (0.0 - lo) / (hi - lo)
        if y0 <= yzero <= y1:
            d.line([left, yzero, W - right, yzero], fill=(225, 225, 225))
        d.text((left - 60, y0), f"{hi:+.2f}", fill=(110, 110, 110))
        d.text((left - 60, y1 - 10), f"{lo:+.2f}", fill=(110, 110, 110))
        for s, col in series:
            pts = [(px((float(t[i]) - t0) / (t1 - t0 or 1.0)),
                    y1 - (y1 - y0) * (min(max(float(s[i]), lo), hi) - lo) / (hi - lo))
                   for i in range(len(t))]
            if len(pts) > 1:
                d.line(pts, fill=col, width=2)
    d.text((left, 8), "spin trace (red=LF orange=LB green=RB blue=RF; panel1: red w_cmd, "
                      "blue omega_imu, green omega_delta)", fill=(0, 0, 0))
    img.save(path)


if __name__ == "__main__":
    main()
