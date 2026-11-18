#!/usr/bin/env python3
"""轮速环阶跃响应判读（配合 tools/jlink.ps1 -Action dump-trace）。

用途：架空状态下把拨轮从中间**一次性拨到某个位置**（= 四轮目标速度的阶跃），
用固件内置的 100 Hz 录制把 `tgt / vel / out` 存下来，离线算这组数：

    上升时间 t10~t90、超调量、稳定时间、稳态误差、out 饱和度、振荡次数

判读要点（"一开始快然后变慢"是什么样子）：
    · 超调量 > 10% 且后面靠 P 拉回  → 前馈 ff 偏大 / kp 偏软
    · t90 明显长、超调小            → 前馈不足、kp 偏低（起步拖）
    · out 长时间贴在 out_max        → 输出饱和，增益/限幅不匹配

用法：
    python tools/wheel_step.py --dump spin_trace.bin [--plot step.png]
    python tools/wheel_step.py --dump spin_trace.bin --label "150/150/700"
"""

import argparse
import os
import sys

import numpy as np

WHEELS = ["LF", "LB", "RB", "RF"]
NCOLS = 21
COL_T, COL_WCMD = 0, 2
COL_TGT, COL_VEL, COL_OUT = 4, 8, 12
DT = 0.01


def load(path):
    raw = np.fromfile(path, dtype="<f4")
    if raw.size == 0:
        sys.exit(f"{path} 是空的：先跑 jlink.ps1 -Action dump-trace")
    if raw.size % NCOLS:
        sys.exit(f"文件长度 {raw.size} 不是 {NCOLS} 的整数倍：dump 地址/大小不对")
    d = raw.reshape(-1, NCOLS)
    return d[np.any(d != 0.0, axis=1)]


def step_metrics(t, target, actual, out, out_max=3000.0, tol=0.05):
    """对某一轮，用 target 的上升沿做基准，算阶跃响应指标。

    ⚠️ 稳态必须取「阶跃持续期间的最后一段」，不能取录制窗口末尾 ——
    录制是 7 s，而台阶通常只有 2~3 s，末尾轮子早就停了（旧版本在这里判成"无阶跃"）。
    """
    a = np.abs(target)
    peak = float(np.max(a))
    if peak < 0.3:
        return None
    idx = int(np.argmax(a > 0.1 * peak))          # 目标开始动的那一拍
    # 台阶结束：目标回落到峰值的 50% 以下（拨轮收回）
    after = np.nonzero(a[idx:] < 0.5 * peak)[0]
    end = idx + int(after[0]) if after.size else len(a) - 1
    if end - idx < 20:                            # 台阶短于 200 ms 不算
        return None

    t0 = t[idx]
    sign = np.sign(target[idx])
    tg = target[idx:end] * sign
    av = actual[idx:end] * sign
    ou = out[idx:end] * sign

    final = float(np.mean(av[-max(1, int(0.3 / DT)):]))   # 台阶最后 0.3 s 当稳态
    if final <= 1e-3:
        return None

    def first_cross(frac):
        hit = np.argmax(av >= frac * final)
        return float(t[idx + hit] - t0) if av[hit] >= frac * final else float("nan")

    t10, t90 = first_cross(0.1), first_cross(0.9)
    ovs = (float(np.max(av)) - final) / final * 100.0

    # 稳定时间：之后所有点都在 ±tol 带内
    band = np.abs(av - final) > tol * abs(final)
    if band.any():
        last = int(np.max(np.nonzero(band)[0]))
        t_settle = float(t[idx + last] - t0) if last + 1 < len(av) else float("nan")
    else:
        t_settle = 0.0

    cross = int(np.sum(np.diff(np.sign(av - final)) != 0))
    sat = float(np.max(np.abs(ou)) / out_max * 100.0)
    return dict(t10=t10, t90=t90, ovs=ovs, t_settle=t_settle, cross=cross,
                final=final, sat=sat, t0=t0, span=(end - idx) * DT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", default="spin_trace.bin")
    ap.add_argument("--label", default="")
    ap.add_argument("--out_max", type=float, default=3000.0)
    ap.add_argument("--radius", type=float, default=0.07786,
                    help="轮半径 m（录制里 tgt 是 m/s，需换算成 rad/s）")
    ap.add_argument("--plot", default="")
    args = ap.parse_args()
    if not os.path.exists(args.dump):
        sys.exit(f"找不到 {args.dump}")

    d = load(args.dump)
    t = d[:, COL_T] / 1000.0
    # ⚠️ 录制里两列单位不一致（2026-11-18 实测确认）：
    #     tgt = 逆解器输出的**轮子线速度 m/s**（WheelVel::v 的定义）
    #     vel = 电机**输出端角速度 rad/s**（M3508::vel()）
    # 不换算就会得到"实际/目标 = 1284%" 这种假数字（spin_trace.py 里那条 85361% 同源）。
    tgt = d[:, COL_TGT:COL_TGT + 4] / args.radius
    vel = d[:, COL_VEL:COL_VEL + 4]
    out = d[:, COL_OUT:COL_OUT + 4]

    print("=" * 84)
    print(f"轮速环阶跃判读  {args.dump}  {len(d)} 行 / {len(d) * DT:.2f} s"
          + (f"   参数 {args.label}" if args.label else ""))
    print("=" * 84)

    rows = []
    for i, w in enumerate(WHEELS):
        m = step_metrics(t, tgt[:, i], vel[:, i], out[:, i], args.out_max)
        if m is None:
            print(f"  {w}: 没有有效阶跃（目标峰值 < 0.3 rad/s）")
            continue
        rows.append((w, i, m))
        print(f"\n  [{w}] 目标峰值 {np.max(np.abs(tgt[:, i])):6.2f} rad/s   稳态 {m['final']:6.2f}")
        print(f"        t10 {m['t10'] * 1000:5.0f} ms   t90 {m['t90'] * 1000:5.0f} ms"
              f"   超调 {m['ovs']:+6.1f}%   稳定 {m['t_settle'] * 1000:5.0f} ms"
              f"   过零 {m['cross']:2d} 次   out 峰值 {m['sat']:5.1f}%")

    if not rows:
        sys.exit(0)

    over = [m["ovs"] for _, _, m in rows]
    t90s = [m["t90"] for _, _, m in rows]
    print("\n[汇总]")
    print(f"  平均 t90 = {np.nanmean(t90s) * 1000:.0f} ms    平均超调 = {np.mean(over):+.1f}%"
          f"    最大超调 = {np.max(over):+.1f}%")
    if np.mean(over) > 10.0:
        print("  ✗ 超调偏大：前馈 ff 多半过量（动摩擦 << 静摩擦），或 kp 偏软靠积分补")
    elif np.nanmean(t90s) > 0.25:
        print("  ✗ 上升偏慢：前馈不足或 kp 偏低（起步被静摩擦拖住）")
    else:
        print("  ✓ 阶跃形状可接受（超调 <10% 且 t90 <250 ms）")

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
            for i in range(4):
                ax[0].plot(t, tgt[:, i], "--", lw=0.9)
                ax[0].plot(t, vel[:, i], lw=1.1, label=WHEELS[i])
            ax[0].set_ylabel("wheel rad/s"); ax[0].legend(ncol=4); ax[0].grid(alpha=.3)
            for i in range(4):
                ax[1].plot(t, out[:, i], lw=1.0)
            ax[1].set_ylabel("out (raw)"); ax[1].grid(alpha=.3); ax[1].set_xlabel("t (s)")
            fig.tight_layout(); fig.savefig(args.plot, dpi=110)
            print(f"\nPNG: {args.plot}")
        except Exception as e:
            print(f"\n(画图失败：{e})")


if __name__ == "__main__":
    main()
