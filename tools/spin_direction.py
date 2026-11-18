#!/usr/bin/env python3
"""小陀螺"平移走不直"的定量判读（配合 tools/jlink.ps1 -Action dump-trace）。

背景：操作手反馈「小陀螺时平移方向跟着晃、走不直」。方向由
    世界方向 = θ_车 + kFrameDelta() = IMU yaw + (δ − g_frame_lead·ω)
决定。本脚本把这三项分开算，判断抖动来自
    A. 坐标系角（δ 或 ω 噪声被 g_frame_lead 放大）—— 命令侧
    B. 轮速跟不上（vel vs tgt 的幅值/相位）
    C. 9 Hz 纹波（轮速频谱）

用法：
    python tools/spin_direction.py --dump spin_trace.bin [--plot dir.png]
    python tools/spin_direction.py --dump spin_trace.bin --lead 0.19 --lever 0.30676

只读离线数据，不参与控制。numpy 即可。
"""

import argparse
import os
import sys

import numpy as np

WHEELS = ["LF", "LB", "RB", "RF"]
NCOLS = 21
COL_T, COL_MODE, COL_WCMD, COL_DELTA = 0, 1, 2, 3
COL_TGT, COL_VEL, COL_OUT, COL_JIT, COL_IMU = 4, 8, 12, 16, 20
DT = 0.01          # 100 Hz


def load(path):
    raw = np.fromfile(path, dtype="<f4")
    if raw.size == 0:
        sys.exit(f"{path} 是空的：先跑 jlink.ps1 -Action dump-trace")
    if raw.size % NCOLS:
        sys.exit(f"文件长度 {raw.size} 不是 {NCOLS} 的整数倍：dump 地址/大小不对")
    d = raw.reshape(-1, NCOLS)
    # 丢掉全 0 的行（未录满时的尾巴）
    keep = np.any(d != 0.0, axis=1)
    return d[keep]


def unwrap(x):
    return np.unwrap(x)


def smooth(x, n):
    if n <= 1:
        return x
    k = np.ones(n) / n
    return np.convolve(x, k, mode="same")


def lead_from_omega(omega, tau, lim=0.6):
    return np.clip(tau * omega, -lim, lim)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", default="spin_trace.bin")
    ap.add_argument("--lead", type=float, default=0.19, help="g_frame_lead (s)")
    ap.add_argument("--lever", type=float, default=0.30676, help="(半轴距+半轮距) m")
    ap.add_argument("--radius", type=float, default=0.07786, help="轮半径 m")
    ap.add_argument("--plot", default="")
    args = ap.parse_args()

    if not os.path.exists(args.dump):
        sys.exit(f"找不到 {args.dump}")

    d = load(args.dump)
    n = d.shape[0]
    t = d[:, COL_T] / 1000.0
    w_cmd = d[:, COL_WCMD]
    delta = d[:, COL_DELTA]
    tgt = d[:, COL_TGT:COL_TGT + 4]
    vel = d[:, COL_VEL:COL_VEL + 4]
    outs = d[:, COL_OUT:COL_OUT + 4]
    imu = unwrap(d[:, COL_IMU])

    # 实测自旋角速度：IMU yaw 差分。固件里 yawRate() 有 10 ms 低通，
    # 这里用 5 拍(50 ms)滑动近似，避免差分噪声把"超前补偿"算成抖动。
    omega = smooth(np.gradient(imu, DT), 5)
    lead = lead_from_omega(omega, args.lead)
    frame = delta - lead
    world_dir = imu + frame

    # 纯自旋时四轮 |v| = ω·lever/R（rad/s）；平移会让四轮出现"共模"分量
    spin_only = np.abs(w_cmd) * args.lever / args.radius
    common = np.abs(np.mean(tgt, axis=1))
    moving = (np.abs(w_cmd) > 0.15) & (common > 0.25 * np.maximum(spin_only, 0.2))

    print("=" * 78)
    print(f"小陀螺平移方向判读   {args.dump}   {n} 行 / {n * DT:.2f} s")
    print("=" * 78)

    if moving.sum() < 50:
        print(f"\n⚠ 可用的『自旋+平移』样本只有 {int(moving.sum())} 个（{moving.sum() * DT:.2f} s），")
        print("  不足以判读。请重录：拔掉 SWD → 小陀螺中速 → 推到前并保持 4~5 s。")
        return

    def stat(name, x):
        x = x[np.isfinite(x)]
        p2p = float(np.ptp(x))
        print(f"  {name:<26} std={np.std(x):7.4f}  ptp={p2p:7.4f}  "
              f"({np.degrees(np.std(x)):5.2f}° / {np.degrees(p2p):5.2f}°)")

    print(f"\n[1] 方向抖动（{int(moving.sum())} 样本 / {moving.sum() * DT:.2f} s）")
    stat("世界方向 imu+δ−lead·ω", world_dir[moving])
    stat("  ├ 分量：δ", delta[moving])
    stat("  ├ 分量：lead·ω", lead[moving])
    stat("  └ 分量：ω (rad/s)", omega[moving])
    stat("IMU yaw", imu[moving])
    # 纯 δ 抖动（不加补偿）作对照：说明补偿是"放大"还是"抑制"了抖动
    stat("对照：imu+δ（无补偿）", (imu + delta)[moving])

    print("\n[2] 轮速跟随（自旋+平移段）")
    err = vel - tgt
    for i, w in enumerate(WHEELS):
        e = err[moving, i]
        print(f"  {w}: 目标均值 {np.mean(tgt[moving, i]):7.3f}  实际 {np.mean(vel[moving, i]):7.3f}"
              f"  误差均值 {np.mean(e):+7.3f}  std {np.std(e):6.3f}  |out| max {np.max(np.abs(outs[moving, i])):6.0f}")

    print("\n[3] 9 Hz 纹波（四轮实际速度频谱，0.5~20 Hz）")
    x = vel[moving, 0] - np.mean(vel[moving, 0])
    if x.size > 64:
        sp = np.abs(np.fft.rfft(x * np.hanning(x.size)))
        fr = np.fft.rfftfreq(x.size, DT)
        band = (fr >= 0.5) & (fr <= 20)
        if band.any():
            k = np.argmax(sp[band])
            fpk = fr[band][k]
            share = sp[band][k] / (np.sum(sp[band]) + 1e-9)
            print(f"  主峰 {fpk:5.2f} Hz（占该频段 {share * 100:4.1f}%）"
                  f"{'  ← 落在 ~9 Hz' if 7.5 <= fpk <= 10.5 else ''}")
            idx = np.argsort(sp[band])[::-1][:3]
            tops = ", ".join(f"{fr[band][i]:.1f}Hz" for i in idx)
            print(f"  前三峰：{tops}")
    else:
        print("  样本太少，跳过")

    print("\n[4] 判读")
    s_dir = np.degrees(np.std(world_dir[moving]))
    s_raw = np.degrees(np.std((imu + delta)[moving]))
    if s_dir >= 3.0:
        print(f"  ✗ 世界方向 std = {s_dir:.1f}°，明显在晃 → 优先查坐标系侧")
    elif s_dir >= 1.0:
        print(f"  ~ 世界方向 std = {s_dir:.1f}°，轻微晃动")
    else:
        print(f"  ✓ 世界方向 std = {s_dir:.1f}°，命令侧是干净的 → 抖动来自轮子/地面")
    print(f"    （补偿前 std={s_raw:.1f}°；补偿把抖动从 {s_raw:.1f}° 变成 {s_dir:.1f}°）")

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
            ax[0].plot(t, np.degrees(world_dir), label="world dir")
            ax[0].plot(t, np.degrees(imu + delta), label="no comp", alpha=0.6)
            ax[0].set_ylabel("deg"); ax[0].legend(); ax[0].grid(alpha=.3)
            ax[1].plot(t, w_cmd, label="w_cmd")
            ax[1].plot(t, omega, label="w_meas")
            ax[1].set_ylabel("rad/s"); ax[1].legend(); ax[1].grid(alpha=.3)
            for i in range(4):
                ax[2].plot(t, tgt[:, i], "--", lw=.8)
                ax[2].plot(t, vel[:, i], lw=1.0)
            ax[2].set_ylabel("wheel rad/s"); ax[2].grid(alpha=.3)
            ax[2].set_xlabel("t (s)")
            ax[2].axvspan(t[moving][0] if moving.any() else 0,
                          t[moving][-1] if moving.any() else 0, alpha=.1)
            fig.tight_layout()
            fig.savefig(args.plot, dpi=110)
            print(f"\nPNG: {args.plot}")
        except Exception as e:  # matplotlib 缺了也不影响数值判读
            print(f"\n(画图失败：{e})")


if __name__ == "__main__":
    main()
