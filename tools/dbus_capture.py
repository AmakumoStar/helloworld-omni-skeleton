#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DBUS 原始流抓包分析（离线，只用标准库）

配套固件：Chassis/Task/component/comm_task.cpp 里的 `kDbusCapture`
          （USART3 用 ReceiveToIdle 切块，128 条记录写进 g_dbus_cap）

用法:
    python tools/dbus_capture.py --dump dbus_cap.bin
    python tools/dbus_capture.py --dump dbus_cap.bin --hexdump 128
    python tools/dbus_capture.py --dump dbus_cap.bin --json

它回答三个数（known_issues.md 里要求「确认之前不要再改解码逻辑」的那三个）：
    1. 一帧到底多少字节
    2. 帧头是什么（是不是 0x20 0x00）
    3. 帧间隔多少毫秒
并且直接给出「DT7::decode 应该从第几个字节开始读、帧长按多少切」的结论。

判据说明（为什么是这些数）：
    - 拨杆：DT7 协议里两个拨杆只可能是 1/2/3，落在 data[5]>>4 的高/低 2 位
    - 摇杆：11 bit 原始值 364..1684 对应 -1..+1（中位 1024）
    真实帧里这两个条件会同时满足；窗口没对齐时几乎必然违反。
"""

import argparse
import json
import struct
import sys
from collections import Counter


def _fix_console_encoding():
    """Windows 控制台默认不是 UTF-8，直接 print 中文会变乱码；按实际代码页输出"""
    try:
        import ctypes
        cp = ctypes.windll.kernel32.GetConsoleOutputCP()
        if cp and hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="cp%d" % cp, errors="replace")
    except Exception:
        pass

# ---------------------------------------------------------------- 内存布局
# 与 comm_task.cpp 的 DbusCapBlock 严格对应（改字段顺序要同步改这里）
MAGIC = 0xDB05CA90
OFF_MAGIC = 0
OFF_N = 4
OFF_N_TOTAL = 8
OFF_ARM_FAIL = 12
OFF_ENT = 16
ENTRY_SIZE = 72          # tick u32 + len u16 + data 64B + 2B 对齐
MAX_ENTRIES = 128
OFF_HIST = OFF_ENT + ENTRY_SIZE * MAX_ENTRIES   # 9232
HIST_LEN = 65
TOTAL_SIZE = OFF_HIST + HIST_LEN * 2            # 9362 -> 结构体对齐后 9364

# DT7 布局（与 Chassis/src/DT7.cpp 一致）
DT7_LEN = 18
CH_MIN, CH_MID, CH_MAX = 364, 1024, 1684


def u32(buf, off):
    return struct.unpack_from("<I", buf, off)[0]


def u16(buf, off):
    return struct.unpack_from("<H", buf, off)[0]


def find_base(buf):
    """magic 有可能因为符号找错而整体偏移，前 64 字节内扫一遍"""
    for base in range(0, 65, 4):
        if base + 4 <= len(buf) and u32(buf, base) == MAGIC:
            return base
    return None


def parse(buf, base):
    n = u32(buf, base + OFF_N)
    n_total = u32(buf, base + OFF_N_TOTAL)
    arm_fail = u32(buf, base + OFF_ARM_FAIL)
    n = min(n, MAX_ENTRIES)
    entries = []
    for i in range(n):
        off = base + OFF_ENT + i * ENTRY_SIZE
        if off + ENTRY_SIZE > len(buf):
            break
        tick = u32(buf, off)
        ln = u16(buf, off + 4)
        ln = min(ln, 64)
        data = bytes(buf[off + 6: off + 6 + ln])
        entries.append({"tick": tick, "len": ln, "data": data})
    hist = []
    for i in range(HIST_LEN):
        off = base + OFF_HIST + i * 2
        if off + 2 <= len(buf):
            hist.append(u16(buf, off))
        else:
            hist.append(0)
    return {"n": n, "n_total": n_total, "arm_fail": arm_fail,
            "entries": entries, "hist": hist}


# ---------------------------------------------------------------- 解码试算
def ch11(b, bit):
    """从字节流按 11 bit 取一个通道（小端位序，与 DT7.cpp 一致）"""
    idx = bit // 8
    shift = bit % 8
    if idx + 2 >= len(b):
        return None
    v = (b[idx] | (b[idx + 1] << 8) | (b[idx + 2] << 16)) >> shift
    return v & 0x07FF


def switches(b):
    """拨杆：DT7::decode 用 data[5] 的高 4 位"""
    if len(b) < 6:
        return None, None
    sw = b[5]
    left = ((sw >> 4) & 0x0C) >> 2
    right = (sw >> 4) & 0x03
    return left, right


def score_window(w):
    """一个 18 字节窗口的合法性：拨杆 1~3 + 四个摇杆都在量程内"""
    left, right = switches(w)
    ok_sw = (left in (1, 2, 3)) and (right in (1, 2, 3))
    chans = [ch11(w, 0), ch11(w, 11), ch11(w, 22), ch11(w, 33)]
    ok_ch = all(c is not None and CH_MIN <= c <= CH_MAX for c in chans)
    return ok_sw and ok_ch, left, right, chans


def evaluate(stream, frame_len, phase):
    """按 (帧长, 相位) 切帧，返回 (合法率, 采样帧, 前置字节)"""
    wins = []
    pos = phase
    while pos + DT7_LEN <= len(stream):
        wins.append(pos)
        pos += frame_len
    if not wins:
        return 0.0, [], b""
    good = 0
    sample = None
    for p in wins:
        ok, left, right, chans = score_window(stream[p: p + DT7_LEN])
        if ok:
            good += 1
            if sample is None:
                sample = {"left": left, "right": right, "ch": chans}
    # 窗口前面 2 字节（用来判断有没有 0x20 0x00 帧头）
    pre = bytes(stream[max(0, wins[0] - 2): wins[0]])
    return good / len(wins), sample, pre


def analyse(result, dump_name):
    ents = result["entries"]
    out = {"dump": dump_name, "n": result["n"], "n_total": result["n_total"],
           "arm_fail": result["arm_fail"], "entries": ents}
    if not ents:
        out["error"] = "没有记录：g_dbus_cap.n == 0，说明 ReceiveToIdle 没被触发过"
        return out

    lens = Counter(e["len"] for e in ents)
    out["len_hist"] = dict(lens)

    # 帧间隔（块到达时刻，单位 ms；只统计相邻差值为正的）
    ticks = [e["tick"] for e in ents]
    diffs = [b - a for a, b in zip(ticks, ticks[1:]) if b != a]
    diffs_sorted = sorted(diffs)
    if diffs_sorted:
        med = diffs_sorted[len(diffs_sorted) // 2]
        out["interval_ms"] = {"min": diffs_sorted[0], "median": med,
                              "max": diffs_sorted[-1], "n": len(diffs_sorted)}

    # 每条记录的起始字节（帧头线索）
    first2 = Counter(bytes(e["data"][:2]).hex(" ") for e in ents if e["len"] >= 2)
    out["first2"] = first2.most_common(5)

    # ① 先按「每个抓包块本身就是一帧」验证（ReceiveToIdle 的正确模型：
    #    空闲间隔已经把帧切开了，块与块之间隔着十几毫秒，拼成连续流反而是错的）
    mode_len, mode_cnt = lens.most_common(1)[0]
    out["per_block"] = None
    if mode_len >= DT7_LEN and mode_cnt >= 0.8 * len(ents):
        blocks = [e for e in ents if e["len"] == mode_len]
        best_off, best_ratio, best_smp = 0, 0.0, None
        for off in range(0, mode_len - DT7_LEN + 1):
            good = 0
            smp = None
            for e in blocks:
                ok, left, right, chans = score_window(e["data"][off:off + DT7_LEN])
                if ok:
                    good += 1
                    if smp is None:
                        smp = {"left": left, "right": right, "ch": chans}
            ratio = good / len(blocks)
            if ratio > best_ratio:
                best_off, best_ratio, best_smp = off, ratio, smp
        out["per_block"] = {"block_len": mode_len, "offset": best_off,
                            "ratio": round(best_ratio, 3), "sample": best_smp,
                            "blocks": len(blocks)}
    if out["per_block"] and out["per_block"]["ratio"] >= 0.9:
        # 已经用块内对齐验证通过，就不必再看连续流假设了
        out["candidates"] = []
        return out

    # ② 退一步：拼接成连续流，试各种 (帧长, 相位)
    stream = b"".join(e["data"] for e in ents)
    cand_len = sorted({mode_len, DT7_LEN, 20, 25, 26, 64})
    cand_len = [l for l in cand_len if 1 <= l <= 128]
    best = []
    for fl in cand_len:
        for ph in range(0, fl):
            ratio, sample, pre = evaluate(stream, fl, ph)
            if ratio > 0:
                best.append({"frame_len": fl, "phase": ph, "ratio": round(ratio, 3),
                             "pre_bytes": pre.hex(" "), "sample": sample})
    best.sort(key=lambda d: (-d["ratio"], d["frame_len"], d["phase"]))
    out["candidates"] = best[:6]
    return out


def report(out, hexdump=0):
    print("=" * 72)
    print("DBUS 抓包分析: %s" % out["dump"])
    print("=" * 72)
    if "error" in out:
        print("!! " + out["error"])
        return
    print("记录条数 n = %d / %d     总块数 n_total = %d     重挂失败 arm_fail = %d"
          % (out["n"], MAX_ENTRIES, out["n_total"], out["arm_fail"]))
    if out["arm_fail"]:
        print("!! arm_fail 非 0：ReceiveToIdle 重新挂载失败，数据可能有缺口")

    print("\n-- 块长分布（len: 次数）")
    for ln, cnt in sorted(out["len_hist"].items()):
        print("   %3d 字节 : %d" % (ln, cnt))
    lat = out["len_hist"]
    total_blocks = sum(lat.values())
    mode_len0, mode_cnt0 = max(lat.items(), key=lambda kv: kv[1])
    if len(lat) == 1:
        only = list(lat)[0]
        if only >= 64:
            print("   → 块长恒为上限 %d：没有出现空闲间隔（接收机是连续流，或波特率不对）" % only)
        else:
            print("   → 块长恒定 %d 字节 = 一帧的长度（空闲间隔把它切出来了）" % only)
    elif mode_cnt0 >= 0.8 * total_blocks:
        print("   → 主流块长 %d 字节（占 %d/%d）：其余多半是上电瞬间被切开的不完整块，正常"
              % (mode_len0, mode_cnt0, total_blocks))
    else:
        print("   → 块长不唯一：帧内有 >2ms 的空隙，或存在丢字节/粘帧，看下面的候选表")

    if "interval_ms" in out:
        iv = out["interval_ms"]
        print("\n-- 块间隔 (ms): min=%d 中位=%d max=%d（样本 %d）"
              % (iv["min"], iv["median"], iv["max"], iv["n"]))
        if iv["median"]:
            print("   → 约 %.1f 帧/秒" % (1000.0 / iv["median"]))

    print("\n-- 每条记录的前 2 字节（出现最多的组合）")
    for hx, cnt in out["first2"]:
        print("   %-8s : %d" % (hx, cnt))

    pb = out.get("per_block")
    if pb and pb["ratio"] >= 0.9:
        s = pb["sample"] or {}
        print("\n-- 按「每个抓包块 = 一帧」验证（ReceiveToIdle 的正确模型）")
        print("   块长 %d 字节，块内偏移 %d 起是 DT7 的 data[0]，合法率 %.0f%%（%d 块）"
              % (pb["block_len"], pb["offset"], pb["ratio"] * 100, pb["blocks"]))
        print("   拨杆 L/R=%s/%s   摇杆=%s" % (s.get("left"), s.get("right"), s.get("ch")))
        if pb["offset"] == 0 and pb["block_len"] == DT7_LEN:
            print("\n==> 结论：一帧就是 18 字节、没有帧头，DT7::decode 直接用接收缓冲即可（偏移 0）。")
            print("    只要用 ReceiveToIdle 按空闲切块，每块就是一整帧 —— 不需要任何偏移/搜索。")
        else:
            print("\n==> 结论：每块 = 一帧，帧长 %d 字节，解码从块内第 %d 字节开始"
                  % (pb["block_len"], pb["offset"]))
            print("    （多出的 %d 字节是帧头/尾部填充，接收时跳过即可）"
                  % (pb["block_len"] - DT7_LEN))
        return

    print("\n-- 解码候选（frame_len=切帧长度, phase=连续流里第几个字节当 DT7 的 data[0]）")
    if not out["candidates"]:
        print("   （无候选：没有任何窗口能同时满足拨杆 1~3 与摇杆量程）")
    for c in out["candidates"]:
        s = c["sample"] or {}
        print("   frame_len=%-3d phase=%-2d 合法率=%.0f%%  窗口前 2 字节=%-6s  "
              "拨杆 L/R=%s/%s  摇杆=%s"
              % (c["frame_len"], c["phase"], c["ratio"] * 100, c["pre_bytes"] or "--",
                 s.get("left"), s.get("right"), s.get("ch")))
    if out["candidates"] and out["candidates"][0]["ratio"] >= 0.9:
        c = out["candidates"][0]
        print("\n==> 结论：帧长 %d 字节，DT7::decode 的 data[0] 应指向连续流的第 %d 字节"
              "（即每帧跳过前 %d 字节）" % (c["frame_len"], c["phase"], c["phase"]))
        if c["phase"] == 0:
            print("    按 18 字节定长接收即可，不需要偏移。")
        else:
            print("    需要把接收缓冲/指针整体后移 %d 字节，并把 IsDbusFrameValid 与" % c["phase"])
            print("    DT7::decode 的取数位置同步改（三处必须一起改，只改一处必错）。")
    elif out["candidates"]:
        print("\n==> 没有 >=90% 的候选：数据里没有稳定的 18 字节帧结构，"
              "先确认波特率/极性/线序，或把 --hexdump 打出来人工看一眼。")

    if hexdump and out["entries"]:
        print("\n-- 前两条记录原始字节")
        for i, e in enumerate(out["entries"][:2]):
            print("   [%d] tick=%d len=%d" % (i, e["tick"], e["len"]))
            data = e["data"]
            for k in range(0, len(data), 16):
                chunk = data[k:k + 16]
                print("       %04x  %-47s  %s" % (k, chunk.hex(" "),
                                                  "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)))


def main():
    _fix_console_encoding()
    ap = argparse.ArgumentParser(description="DBUS 抓包离线分析")
    ap.add_argument("--dump", required=True, help="J-Link savebin 导出的 bin 文件")
    ap.add_argument("--hexdump", type=int, default=0, help="额外打印前 N 字节（0=不打印）")
    ap.add_argument("--json", action="store_true", help="输出 JSON（便于存档比对）")
    args = ap.parse_args()

    with open(args.dump, "rb") as f:
        buf = f.read()
    if len(buf) < TOTAL_SIZE:
        print("!! 文件只有 %d 字节，少于结构体 %d 字节：savebin 长度可能写错了"
              % (len(buf), TOTAL_SIZE))
    base = find_base(buf)
    if base is None:
        print("!! 没在文件头部找到 magic 0x%08X：符号地址找错了，或者抓包块没被初始化"
              % MAGIC)
        print("   文件前 32 字节: %s" % buf[:32].hex(" "))
        return 2
    if base != 0:
        print("!! magic 出现在偏移 %d，已自动对齐（符号地址多了 %d 字节）" % (base, base))

    result = parse(buf, base)
    out = analyse(result, args.dump)
    if args.json:
        printable = dict(out)
        printable["entries"] = [
            {"tick": e["tick"], "len": e["len"], "data": e["data"].hex()} for e in out["entries"]
        ]
        print(json.dumps(printable, ensure_ascii=False, indent=2))
    else:
        report(out, args.hexdump)
    return 0


if __name__ == "__main__":
    sys.exit(main())
