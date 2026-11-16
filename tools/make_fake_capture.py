# 生成一个假的 DBUS 抓包 dump，用来验证 tools/dbus_capture.py 的解析与结论是否自洽
#
# 用法：
#   python tools/make_fake_capture.py out.bin              # 20 字节帧：0x20 0x00 + 18 字节 DT7 体
#   python tools/make_fake_capture.py out.bin ""           # 18 字节帧：无帧头
#   python tools/make_fake_capture.py out.bin "20 00" 7    # 指定帧头字节与帧间隔(ms)
import struct
import sys

MAGIC = 0xDB05CA90
ENTRY = 72
N = 128


def pack_bits(vals):
    """vals: [(bitpos, value, bitcount)]，LSB-first 打包成 18 字节（同 DT7 位序）"""
    b = bytearray(18)
    for pos, v, nbits in vals:
        for k in range(nbits):
            if (v >> k) & 1:
                bit = pos + k
                b[bit // 8] |= 1 << (bit % 8)
    return b


def main():
    out = sys.argv[1]
    header = bytes.fromhex(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2] else b""
    gap = int(sys.argv[3]) if len(sys.argv) > 3 else 14

    # 四个摇杆中位 1024；left=3、right=1；拨轮 1024
    payload = pack_bits([(0, 1024, 11), (11, 1024, 11), (22, 1024, 11),
                         (33, 1024, 11), (44, 1, 2), (46, 3, 2),
                         (96, 0, 8), (104, 0, 8), (112, 0, 16), (128, 1024, 11)])
    frame = header + bytes(payload)

    buf = bytearray(16 + ENTRY * N + 65 * 2)
    struct.pack_into("<I", buf, 0, MAGIC)
    struct.pack_into("<I", buf, 4, N)
    struct.pack_into("<I", buf, 8, N + 37)
    struct.pack_into("<I", buf, 12, 0)
    for i in range(N):
        off = 16 + i * ENTRY
        struct.pack_into("<I", buf, off, 100 + i * gap)
        struct.pack_into("<H", buf, off + 4, len(frame))
        buf[off + 6: off + 6 + len(frame)] = frame
    struct.pack_into("<H", buf, 16 + ENTRY * N + len(frame) * 2, N)  # len_hist

    with open(out, "wb") as f:
        f.write(buf)
    print("wrote %s, frame=%d bytes (header=%r), gap=%d ms"
          % (out, len(frame), header.hex(" ") or "none", gap))


if __name__ == "__main__":
    main()
