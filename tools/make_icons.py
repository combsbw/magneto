#!/usr/bin/env python3
"""Draw the Magneto icons (a pancake coil seen face-on) straight to PNG - no image libraries.

Writes icons/icon-192.png and icons/apple-touch-icon.png (180x180). Rerun after changing the art.
"""
import math
import os
import struct
import zlib

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
out_dir = os.path.join(root, "icons")
os.makedirs(out_dir, exist_ok=True)

BG = (14, 17, 22)          # #0e1116
RING_A = (90, 169, 255)    # outer rings, cool blue
RING_B = (73, 224, 184)    # inner rings, teal
CORE = (255, 196, 90)      # washer at the winding center


def lerp(a, b, t):
    return tuple(int(a[i] + (b[i] - a[i]) * t + 0.5) for i in range(3))


def draw(size):
    px = bytearray(size * size * 4)
    s = float(size)
    cx = cy = s / 2.0
    corner = s * 0.22
    rings = [(0.80, RING_A), (0.655, lerp(RING_A, RING_B, 0.33)), (0.51, lerp(RING_A, RING_B, 0.66)), (0.365, RING_B)]
    ring_w = s * 0.052
    core_r = s * 0.115

    for y in range(size):
        for x in range(size):
            # rounded-square alpha mask
            dx = max(abs(x - cx) - (s / 2 - corner), 0.0)
            dy = max(abs(y - cy) - (s / 2 - corner), 0.0)
            if math.hypot(dx, dy) > corner - 0.5:
                continue
            r = math.hypot(x - cx, y - cy)
            col = BG
            for frac, ring_col in rings:
                rr = frac * s / 2
                d = abs(r - rr)
                if d < ring_w:
                    edge = min(1.0, (ring_w - d) / (s * 0.012))
                    col = lerp(BG, ring_col, edge)
                    break
            if r < core_r:
                edge = min(1.0, (core_r - r) / (s * 0.015))
                col = lerp(BG, CORE, edge)
            i = (y * size + x) * 4
            px[i] = col[0]
            px[i + 1] = col[1]
            px[i + 2] = col[2]
            px[i + 3] = 255
    return px


def write_png(path, size, px):
    raw = b"".join(b"\x00" + bytes(px[y * size * 4:(y + 1) * size * 4]) for y in range(size))

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 6, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 9)))
        f.write(chunk(b"IEND", b""))


for name, size in (("icon-192.png", 192), ("apple-touch-icon.png", 180)):
    write_png(os.path.join(out_dir, name), size, draw(size))
    print("icons/%s" % name)
