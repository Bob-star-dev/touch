#!/usr/bin/env python3
"""Ekstrak aset dari mockup 480x560 -> ui_assets.h (LVGL RGB565 TRUE_COLOR)."""
from PIL import Image

SRC = "/home/harjo/Documents/touch"
OUT = "/home/harjo/Documents/touch/ui_assets.h"

# name -> (file, crop box 2x, output size 1x)
ASSETS = [
    ("img_plane",    "1baru.png", (80, 100, 384, 344), (152, 122)),
    ("img_weather",  "1baru.png", (24,  10,  72,  50), (24, 20)),
    # Badan baterai mulai di x=394, bukan 405 -- crop lama memotong tepi kirinya.
    ("img_battery",  "1baru.png", (394, 16, 450,  42), (28, 13)),
    ("img_heart_sm", "2.png", (44, 134,  84, 172), (20, 19)),
    ("img_drop",     "2.png", (48, 272,  80, 316), (16, 22)),
    ("img_heart_lg", "3.png", (80, 150, 140, 210), (30, 30)),
    ("img_diamond",  "1baru.png", (230, 358, 250, 378), (10, 10)),
]


def rgb565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def emit(name, src, box, size):
    im = Image.open(f"{SRC}/{src}").convert("RGB").crop(box).resize(size, Image.LANCZOS)
    w, h = im.size
    px = list(im.getdata())
    body = []
    for i, (r, g, b) in enumerate(px):
        v = rgb565(r, g, b)
        body.append("0x%02X,0x%02X," % (v & 0xFF, v >> 8))  # little endian
    lines = []
    for i in range(0, len(body), 12):
        lines.append("  " + "".join(body[i:i + 12]))
    return (
        f"/* {name}: {w}x{h}, {w*h*2} bytes */\n"
        f"static const uint8_t {name}_map[] = {{\n" + "\n".join(lines) + "\n};\n"
        f"static const lv_img_dsc_t {name} = {{\n"
        f"  {{ LV_IMG_CF_TRUE_COLOR, 0, 0, {w}, {h} }}, {w*h*2}, {name}_map\n}};\n"
    ), w * h * 2


chunks, total = [], 0
for name, src, box, size in ASSETS:
    c, n = emit(name, src, box, size)
    chunks.append(c)
    total += n
    print(f"{name:14s} {size[0]}x{size[1]}  {n:6d} B")

with open(OUT, "w") as f:
    f.write("/* Dibuat otomatis dari mockup 1..5.png -- jangan diedit manual. */\n")
    f.write("#pragma once\n#include <lvgl.h>\n\n")
    f.write("\n".join(chunks))

print(f"total {total} B -> {OUT}")
