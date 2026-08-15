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
    # Crop lama (80,150,140,210) berukuran 60x60 padahal hatinya hanya menempati
    # x 80..120, y 150..188 -- tepat menempel di sudut kiri-atas crop, menyisakan
    # 19 px latar di kanan dan 21 px di bawah. Karena aset ini TRUE_COLOR tanpa
    # alpha, padding itu ikut jadi bagian gambar: hati tergambar di kiri-atas
    # kotak 30x30 sehingga pusatnya jatuh di (45, 78.5) padahal pusat arc
    # (50, 84) -- meleset 5 px. Crop dirapatkan ke hati, ditambah 1 px di kanan
    # dan bawah supaya ukurannya genap (42x40 -> tepat 21x20 di layar) dan pusat
    # hati jatuh persis di pusat aset.
    ("img_heart_lg", "3.png", (80, 150, 122, 190), (21, 20)),
    ("img_diamond",  "1baru.png", (230, 358, 250, 378), (10, 10)),

    # --- Ikon kartu wajah jam baru ---
    # Diambil 1:1 dari wajah_jam_modular_240x280.png, yang memang sudah dibuat
    # pada ukuran layar sebenarnya -- jadi tidak ada penskalaan sama sekali dan
    # ikonnya setajam di desain. Kotak 16x18 dipakai untuk KEEMPATNYA walau
    # ikonnya sendiri berbeda ukuran (hati 12x11, tetes 10x12, segi enam 12x14,
    # meteran 16x13): dengan kotak seragam, keempat kartu memakai koordinat
    # tempel yang sama dan perbedaan ukuran ikon tidak bocor ke tata letak.
    #
    # Aset LVGL ini TRUE_COLOR tanpa alpha, jadi latar yang ikut terpotong
    # menjadi bagian gambar. Itu justru yang diinginkan di sini: latarnya persis
    # warna kartu (#171A21), dan kartu di layar memakai warna yang sama, jadi
    # potongannya melebur tanpa kotak yang terlihat.
    ("ic_detak",   "wajah_jam_modular_240x280.png", (20, 110, 36, 128), (16, 18)),
    ("ic_spo2",    "wajah_jam_modular_240x280.png", (134, 110, 150, 128), (16, 18)),
    ("ic_glukosa", "wajah_jam_modular_240x280.png", (20, 200, 36, 218), (16, 18)),
    ("ic_tekanan", "wajah_jam_modular_240x280.png", (134, 200, 150, 218), (16, 18)),
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
