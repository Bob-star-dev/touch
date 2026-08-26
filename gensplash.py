#!/usr/bin/env python3
"""Ekstrak logo AsaWatch -> splash_assets.h (LVGL RGB565 TRUE_COLOR).

Dipisah dari genassets.py dengan sengaja. genassets.py adalah pemotong: ia
mengambil kotak piksel dari mockup yang latarnya SUDAH berwarna kartu, jadi
tidak ada yang perlu diubah selain ukuran. Berkas logo di sini kebalikannya --
tinta hijau gelap di atas kertas PUTIH -- dan ditempel apa adanya ke layar
gelap ia akan tampil sebagai kotak putih menyilaukan. Jadi jalurnya bukan
"potong lalu kecilkan" melainkan "angkat tintanya, terangkan, tempel ke gelap",
dan mencampur dua jalur itu di satu berkas hanya akan mengaburkan keduanya.

TIGA LANGKAH, dan alasan masing-masing:

1. ALFA DARI JARAK KE PUTIH, bukan dari kecerahan.
   Godaan pertama adalah alpha = 1 - luminance: putih hilang, hitam pekat.
   Itu keliru di logo ini, karena garis hati justru berwarna hijau MUDA
   (#7FB3A0, luminance tinggi) -- ia akan ikut hilang bersama kertasnya dan
   yang tersisa cuma badan jam. Yang membedakan tinta dari kertas di sini
   bukan gelap-terangnya melainkan seberapa jauh dari putih murni, jadi itulah
   yang diukur.

2. TERANGKAN NILAINYA, PERTAHANKAN RONANYA.
   Seluruh tinta logo duduk di V 0.28..0.66 (diukur, bukan ditaksir). Di atas
   latar mendekati hitam, V 0.28 nyaris tak terbaca. V dipetakan ulang ke
   0.45..1.0 sementara H dan S dibiarkan -- jadi hijau tetap hijau dan
   perbedaan antara badan jam, garis hati, dan garis EKG tetap terjaga.
   Menggantinya dengan satu warna rata akan membuat ketiganya melebur.

3. LATAR DIPANGGANG, bukan alfa.
   LVGL punya TRUE_COLOR_ALPHA, tapi harganya 3 byte/piksel dan pencampuran
   per piksel setiap frame. Splash ini digambar di layar berlatar SATU warna
   rata (SPL_BG di touch.ino), jadi latar itu bisa ikut dipanggang ke dalam
   gambar: hasilnya 2 byte/piksel, blit polos, dan tepi kotaknya tidak terlihat
   karena warnanya identik dengan layar. Syaratnya cuma satu, dan itu mengikat:
   SPL_BG di touch.ino harus sama persis dengan BG di bawah.
"""
import colorsys

from PIL import Image

SRC = "/home/harjo/Documents/touch/IMG-20260821-WA0004.jpg"
OUT = "/home/harjo/Documents/touch/splash_assets.h"

# Harus identik dengan SPL_BG di touch.ino. Lihat langkah 3 di atas.
BG = (0x09, 0x0A, 0x10)

# Ambang "sudah bukan kertas lagi", dalam satuan jarak ke putih (0..255).
# 55 dipilih dari histogram berkas ini: derau JPEG di area kertas berhenti di
# sekitar 12, dan piksel logo paling pucat (tepi anti-alias garis hati) ada di
# 70-an. Ambang di tengah keduanya membuang derau tanpa menggerogoti tepi.
AMBANG = 55.0

V_DASAR = 0.45   # V terendah yang boleh tampil di layar gelap
V_RENTANG = 0.55  # sisanya diisi V asli, jadi 0..1 -> 0.45..1.00

# nama -> (kotak potong pada berkas sumber, ukuran keluaran)
# Kotak diukur otomatis dari profil baris berkas sumber (tiga pita: tanda jam,
# kata "ASAWatch", anak judul), bukan ditebak dengan mata.
ASET = [
    ("spl_mark", (567, 52, 1034, 593), (90, 104)),
    ("spl_kata", (263, 611, 1337, 754), (190, 25)),
]


def angkat(px):
    """Satu piksel kertas-putih -> piksel siap tempel di atas BG."""
    r, g, b = px[0] / 255.0, px[1] / 255.0, px[2] / 255.0
    alfa = min(1.0, (255 - min(px)) / AMBANG)
    if alfa <= 0.0:
        return BG
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    v = V_DASAR + V_RENTANG * v
    if v > 1.0:
        v = 1.0
    r, g, b = colorsys.hsv_to_rgb(h, s, v)
    return tuple(
        int(round(BG[i] * (1.0 - alfa) + c * 255.0 * alfa))
        for i, c in enumerate((r, g, b))
    )


def rgb565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def emit(nama, kotak, ukuran):
    im = Image.open(SRC).convert("RGB").crop(kotak)
    # Diangkat SEBELUM diperkecil, supaya penghalusan LANCZOS mencampur warna
    # akhir. Urutan terbalik akan mencampur tinta dengan kertas putih dulu, lalu
    # mengangkat campuran itu -- tepi hurufnya jadi berhalo terang.
    im.putdata([angkat(p) for p in im.getdata()])
    im = im.resize(ukuran, Image.LANCZOS)
    w, h = im.size
    body = []
    for r, g, b in im.getdata():
        v = rgb565(r, g, b)
        body.append("0x%02X,0x%02X," % (v & 0xFF, v >> 8))  # little endian
    lines = ["  " + "".join(body[i:i + 12]) for i in range(0, len(body), 12)]
    return (
        f"/* {nama}: {w}x{h}, {w * h * 2} bytes */\n"
        f"static const uint8_t {nama}_map[] = {{\n" + "\n".join(lines) + "\n};\n"
        f"static const lv_img_dsc_t {nama} = {{\n"
        f"  {{ LV_IMG_CF_TRUE_COLOR, 0, 0, {w}, {h} }}, {w * h * 2}, {nama}_map\n}};\n"
    ), w * h * 2, im


if __name__ == "__main__":
    potongan, total = [], 0
    for nama, kotak, ukuran in ASET:
        c, n, im = emit(nama, kotak, ukuran)
        potongan.append(c)
        total += n
        im.save(f"/tmp/claude-1000/-home-harjo-Documents-touch/{nama}.png")
        print(f"{nama:10s} {ukuran[0]}x{ukuran[1]}  {n:6d} B")

    with open(OUT, "w") as f:
        f.write("/* Dibuat otomatis oleh gensplash.py dari IMG-20260821-WA0004.jpg.\n"
                "   Jangan diedit manual -- jalankan ulang skripnya.\n"
                "   Latar SUDAH dipanggang ke dalam gambar: warnanya harus sama\n"
                "   persis dengan SPL_BG di touch.ino, kalau tidak kotak gambarnya\n"
                "   akan terlihat sebagai persegi yang lebih terang. */\n")
        f.write("#pragma once\n#include <lvgl.h>\n\n")
        f.write("\n".join(potongan))

    print(f"total {total} B -> {OUT}")
