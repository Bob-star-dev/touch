#!/usr/bin/env python3
"""Hasilkan home_digits.h -- sepuluh bitmap digit '0'-'9' untuk jam besar
halaman utama (build_home() di touch.ino), gradasi ungu->sian, tata letak
dua baris meniru referensi "Screenshot from 2026-09-02 19-46-52.png" (watch
face Amazfit/Zepp bergaya "PAI").

RIWAYAT FONT:
1. Kontur tiga-garis bikinan sendiri (erosi lalu EDT scipy) di atas font
   Anton solid -- berulang kali "bolong" di device sungguhan, dibuang.
2. Noto Sans Bold solid biasa -- aman (tidak ada garis tipis sama sekali)
   tapi bentuknya tidak mirip referensi (referensi punya gaya "garis tiga
   sejajar", bukan isian penuh).
3. SEKARANG: "WOX-Striped Triple" (studiotypo) -- glyph-nya SENDIRI sudah
   berupa tiga garis sejajar bersarang (bukan sintesis kita), cocok dengan
   referensi TANPA perlu teknik citra apa pun. Diuji render-downscale-
   simulasi RGB565 di /tmp sebelum dipakai: pada FONT_SIZE=700 lalu
   diperkecil DUA LANGKAH (setengah dulu, baru ke ROW_H) garis-garisnya
   (~8px pada render awal) tetap utuh, tidak menyusut ke sub-piksel seperti
   kegagalan teknik #1. Lisensi: DEMO, "free for personal use" -- perlu
   lisensi komersial dari studiotypo.com sebelum distribusi di luar
   pengujian pribadi.
4. Sempat dicoba font "Stripe" oleh Stasia Popova (dafont, lisensi lebih
   longgar: Public domain/GPL/OFL) sebagai uji perbandingan atas permintaan
   eksplisit -- ternyata font tulisan-tangan/kursif dengan outline tipis
   dan arsir dekoratif, BUKAN gaya "garis sejajar", dan dinilai jelek di
   device sungguhan. Dibatalkan, kembali ke WOX-Striped Triple (poin 3).
5. Sempat dicoba ganti digit '1','3','4','6' (yang bentuk WOX-nya dinilai
   "aneh" dibanding referensi) dengan gambar tangan teknik "garis sejajar
   tiga lapis" manual (offset perpendikular, sambungan tangen matematis ke
   lingkaran untuk '6'/'3') -- setelah di-flash tetap dinilai "aneh" untuk
   SEMUA empat digit itu, jadi DIBATALKAN. Kembali ke WOX-Striped Triple
   murni untuk semua 0-9 (poin 3).
6. SEKARANG: referensi ganti total ("Screenshot from 2026-09-04 00-17-12.png"
   -- gaya pngtree, angka SOLID tebal + garis aksen magenta diagonal, BUKAN
   lagi gaya "garis tiga sejajar"). Garis aksennya sendiri BUKAN bagian
   font mana pun (elemen grafis terpisah yang ditempel di atas), jadi tidak
   diikutkan -- yang dicari cuma bentuk dasar angkanya. Dibandingkan
   berdampingan (lihat riwayat percakapan) melawan Fredoka/Baloo2/Quicksand/
   Nunito/Poppins -- Poppins Black (Google Fonts, OFL, bebas komersial)
   paling dekat: '0' oval-tinggi, ketebalan seragam, sudut tegas tapi
   sedikit membulat. Isian SOLID biasa (seperti Noto Sans dulu, poin 2) --
   tidak butuh downscale dua-langkah/pengukuran lebar garis seperti WOX
   karena tidak ada garis tipis yang bisa "bolong".

KENAPA BITMAP, BUKAN FONT LVGL:
LVGL 8.3 cuma mendukung SATU warna rata per label, tidak ada gradasi
per-glyph -- jadi tiap digit tetap digambar sebagai gambar RGB565 biasa
(sama seperti ic_detak dkk. di genassets.py) dan ditempel lewat mk_img(),
bukan mk_label().

Perlu Pillow saja. Font diunduh otomatis dari dafont.com kalau belum ada
cache lokal (FONT_CACHE).
"""
import os
import urllib.request

from PIL import Image, ImageDraw, ImageFont

FONT_CACHE = "/tmp/Poppins-Black.ttf"
FONT_URL = "https://github.com/google/fonts/raw/main/ofl/poppins/Poppins-Black.ttf"

# Diukur dari Screenshot from 2026-09-02 19-46-52.png (lihat riwayat
# percakapan): gradasi menerus dari ungu di baris jam ke sian di baris menit,
# BUKAN diulang per baris -- lihat colorize(), yang memakai posisi Y absolut
# di layar 240x280, bukan Y relatif tiap bitmap.
GRAD_ATAS  = (175, 120, 225)   # ungu, dekat y=0
GRAD_BAWAH = (70, 210, 220)    # sian, dekat y=280 -- HARUS sama dgn touch.ino
CANVAS_H = 280                 # HARUS sama dengan SCREEN_H di touch.ino
BG = (0, 0, 0)                  # HARUS sama dengan C_HOME_BG di touch.ino

FONT_SIZE = 260       # ukuran render awal, diperkecil ke ROW_H lewat LANCZOS
                       # sekali jalan -- aman karena isian solid tidak punya
                       # garis tipis yang bisa hilang saat downscale.

ROW_H = 112          # tinggi akhir tiap digit di layar -- HARUS sama dengan
                     # HOME_ROW_H di touch.ino
COL_W = 75           # HARUS sama dengan HOME_COL_W di touch.ino (lebar kolom,
                     # digit dipusatkan horizontal di dalamnya)

CHARS = "0123456789"
OUT = "/home/harjo/Documents/touch/home_digits.h"


def siapkan_font():
    if not os.path.exists(FONT_CACHE):
        print("mengunduh Poppins Black...")
        urllib.request.urlretrieve(FONT_URL, FONT_CACHE)
    return FONT_CACHE


def buat_masker(font, ch):
    """Masker L (0/255) berisi glyph diisi PENUH (solid), dipotong pas ke
    bounding-box-nya. Tidak butuh downscale dua-langkah seperti era WOX --
    Poppins Black isian solid, tidak ada garis tipis yang bisa sub-piksel/
    "bolong" saat diperkecil sekali jalan (lihat RIWAYAT FONT poin 6)."""
    l, t, r, b = font.getbbox(ch)
    pad = 6
    w, h = r - l + 2 * pad, b - t + 2 * pad
    dasar = Image.new("L", (w, h), 0)
    ImageDraw.Draw(dasar).text((pad - l, pad - t), ch, font=font, fill=255)
    return dasar


def colorize(mask, y_absolut):
    """Warnai masker L dengan gradasi ungu->sian berdasarkan posisi Y
    ABSOLUT di layar 240x280 -- lihat GRAD_ATAS/BAWAH -- supaya transisinya
    menerus lintas baris jam & menit, bukan diulang per digit."""
    w, h = mask.size
    grad = Image.new("RGB", (w, h))
    for y in range(h):
        frac = max(0.0, min(1.0, (y_absolut + y) / CANVAS_H))
        col = tuple(int(GRAD_ATAS[i] * (1 - frac) + GRAD_BAWAH[i] * frac) for i in range(3))
        row = Image.new("RGB", (w, 1), col)
        grad.paste(row, (0, y))
    out = Image.new("RGB", (w, h), BG)
    out.paste(grad, (0, 0), mask)
    return out


def rgb565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def emit(name, im):
    w, h = im.size
    px = list(im.getdata())
    body = []
    for (r, g, b) in px:
        v = rgb565(r, g, b)
        body.append("0x%02X,0x%02X," % (v & 0xFF, v >> 8))
    lines = ["  " + "".join(body[i:i + 12]) for i in range(0, len(body), 12)]
    return (
        f"/* {name}: {w}x{h}, {w*h*2} bytes */\n"
        f"static const uint8_t {name}_map[] = {{\n" + "\n".join(lines) + "\n};\n"
        f"static const lv_img_dsc_t {name} = {{\n"
        f"  {{ LV_IMG_CF_TRUE_COLOR, 0, 0, {w}, {h} }}, {w*h*2}, {name}_map\n}};\n"
    ), w * h * 2


if __name__ == "__main__":
    font_path = siapkan_font()
    font = ImageFont.truetype(font_path, FONT_SIZE)

    # Kedua baris dipakai untuk bitmap yang SAMA (home_digit_N tunggal per
    # digit, dipakai ulang di baris jam MAUPUN menit -- lihat HOME_DIGIT_IMG
    # di touch.ino) -- tapi warnanya sudah dibakar per posisi Y, jadi setiap
    # digit dibuat DUA VERSI (satu untuk baris atas, satu untuk baris bawah)
    # supaya gradasinya tetap benar di kedua baris.
    chunks, total = [], 0
    for baris, y_absolut in (("atas", 18), ("bawah", 163)):
        for ch in CHARS:
            masker = buat_masker(font, ch)
            # fit ke DUA dimensi (bukan cuma tinggi) -- sebagian glyph lebih
            # lebar dari kolom kalau diskalakan cuma berdasar tinggi.
            s = min(COL_W / masker.size[0], ROW_H / masker.size[1])
            neww = max(1, int(masker.size[0] * s))
            newh = max(1, int(masker.size[1] * s))
            masker = masker.resize((neww, newh), Image.LANCZOS)
            colored = colorize(masker, y_absolut)
            padded = Image.new("RGB", (COL_W, ROW_H), BG)
            padded.paste(colored, ((COL_W - neww) // 2, (ROW_H - newh) // 2))
            name = f"home_digit_{baris}_{ch}"
            c, n = emit(name, padded)
            chunks.append(c)
            total += n
            print(f"{name:20s} {COL_W}x{ROW_H}  {n:6d} B  (ink w={neww})")

    with open(OUT, "w") as f:
        f.write("/* Dibuat otomatis oleh genhomedigits.py -- jangan diedit manual,\n"
                "   jalankan ulang skripnya. Latar SUDAH dipanggang jadi hitam pekat\n"
                "   (C_HOME_BG); lebar SUDAH dipad ke HOME_COL_W dan ink dipusatkan\n"
                "   horizontal -- build_home() di touch.ino cuma menempel bitmap ini\n"
                "   di (HOME_COL_Xn, HOME_*_Y) tanpa perhitungan tambahan apa pun.\n"
                "   Ada DUA salinan tiap digit (_atas/_bawah) karena warnanya dibakar\n"
                "   per posisi Y absolut di layar -- lihat komentar di genhomedigits.py.\n"
                "   Font sumber & lisensinya: lihat RIWAYAT FONT di genhomedigits.py. */\n")
        f.write("#pragma once\n#include <lvgl.h>\n\n")
        f.write("\n".join(chunks))

    print(f"total {total} B -> {OUT}")
