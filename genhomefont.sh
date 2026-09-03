#!/usr/bin/env bash
# Hasilkan DUA font LVGL dari Poppins Black untuk halaman utama desain "dua
# lingkaran" (lihat komentar C_HOME_BG di touch.ino):
#   font_home_big.c    digit-saja (0-9), 90px  -- jam besar "10"/"08"
#     (sempat 80px, diperbesar ke 90px atas permintaan lanjutan)
#   font_home_kecil.c  spasi+koma+digit+A-Z, 20px -- baris "SENIN, OKT 24"
#     (dibuat tebal atas permintaan eksplisit -- lv_font_montserrat_20
#     bawaan LVGL cuma satu ketebalan, tidak ada varian bold; sempat 26px,
#     diperkecil lagi ke 20px atas permintaan lanjutan)
# Beda dari genhomedigits.py (revisi sebelumnya, sudah tidak dipakai): jam
# sekarang PUTIH RATA, bukan bergradasi, jadi boleh balik jadi font LVGL
# biasa alih-alih bitmap.
#
# Perlu Node.js (npx) dan koneksi internet (lv_font_conv diunduh via npx,
# Poppins Black dari Google Fonts kalau /tmp/Poppins-Black.ttf belum ada).
#
# --size 90 (big) dipilih supaya box_h glyph ~66-70px, sesuai HOME_MENIT_Y-
# HOME_JAM_Y=76 di touch.ino. --size 20 (kecil)
# dipilih setelah dua kali revisi (26px lalu diperkecil) -- line_height~19.
# Kalau salah satu ukuran diganti, sesuaikan ulang geometri HOME_JAM_*/
# HOME_MENIT_Y/HOME_TANGGAL_Y di touch.ino juga.
set -e
FONT_TTF=/tmp/Poppins-Black.ttf
if [ ! -f "$FONT_TTF" ]; then
  curl -sL "https://github.com/google/fonts/raw/main/ofl/poppins/Poppins-Black.ttf" -o "$FONT_TTF"
fi

# lv_font_conv menulis include kondisional LV_LVGL_H_INCLUDE_SIMPLE --
# disamakan dengan gaya #include <lvgl.h> yang dipakai berkas font lain di
# direktori ini (font_digits_48.c dkk.).
rapikan_include() {
  sed -i '/#ifdef LV_LVGL_H_INCLUDE_SIMPLE/,/#endif/c\#include <lvgl.h>' "$1"
}

npx --yes lv_font_conv@1.5.3 --no-compress --no-prefilter --bpp 4 --size 90 \
  --font "$FONT_TTF" -r 0x30-0x39 \
  --format lvgl -o font_home_big.c --force-fast-kern-format
rapikan_include font_home_big.c

npx --yes lv_font_conv@1.5.3 --no-compress --no-prefilter --bpp 4 --size 20 \
  --font "$FONT_TTF" -r "0x20,0x2C,0x30-0x39,0x41-0x5A" \
  --format lvgl -o font_home_kecil.c --force-fast-kern-format
rapikan_include font_home_kecil.c

echo "selesai -> font_home_big.c, font_home_kecil.c"
