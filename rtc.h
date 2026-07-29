/*
 * PCF85063 (RTC on-board Waveshare ESP32-C6-Touch-LCD-1.69, I2C 0x51).
 *
 * PENTING soal bus I2C: RTC berbagi bus dengan touch CST816T, dan driver touch
 * itu sensitif waktu -- register finger-count-nya kosong hampir seketika setelah
 * IRQ. Karena itu SEMUA fungsi di file ini hanya boleh dipanggil dari konteks
 * loop()/timer LVGL, thread yang sama dengan touch_poll(). Jangan dipanggil dari
 * task jaringan, supaya tidak ada dua transaksi I2C yang saling menyela.
 */
#pragma once

#include <time.h>

/* Deteksi + inisialisasi. Mengembalikan false kalau chip tidak menjawab. */
bool rtc_begin(void);

/* true kalau rtc_begin() berhasil. */
bool rtc_ok(void);

/* Baca waktu. Isi struct tm (tm_year sudah -1900, tm_mon sudah 0-11).
 * false kalau RTC tidak ada atau isinya belum masuk akal (mis. tahun < 2020,
 * tandanya RTC belum pernah diset / baterai cadangan habis). */
bool rtc_read(struct tm *out);

/* Tulis waktu ke RTC (dipakai setelah sinkronisasi NTP berhasil). */
bool rtc_write(const struct tm *in);

/* Pindai bus I2C dan cetak alamat yang menjawab -- untuk diagnosa. */
void rtc_scan_bus(void);
