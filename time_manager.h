/*
 * Pemilik tunggal "jam sekarang".
 *
 * Cara kerja:
 *   boot        -> dibaca dari PCF85063 (RTC jadi sumber utama)
 *   NTP sukses  -> waktu dikoreksi DAN ditulis balik ke RTC
 *   antar detik -> diinterpolasi dari millis(), RTC dibaca ulang tiap 1 menit
 *
 * Interpolasi millis() dipakai supaya bus I2C tidak diakses setiap detik --
 * bus itu dipakai bersama touch CST816T yang sensitif waktu. Drift millis()
 * dalam 1 menit hanya beberapa milidetik, jauh di bawah resolusi tampilan.
 *
 * Waktu disimpan sebagai "epoch lokal" (epoch yang sudah digeser ke zona waktu
 * TZ_OFFSET_SEC), lalu dipecah dengan gmtime_r. Pasangan timegm/gmtime ini
 * konsisten sendiri, jadi tidak perlu mengatur variabel TZ global.
 */
#pragma once

#include <time.h>

typedef enum {
  TIME_SRC_NONE = 0,   /* belum ada waktu yang bisa dipercaya */
  TIME_SRC_RTC,        /* dari PCF85063                       */
  TIME_SRC_NTP         /* sudah disinkronkan lewat NTP        */
} time_src_t;

/* Panggil sekali di setup(), SETELAH rtc_begin(). Konteks loop. */
void tm_begin(void);

/* Panggil rutin dari konteks loop/timer LVGL (murah, aman dipanggil sering).
 * Di sinilah RTC dibaca ulang dan hasil NTP yang tertunda diterapkan. */
void tm_tick(void);

/* Waktu sekarang. false kalau belum ada sumber yang valid. */
bool tm_now(struct tm *out);

/* true kalau tm_now() bisa dipercaya. */
bool tm_valid(void);

/* Asal waktu yang sedang dipakai. */
time_src_t tm_source(void);

/* Dipanggil task jaringan setelah NTP berhasil -- AMAN dari thread lain.
 * Hanya menitipkan nilai; penulisan ke RTC dilakukan tm_tick() di konteks loop
 * supaya akses I2C tetap satu thread. */
void tm_submit_ntp(const struct tm *local_time);

/* Ringkasan satu baris tentang asal waktu SAAT BOOT (sebelum NTP menimpanya).
 * Disimpan karena print di setup() hampir selalu hilang: USB CDC board ini
 * re-enumerate setelah reset. Cetak ini dari heartbeat supaya selalu terbaca. */
const char *tm_boot_info(void);

/* Nama hari/bulan bahasa Indonesia, huruf besar. */
const char *tm_day_name(int wday);        /* 0 = Minggu */
const char *tm_month_name(int mon_0_11);  /* 0 = Januari */
