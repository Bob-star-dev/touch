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
  TIME_SRC_NTP,        /* sudah disinkronkan lewat NTP        */
  TIME_SRC_BLE         /* dari jam HP lewat ANCHOR_WAKTU      */
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

/* Setel jam dari epoch UTC yang dikirim HP (opcode ANCHOR_WAKTU, dokumen 2.2).
 * Offset zona waktu TZ_OFFSET_SEC ditambahkan di sini, jadi pemanggil cukup
 * meneruskan epoch UTC apa adanya seperti yang ada di paket.
 *
 * KONTEKS LOOP saja: fungsi ini menulis RTC lewat I2C. Aman dari
 * jalankan_perintah() di aw_jam.cpp (dijalankan di loop), TIDAK aman dari
 * callback NimBLE.
 *
 * Sumbernya diperlakukan setara NTP -- bukan lebih rendah. HP menyinkronkan
 * jamnya sendiri ke jaringan operator, dan pada jam tangan yang dipakai di luar
 * rumah, HP itu jauh lebih sering ada daripada Wi-Fi rumah. Menolak epoch dari
 * HP demi menjaga hasil NTP yang mungkin sudah berjam-jam lalu justru
 * mempertahankan waktu yang lebih tua. */
void tm_terapkan_epoch_utc(uint32_t epoch_utc);

/* Ringkasan satu baris tentang asal waktu SAAT BOOT (sebelum NTP menimpanya).
 * Disimpan karena print di setup() hampir selalu hilang: USB CDC board ini
 * re-enumerate setelah reset. Cetak ini dari heartbeat supaya selalu terbaca. */
const char *tm_boot_info(void);

/* Singkatan hari/bulan bahasa Inggris tiga huruf ("Wed", "Jan"), persis seperti
 * yang dipakai baris tanggal di wajah jam. Sengaja sudah dalam bentuk singkat --
 * bukan nama penuh yang dipotong pemanggil -- supaya bentuk yang tampil di layar
 * ditentukan di satu tempat saja, bukan oleh "%.3s" yang tersebar di pemanggil. */
const char *tm_day_name(int wday);        /* 0 = Sunday  */
const char *tm_month_name(int mon_0_11);  /* 0 = January */
