/*
 * Wi-Fi + task latar yang menjalankan NTP dan pengambilan cuaca.
 *
 * Kenapa ada modul keempat di luar rtc/weather/time_manager: NTP dan cuaca
 * SAMA-SAMA butuh Wi-Fi. Menaruh pengelolaan Wi-Fi di time_manager membuat
 * weather bergantung padanya (dan sebaliknya), jadi lapisan koneksi dipisah
 * sendiri di sini. Isinya tetap tipis.
 *
 * Pembagian thread -- ini yang menjaga UI tetap responsif:
 *
 *   task jaringan (di sini)      loop()/timer LVGL
 *   -------------------------    -------------------------------
 *   WiFi.begin, NTP, HTTP GET    lv_timer_handler, touch_poll
 *   (blocking, tapi yield)       akses I2C: touch + RTC
 *   TIDAK menyentuh I2C          TIDAK melakukan operasi jaringan
 *   TIDAK menyentuh LVGL         satu-satunya yang menulis label
 *
 * LVGL tidak thread-safe dan bus I2C dipakai bersama touch, jadi task ini
 * hanya menitipkan data lewat weather_get() / tm_submit_ntp(). Operasi blocking
 * di sini aman karena semuanya menunggu di socket atau vTaskDelay, yang
 * menyerahkan CPU ke scheduler sehingga loop() tetap jalan.
 */
#pragma once

/* Buat task jaringan. Panggil sekali di setup(), paling akhir. */
void net_begin(void);

/* Status koneksi terakhir. */
bool net_connected(void);

/* true kalau NTP pernah berhasil sejak boot. */
bool net_ntp_done(void);
