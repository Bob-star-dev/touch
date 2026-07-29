/*
 * Konfigurasi yang harus diisi sendiri.
 *
 * PENTING: isi ketiga nilai bertanda GANTI_ di bawah sebelum flash, kalau tidak
 * Wi-Fi dan cuaca tidak akan jalan (jam tetap jalan dari RTC).
 *
 * Jangan commit file ini ke repo publik: berisi password Wi-Fi dan API key.
 */
#pragma once

/* ---------------- Wi-Fi ---------------- */
#define WIFI_SSID       "asawatch"
#define WIFI_PASSWORD   "12345678"

/* ---------------- OpenWeatherMap ----------------
 * API key gratis: https://home.openweathermap.org/api_keys
 * OWM_CITY memakai format "Kota,KodeNegara". Alternatif pakai koordinat:
 * ganti OWM_QUERY di weather.cpp menjadi "lat=..&lon=..".
 */
#define OWM_API_KEY     "8b41c3dffb9f25e1a2a82cb011447f00"
/* HARUS "Surakarta,ID", jangan "Solo,ID". Keduanya menjawab HTTP 200, tapi
 * "Solo,ID" di basis data OWM menunjuk desa Solo di Flores (121.12, -8.80) --
 * sekitar 1300 km dari Solo/Surakarta Jawa Tengah (110.83, -7.56). Verifikasi
 * langsung: Solo,ID -> 19.04 C, Surakarta,ID -> 31.65 C.
 * Kalau ragu, pakai koordinat: ganti "q=" jadi "lat=-7.5561&lon=110.8317". */
#define OWM_CITY        "Surakarta,ID"

/* ---------------- Zona waktu ----------------
 * WIB = UTC+7. WITA = 8, WIT = 9. Indonesia tidak memakai DST.
 */
#define TZ_OFFSET_SEC   (7 * 3600)
#define TZ_DST_SEC      0

/* ---------------- Interval ---------------- */
#define WEATHER_PERIOD_MS   (30UL * 60UL * 1000UL)  /* cuaca: 30 menit       */
#define NTP_RESYNC_MS       (6UL * 3600UL * 1000UL) /* NTP ulang: 6 jam      */
#define RTC_RESYNC_MS       (60UL * 1000UL)         /* baca RTC: 1 menit     */
#define WIFI_RETRY_MS       (60UL * 1000UL)         /* coba sambung lagi     */

/* Backoff kalau permintaan cuaca gagal. WAJIB ada: tanpa jeda yang membesar,
 * kegagalan berulang (mis. key belum aktif -> HTTP 401) membuat board memanggil
 * API tiap siklus task (2 detik) = ~30 panggilan/menit, mendekati batas 60/menit
 * paket gratis dan berisiko membuat key kena rate-limit. */
#define WEATHER_RETRY_MIN_MS  (30UL * 1000UL)       /* jeda gagal pertama    */
#define WEATHER_RETRY_MAX_MS  (15UL * 60UL * 1000UL) /* batas atas backoff   */

/* ---------------- Baterai ----------------
 * GPIO0 adalah satu-satunya pin ADC1 yang tidak dipakai LCD (GPIO1..6 terpakai),
 * jadi di situlah pembagi tegangan baterai board ini.
 *
 * BATT_DIVIDER = Vbaterai / Vpin. Nilai 2.0 berarti pembagi 1:2 (mis. 100k/100k).
 * KALIBRASI: lihat angka "raw" di baris [batt] pada Serial, lalu ukur tegangan
 * baterai dengan multimeter. BATT_DIVIDER = Vukur / Vraw.
 */
#define BATT_ADC_PIN    0

/* Dikalibrasi memakai charger sebagai referensi tegangan.
 *
 * Nominal pembagi 200k/100k = 3.00, tapi itu mengabaikan toleransi resistor dan
 * offset ADC. Charger Li-Po CC/CV meregulasi ke 4.200 V (+-1%) di akhir
 * pengisian, jadi itu referensi fisik yang bisa dipakai: dengan pin membaca
 * 1414 mV saat pengisian selesai, rasio sebenarnya = 4200 / 1414 = 2.97.
 *
 * Kenapa 1% ini penting: rasio 3.00 memberi 4242 mV, 42 mV di atas 4.200 V.
 * Karena kurva memetakan apa pun >= 4200 mV menjadi 100%, kelebihan itu menelan
 * sekitar 4% penurunan pertama -- baterai mulai terkuras tapi angkanya diam di
 * 100%. Itulah sebabnya "selalu 100%" terlihat seperti sensor macet.
 *
 * Multimeter tetap acuan terbaik: BATT_DIVIDER = Vukur / raw yang tercetak. */
#define BATT_DIVIDER    2.97f

/* Tampilkan log diagnostik jam/cuaca/baterai di Serial. */
#define NET_DEBUG 1
