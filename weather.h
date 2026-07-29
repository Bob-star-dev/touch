/*
 * Cuaca dari OpenWeatherMap.
 *
 * weather_fetch() melakukan HTTP blocking, jadi HANYA boleh dipanggil dari task
 * jaringan -- jangan dari loop()/timer LVGL, karena akan membekukan UI beberapa
 * detik. Hasilnya dititipkan lewat weather_get() yang aman dari thread lain.
 *
 * Teks kondisi sengaja dibatasi satu-dua kata pendek: label cuaca di header
 * hanya punya ruang 74 px pada montserrat_10 (x46..120, sebelum garis pemisah).
 * "CERAH BERAWAN" (95 px) dan "HUJAN RINGAN" (81 px) tidak muat, jadi tidak
 * dipakai. Semua string di weather.cpp sudah diukur <= 74 px.
 */
#pragma once

typedef struct {
  bool valid;        /* true kalau sudah pernah berhasil diambil */
  int  temp_c;       /* suhu dalam Celsius, sudah dibulatkan     */
  char cond[16];     /* kondisi, huruf besar, mis. "BERAWAN"     */
} weather_t;

/* Siapkan mutex. Panggil sekali sebelum task jaringan dijalankan. */
void weather_begin(void);

/* Ambil data dari OWM. Blocking. Hanya dari task jaringan.
 * Mengembalikan true kalau berhasil dan data sudah diperbarui. */
bool weather_fetch(void);

/* Salinan data terakhir. Aman dipanggil dari thread mana pun. */
void weather_get(weather_t *out);
