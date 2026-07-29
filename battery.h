/*
 * Baterai Li-Po 1-sel, dibaca lewat pembagi tegangan ke ADC1.
 *
 * Kenapa aman dibaca sambil Wi-Fi aktif:
 *   - Pin-nya ada di ADC1. Konflik ADC-vs-Wi-Fi yang terkenal itu hanya terjadi
 *     di ADC2 (dan itu pun khas ESP32 classic); ADC1 tetap bisa dipakai.
 *   - Yang benar-benar mengganggu justru fisiknya: burst transmit Wi-Fi menarik
 *     ratusan mA sekejap sehingga tegangan baterai ambles sesaat. Kalau dibaca
 *     sekali, hasilnya bisa lompat jauh. Karena itu dipakai MEDIAN dari
 *     beberapa sampel (menolak pencilan, bukan merata-ratakannya) lalu
 *     dihaluskan lagi dengan EMA.
 *
 * Semua fungsi dipanggil dari konteks loop(). ADC tidak memakai bus I2C, jadi
 * tidak mengganggu touch maupun RTC.
 */
#pragma once

/* Siapkan ADC. Panggil sekali di setup(). */
void battery_begin(void);

/* Ambil beberapa sampel dan perbarui nilai internal. Murah (~0.3 ms): jendela
 * median dibangun lintas pemanggilan lewat buffer cincin, jadi fungsi ini tidak
 * pernah memblokir lama dan aman dipanggil dari timer LVGL. */
void battery_update(void);

/* Kapasitas terkira, 0..100. */
int battery_percent(void);

/* Tegangan hasil penghalusan, dalam milivolt. */
int battery_millivolts(void);

/* Tegangan mentah di pin ADC (sebelum dikali rasio pembagi) -- untuk kalibrasi. */
int battery_raw_millivolts(void);

/* Selisih max-min dalam jendela sampel terakhir, di pin (mV). Ukuran seberapa
 * besar riak yang ditolak median -- berguna untuk menilai gangguan Wi-Fi. */
int battery_spread_mv(void);

/* Hitungan ADC mentah 0..4095. Nilai yang menempel di ~4095 berarti tegangan
 * di pin melebihi rentang ukur -- hasilnya mentok dan berhenti mengikuti
 * baterai, yang tampak seperti "selalu 100%". */
int battery_raw_counts(void);

/* Riwayat tegangan pin (mV), satu titik per menit, tertua dulu. Dipakai untuk
 * memeriksa apakah tegangan benar-benar turun saat board jalan dari baterai --
 * kondisi yang tidak bisa diamati lewat serial karena USB harus dicabut. */
void battery_history(char *buf, int n);
int  battery_history_count(void);

/* true kalau sudah ada pengukuran yang bisa dipakai. */
bool battery_valid(void);
