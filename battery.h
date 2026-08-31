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

/* Tegangan terendah selama 3 menit terakhir -- inilah dasar persen, bukan nilai
 * sesaat. Sebabnya: perubahan beban (radio Wi-Fi menyala/mati) menggeser
 * tegangan ~53 mV di board ini, cukup memindahkan angka 4% sehingga persen
 * naik-turun mengikuti radio, bukan mengikuti daya. Karena siklus Wi-Fi ~60 s,
 * setiap jendela 3 menit hampir pasti memuat satu sag, jadi nilainya stabil dan
 * selalu berarti hal yang sama: tegangan saat berbeban. */
int battery_floor_mv(void);

/* true kalau jam sedang tersambung ke charger.
 *
 * Digabung dari tiga aturan yang masing-masing buta di tempat berbeda, dan
 * saling menambal justru di titik butanya (rinciannya di battery.cpp):
 *   1. langkah >=50 mV dalam 4 detik  -> kabel dicolok/dicabut, seketika
 *   2. sag terhadap langkah beban     -> memegang fase CV, saat tegangan rata
 *   3. tren naik 3 menit              -> memegang fase CC, saat sag menyesatkan
 *
 * Versi sebelumnya hanya punya aturan 3, dan itu sebabnya indikatornya "kadang
 * jalan kadang tidak": mengisi dari kondisi hampir penuh langsung masuk fase CV
 * dan tidak pernah terdeteksi sama sekali. */
bool battery_charging(void);

/* Sag terakhir yang terukur (mV di sisi baterai): berapa tegangan naik saat
 * beban dilepas. Kecil = sumber teregulasi (charger), besar = sel yang ambles.
 * battery_sag_valid() false sampai langkah beban pertama terjadi, dan dijadikan
 * false lagi setiap kali deteksi langkah menggantikan keputusannya. Keduanya
 * untuk diagnostik; keputusannya sendiri sudah ada di battery_charging(). */
int  battery_sag_mv(void);
bool battery_sag_valid(void);

/* Diberitahukan TEPAT SEBELUM beban besar berubah (backlight menyala/padam),
 * dengan jadi_berat=true kalau beban akan naik. Inilah yang memicu pengukuran
 * sag -- jam tidak pernah membuat langkah bebannya sendiri, ia hanya menumpang
 * pada mati/nyala layar yang toh sudah terjadi. Murah (~3 ms) dan tidak
 * memblokir menunggu hasil: battery_update() yang menyelesaikannya. */
void battery_beban_akan_berubah(bool jadi_berat);

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

/* Bacaan SEKETIKA, di luar jalur median/EMA/jendela-minimum yang biasa.
 *
 * Ada karena probe sag ("apakah sedang dicolok?") menanyakan hal yang berbeda
 * dari yang dijawab battery_millivolts(): bukan "berapa tegangan yang stabil
 * selama menit-menit terakhir" melainkan "berapa tegangan TEPAT SEKARANG, pada
 * beban yang persis sedang berlaku". Seluruh penghalusan di modul ini justru
 * dirancang untuk membunuh perbedaan sesaat semacam itu -- termasuk perbedaan
 * yang sedang kita cari -- jadi probe harus lewat jalur sendiri.
 *
 * TIDAK menyentuh keadaan internal apa pun (ring median, EMA, jendela 3 menit),
 * jadi memanggilnya tidak menggeser persen yang sedang tampil.
 *
 * Memblokir ~3 ms untuk 31 sampel berurutan -- sepuluh kali anggaran
 * battery_update(), jadi jangan dipanggil tiap siklus. Sesekali tidak apa-apa:
 * probe sag memanggilnya dua kali per mati/nyala layar, dan layar yang baru
 * berganti keadaan toh sedang menggambar ulang seluruh piksel.
 *
 * Nilai kembali dalam mV di sisi BATERAI (sudah dikali BATT_DIVIDER).
 * sebaran_pin_mv, kalau bukan NULL, diisi selisih max-min di sisi PIN -- itu
 * lantai derau pengukuran ini, jadi selisih sag yang lebih kecil darinya tidak
 * berarti apa-apa. */
int battery_baca_langsung_mv(int *sebaran_pin_mv);

/* Lama pembacaan ADC terakhir milik probe sag, mikrodetik. Diagnostik: ini
 * satu-satunya kode baterai yang memblokir lebih dari seribu mikrodetik. */
uint32_t battery_probe_us(void);
