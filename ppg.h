/*
 * MAX30105/MAX30102 (PPG) -> BPM, SpO2, dan estimasi glukosa + tekanan darah
 * EKSPERIMENTAL.
 *
 * Diadaptasi dari sketch ESP32-C3 standalone. Yang HARUS berubah untuk board
 * ini dan alasannya -- ketiganya bukan preferensi gaya, tapi hal yang membuat
 * kode aslinya tidak bisa dipakai di sini:
 *
 *  1. Bus I2C 100 kHz, BUKAN 400 kHz. Sketch asli memanggil
 *     begin(Wire, I2C_SPEED_FAST) dan begin() itu melakukan setClock() -- jadi
 *     ia akan menaikkan bus jadi 400 kHz. Touch CST816T di board ini tidak
 *     stabil di 400 kHz (lihat catatan di touch.ino), jadi sentuhan akan rusak.
 *
 *  2. Pembacaan non-blocking. getRed()/getIR() memanggil safeCheck(250) yang
 *     MENUNGGU sampai 250 ms per pemanggilan -- dua pemanggilan per sampel
 *     berarti loop bisa berhenti sampai setengah detik. LVGL akan membeku
 *     total. Di sini dipakai check()/available()/getFIFO*()/nextSample() yang
 *     hanya mengambil apa yang sudah ada di FIFO.
 *
 *  3. Tidak ada while(1) saat sensor tak terdeteksi. Sketch asli menggantung
 *     selamanya; di jam tangan itu berarti seluruh UI mati hanya karena satu
 *     sensor tidak terpasang. Di sini kegagalan dilaporkan lalu diabaikan.
 *
 * Semua fungsi dipanggil dari konteks loop() -- sama seperti touch dan RTC,
 * supaya tidak ada dua transaksi I2C yang saling menyela.
 *
 * PERINGATAN MEDIS (dipertahankan dari sketch asli):
 * Nilai glukosa TIDAK punya dasar fisiologis tervalidasi. Tidak ada metode PPG
 * non-invasif berbasis Red/IR yang tervalidasi klinis untuk glukosa pada
 * perangkat konsumen. Angkanya adalah model linear placeholder atas fitur
 * sinyal (PI, R, HR) yang harus dikalibrasi sendiri, dan bahkan setelah
 * dikalibrasi akurasinya tidak bisa dijamin. JANGAN dipakai untuk keputusan
 * medis apa pun, termasuk dosis obat atau insulin.
 */
#pragma once

typedef enum {
  PPG_ABSENT = 0,     /* sensor tidak terdeteksi di I2C          */
  PPG_OFF,            /* sengaja dimatikan lewat tombol daya     */
  PPG_NO_CONTACT,     /* tidak ada kulit menempel                */
  PPG_SETTLING,       /* filter DC sedang konvergen              */
  PPG_ACQUIRING,      /* mencari detak yang konsisten            */
  PPG_STABLE          /* bacaan sudah bisa dipakai               */
} ppg_state_t;

typedef struct {
  ppg_state_t state;

  bool  bpm_valid;
  float bpm;

  bool  spo2_valid;
  float spo2;         /* sudah termasuk koreksi bias situs wrist */

  bool  glu_valid;
  float glucose;      /* EKSPERIMENTAL -- lihat peringatan di atas */

  /* Tekanan darah, EKSPERIMENTAL -- sama sekali belum tervalidasi, bahkan
   * lebih mentah daripada model glukosa (lihat SBP0.. di ppg.cpp). Diturunkan
   * dari fitur PPG yang sama (PI, HR, R) karena sensor ini satu-satunya
   * sumber data -- tidak ada jalur EKG untuk pulse-transit-time yang biasanya
   * dipakai perangkat medis. JANGAN dipakai untuk keputusan medis apa pun. */
  bool  bp_valid;
  float sbp;           /* sistol, mmHg, EKSPERIMENTAL */
  float dbp;           /* diastol, mmHg, EKSPERIMENTAL */

  float pi;           /* perfusion index IR (%), fitur kalibrasi   */
  float r;            /* rasio R, fitur kalibrasi                  */

  /* Jumlah detak yang sudah diterima sejak kulit menempel, tanpa batas atas.
   * Berbeda dari gerbang internal MIN_STABLE_BEATS yang berhenti di 3: itu
   * hanya menjawab "sudah ada detak konsisten?", sedangkan angka ini menjawab
   * "sudah berapa banyak?". Dipakai rutin pengukuran untuk menentukan kapan
   * sebuah pengukuran layak diakhiri -- BPM dari 3 detak masih sangat mudah
   * bergeser puluhan bpm oleh satu detak yang salah dibaca. Kembali 0 setiap
   * kali kontak hilang. */
  uint16_t beats;

  /* Statistik sepanjang sesi kontak berjalan. Dipakai untuk chip di halaman
   * detail supaya angka di bawah tidak dikarang saat angka utamanya nyata. */
  int   bpm_min, bpm_avg, bpm_max;
  int   spo2_min, spo2_avg;
  int   glu_min, glu_max;
  int   sbp_min, sbp_avg, sbp_max;
  int   dbp_min, dbp_avg, dbp_max;
  bool  stats_valid;

  /* true = angka ini salinan hasil terakhir, bukan bacaan langsung (sensor
   * sudah diangkat). Nilainya tetap valid untuk ditampilkan; penandanya ada
   * supaya UI bisa membedakan "sedang mengukur" dari "hasil terakhir". */
  bool     held;
  uint32_t hold_age_ms;   /* usia salinan dalam ms; 0 kalau bacaan langsung */
} ppg_data_t;

/* Deteksi + konfigurasi sensor. Panggil setelah Wire.begin().
 * false = sensor tidak ada; modul lalu diam saja dan UI tetap jalan. */
bool ppg_begin(void);

/* Kuras FIFO dan proses sampel. Murah dan non-blocking; panggil dari loop().
 * Sampling internal 100 Hz, FIFO 32 sampel, jadi dipanggil tiap <=20 ms
 * sudah cukup jauh dari risiko overflow. */
void ppg_update(void);

/* Hasil terkini. Selama kulit menempel isinya bacaan langsung; setelah sensor
 * diangkat isinya salinan hasil terakhir dengan held=true, sehingga angka di
 * layar tidak hilang. */
void ppg_get(ppg_data_t *out);

/* Hapus salinan hasil terakhir sehingga UI kembali menampilkan "--".
 * Dipanggil dari luar saat layar dimatikan. Pengukuran baru (kulit menempel
 * lagi) sudah menghapusnya sendiri, jadi tidak perlu dipanggil untuk itu. */
void ppg_clear_hold(void);

/* true kalau sensor terdeteksi saat ppg_begin(). */
bool ppg_present(void);

/* ---- Daya sensor (hemat baterai) ----------------------------------------
 * MAX30105 menyalakan dua LED (Red + IR) pada arus penuh 100 Hz terus-menerus
 * begitu dikonfigurasi -- puluhan mA, dan tetap mengalir walau tidak ada kulit
 * menempel. Itu beban yang sia-sia selama pengguna hanya melihat jam.
 *
 * ppg_set_enabled(false) menulis bit SHUTDOWN di register MODECONFIG (datasheet
 * hal. 19): LED padam dan ADC berhenti, chip turun ke ~0.7 uA tapi tetap
 * menjawab I2C sehingga tidak perlu di-probe ulang. Konfigurasi register
 * dipertahankan, jadi menyalakannya kembali cukup satu bit -- tidak ada
 * sensor.setup() ulang.
 *
 * Saat mati ppg_update() tidak melakukan transaksi I2C sama sekali dan seluruh
 * hasil (termasuk salinan hasil terakhir) dihapus, sehingga UI kembali ke "--".
 * Angka lama yang menggantung saat pengukuran dimatikan akan menyesatkan.
 *
 * Keadaan awal setelah ppg_begin() adalah MATI. Pengukuran baru dimulai saat
 * pengguna menekan tombol daya di halaman home.
 *
 * Kalau chip tidak terdeteksi, kedua fungsi ini tetap mencatat niat pengguna
 * (tanpa menyentuh perangkat keras) supaya tombol di UI tetap berfungsi. */
void ppg_set_enabled(bool on);
bool ppg_enabled(void);

/* Nilai mentah terakhir + jumlah sampel yang sudah diproses, untuk membedakan
 * "tidak ada jari" dari "FIFO tidak jalan" dan "ambang terlalu tinggi".
 * Argumen mana pun boleh NULL. */
void ppg_diag(long *ir, long *red, uint32_t *samples, long *threshold,
              uint32_t *polls);

/* Teks status singkat untuk log/diagnostik. */
const char *ppg_state_text(void);
