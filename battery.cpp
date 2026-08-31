#include <Arduino.h>
#include "battery.h"
#include "config.h"

/* ---- median dari N sampel ----
 * Median dipilih, bukan rata-rata: burst transmit Wi-Fi membuat tegangan ambles
 * beberapa milidetik. Rata-rata akan ikut tertarik turun oleh pencilan itu,
 * median mengabaikannya sama sekali.
 */
#define SAMPLES 15

static int  s_raw_mv   = 0;    /* di pin ADC, sebelum dikali rasio */
static int  s_batt_mv  = 0;    /* setelah rasio + EMA             */
static int  s_percent  = 0;
static bool s_valid    = false;
static int  s_spread   = 0;    /* max-min jendela sampel terakhir */
static int  s_counts   = 0;    /* hitungan ADC mentah 0..4095 -- deteksi saturasi */

/* ---- dasar perhitungan persen: minimum jendela bergerak ----
 *
 * Median di atas hanya mampu menolak pencilan yang LEBIH PENDEK dari jendelanya
 * (~2.5 detik). Sag akibat radio Wi-Fi bukan pencilan pendek: ia bertahan
 * selama seluruh upaya sambung, jadi seluruh jendela berada di dalam satu
 * keadaan beban dan medianya ikut pindah.
 *
 * Terukur di board ini: radio aktif -> 1365 counts, radio mati -> 1382 counts.
 * Selisih 17 counts = 18 mV di pin = 53 mV di baterai = sekitar 4% pada kurva
 * di dekat penuh. Kalau persen mengikuti nilai sesaat, angkanya naik-turun 4%
 * mengikuti radio -- bukan mengikuti daya.
 *
 * Versi sebelumnya memakai floor ABADI (terendah yang pernah teramati). Itu
 * cacat: floor terpasang di 4143 mV saat sel hampir penuh, sementara pemulihan
 * butuh kenaikan +100 mV = 4243 mV -- di atas batas fisik Li-Po 4200 mV. Jadi
 * angkanya terpaku permanen dan tidak akan pernah bisa naik lagi.
 *
 * Sekarang dasarnya minimum selama 3 menit terakhir. Karena siklus retry Wi-Fi
 * ~60 detik, setiap jendela hampir pasti memuat satu sag, sehingga angkanya
 * stabil DAN konsisten (selalu "tegangan saat berbeban"). Bedanya dari floor
 * abadi: jendela ini ikut bergerak NAIK saat baterai benar-benar diisi.
 */
#define WMIN_SLOTS      36            /* 36 x 5 s = 3 menit */
#define WMIN_SLOT_MS    5000UL
static uint16_t s_wmin[WMIN_SLOTS];
static int      s_wmin_n = 0, s_wmin_i = 0;
static int      s_slot_min = 0;       /* minimum slot 5 s yang sedang berjalan */
static uint32_t s_slot_ms  = 0;
static int      s_base_mv  = 0;       /* minimum jendela -> dasar persen */
static bool     s_charging = false;

/* ---- deteksi "sedang dicolok": tiga aturan yang saling menambal ----
 *
 * Ambang di bawah bukan tebakan; semuanya dari log probe di board ini
 * (perintah konsol `sag`/`saglog` di touch.ino, yang tetap ada untuk mengukur
 * ulang kalau selnya diganti):
 *
 *   dicolok  t=22..62s   SAG = -2, +0, +0        berat 4104..4105 mV
 *   baterai  t=82..182s  SAG = +6,+10,+9,+9,+13  berat 4001..3967 mV
 *   dicolok  t=202s      SAG = +1                berat 4104 mV
 *
 * Jarak antara dua gugus itu 5 mV dengan pengulangan +-2 mV. Sempit dalam
 * angka absolut, tetapi konsisten -- dan yang menanggung beban keputusan
 * sehari-hari sebenarnya deteksi langkah 100 mV, bukan sag ini. */
#define STEP_SLOTS       8      /* 8 x ~500 ms = 4 detik  */
#define STEP_MV          50     /* langkah colok/cabut terukur >100 mV */
#define SAG_COLOK_MAX    3      /* <= ini: sumber teregulasi -> dicolok */
#define SAG_BATERAI_MIN  5      /* >= ini: sel ambles      -> di baterai */
#define PROBE_SETTLE_MS  300

static uint16_t s_step[STEP_SLOTS];
static int      s_step_n = 0, s_step_i = 0;

static int      s_probe_v0     = 0;
static uint32_t s_probe_ms     = 0;
static bool     s_probe_nunggu = false;
static bool     s_probe_ke_berat = false;
static int      s_sag_mv       = 0;
static bool     s_sag_ada      = false;
static uint32_t s_probe_us     = 0;
static uint32_t s_tren_sah_ms  = 0;   /* tren dibisukan sampai 3 mnt setelah ini */
static uint32_t s_step_naik_ms = 0;   /* langkah-naik terakhir; menggerbang sag besar */

/* Acuan sejak nyala. Tanpa ini tidak ada cara membedakan dua sebab yang sama
 * sekali berbeda ketika persen tidak bergerak:
 *   - tegangan memang belum turun (kurva Li-Po sangat datar di dekat penuh,
 *     dan 10 menit pada sel 1000 mAh hanya sekitar 2%)
 *   - tegangan turun tapi angkanya tertelan plafon 4200 mV = 100% karena
 *     BATT_DIVIDER terlalu tinggi
 * Selisih dalam mV memperlihatkan keduanya, jauh sebelum persen bergerak. */
static int      s_first_mv = 0;
static uint32_t s_first_ms = 0;

/* Kurva pelepasan Li-Po 1 sel. Hubungan tegangan-kapasitas jauh dari linear --
 * peta linear 3.3-4.2 V akan salah besar di tengah rentang. Titik-titik ini
 * diinterpolasi linear di antaranya. */
typedef struct { int mv; int pct; } curve_pt_t;
static const curve_pt_t CURVE[] = {
  { 4200, 100 }, { 4100, 92 }, { 4000, 85 }, { 3950, 78 },
  { 3900,  70 }, { 3850, 62 }, { 3800, 55 }, { 3750, 47 },
  { 3700,  40 }, { 3650, 33 }, { 3600, 25 }, { 3550, 18 },
  { 3500,  12 }, { 3450,  8 }, { 3400,  5 }, { 3300,  2 },
  { 3000,   0 },
};
static const int CURVE_N = sizeof(CURVE) / sizeof(CURVE[0]);

static int mv_to_percent(int mv) {
  if (mv >= CURVE[0].mv) return 100;
  if (mv <= CURVE[CURVE_N - 1].mv) return 0;
  for (int i = 0; i < CURVE_N - 1; i++) {
    if (mv <= CURVE[i].mv && mv > CURVE[i + 1].mv) {
      int dv = CURVE[i].mv  - CURVE[i + 1].mv;
      int dp = CURVE[i].pct - CURVE[i + 1].pct;
      return CURVE[i + 1].pct + ((mv - CURVE[i + 1].mv) * dp + dv / 2) / dv;
    }
  }
  return 0;
}

static int cmp_int(const void *a, const void *b) {
  return (*(const int *)a) - (*(const int *)b);
}

void battery_begin(void) {
  /* Atenuasi penuh supaya rentang ukur mencapai ~3.3 V: dengan pembagi 1:2
   * baterai 4.2 V muncul sebagai 2.1 V di pin. */
  analogSetPinAttenuation(BATT_ADC_PIN, ADC_11db);
  pinMode(BATT_ADC_PIN, INPUT);
}

/* Buffer cincin: jendela median dibangun LINTAS pemanggilan, bukan di dalam
 * satu pemanggilan. Versi pertama kode ini mengambil 15 sampel berjarak 2 ms
 * sekaligus -- 30 ms memblokir, setara satu periode refresh LVGL penuh, dan
 * cukup untuk menunda pembacaan touch yang datanya cepat hilang. Sekarang tiap
 * pemanggilan hanya ambil 3 sampel berurutan (~0.3 ms) lalu isi cincin, jadi
 * jendelanya justru jadi lebih panjang (~2.5 detik pada laju 500 ms) yang
 * malah lebih baik untuk menolak burst transmit Wi-Fi yang berskala milidetik. */
static int  s_ring[SAMPLES];
static int  s_ring_n = 0;
static int  s_ring_i = 0;

/* ---- riwayat 1 menit/sampel, disimpan di RAM ----
 * Serial tidak bisa dibaca saat board jalan dari baterai (USB dicabut), jadi
 * board mencatat sendiri. Begitu USB dicolok lagi board TIDAK reset -- ia hanya
 * mulai mengisi -- sehingga riwayat ini masih utuh dan bisa dibaca. Inilah satu-
 * satunya cara melihat apakah tegangan benar-benar turun saat memakai baterai. */
/* 30 menit. Cukup untuk tes pelepasan: cabut USB, pakai 20-30 menit, colok lagi
 * (board TIDAK reset, hanya mulai mengisi) lalu baca riwayatnya. Itu satu-satunya
 * cara melihat tegangan saat benar-benar jalan dari baterai, karena serial hanya
 * hidup saat USB tertancap -- dan saat USB tertancap sel sedang diisi. */
#define HIST_N 30
static uint16_t s_hist[HIST_N];
static int      s_hist_n = 0;
static int      s_hist_i = 0;
static uint32_t s_hist_ms = 0;

void battery_update(void) {
  s_counts = analogRead(BATT_ADC_PIN);   /* mentah: 4095 = saturasi */
  for (int k = 0; k < 3; k++) {
    s_ring[s_ring_i] = analogReadMilliVolts(BATT_ADC_PIN);  /* dikalibrasi eFuse */
    s_ring_i = (s_ring_i + 1) % SAMPLES;
    if (s_ring_n < SAMPLES) s_ring_n++;
  }

  int s[SAMPLES];
  memcpy(s, s_ring, s_ring_n * sizeof(int));
  qsort(s, s_ring_n, sizeof(int), cmp_int);
  s_raw_mv = s[s_ring_n / 2];                    /* median */
  s_spread = s[s_ring_n - 1] - s[0];             /* seberapa berisik jendelanya */

  int mv = (int)(s_raw_mv * BATT_DIVIDER + 0.5f);

  if (!s_valid) {
    s_batt_mv  = mv;                             /* pengukuran pertama langsung */
    s_valid    = true;
    s_slot_min = mv;
    s_slot_ms  = millis();
    s_base_mv  = mv;
  } else {
    /* EMA simetris alpha 0.2. Tidak perlu asimetris lagi: yang menahan artefak
     * beban sekarang minimum jendela di bawah, bukan penghalusan ini. */
    s_batt_mv = (s_batt_mv * 8 + mv * 2) / 10;
  }

  /* ---- minimum jendela bergerak 3 menit -> dasar persen ---- */
  if (s_batt_mv < s_slot_min) s_slot_min = s_batt_mv;

  if ((uint32_t)(millis() - s_slot_ms) >= WMIN_SLOT_MS) {
    s_slot_ms = millis();
    s_wmin[s_wmin_i] = (uint16_t)s_slot_min;
    s_wmin_i = (s_wmin_i + 1) % WMIN_SLOTS;
    if (s_wmin_n < WMIN_SLOTS) s_wmin_n++;
    s_slot_min = s_batt_mv;                      /* mulai slot berikutnya */
  }

  /* Minimum atas seluruh slot penuh + slot berjalan. */
  int wmin = s_slot_min;
  for (int k = 0; k < s_wmin_n; k++)
    if (s_wmin[k] < wmin) wmin = s_wmin[k];

  /* ================= Apakah jam sedang dicolok? =================
   * Tiga aturan, dijalankan dari yang paling lemah ke yang paling kuat supaya
   * yang kuat menang dalam pemanggilan yang sama.
   *
   * Versi pertama menjalankannya terbalik dan itu bug yang nyata: aturan tren
   * dievaluasi TERAKHIR, jadi sesaat setelah kabel dicabut, deteksi langkah
   * memang menyetel false -- lalu tren yang masih memuat kenaikan selama
   * mengisi tadi langsung menyetelnya true lagi di baris berikutnya. Ikonnya
   * macet menyala sampai jendela 3 menit itu habis, dan tiap probe sag yang
   * datang di sela itu pun ditimpa lagi. Gejalanya persis "stuck di kondisi
   * charge terus". */

  /* ---- 1. Tren naik 3 menit: memegang fase CC ----
   * HANYA boleh menyalakan, tidak pernah memadamkan. Ia cuma benar di satu
   * fase: saat mengisi dari sel kosong, charger bekerja sebagai sumber ARUS,
   * jadi langkah beban tetap menghasilkan sag persis seperti di baterai dan
   * aturan 2 akan salah menyimpulkan "tidak mengisi". Yang tidak bisa
   * dipalsukan di fase itu adalah tegangan yang naik terus.
   *
   * Kebalikannya tidak berlaku -- tegangan yang tidak naik bukan bukti tidak
   * mengisi (di CV memang tidak naik) -- karena itu tidak ada cabang else.
   *
   * s_tren_sah_ms adalah penawar kemacetan di atas: setiap langkah membisukan
   * tren selama satu panjang jendela penuh, supaya ia tidak pernah menjawab
   * berdasarkan slot yang terkumpul SEBELUM keadaan dayanya berubah. Sengaja
   * memakai stempel waktu, bukan mengosongkan s_wmin: jendela itu juga dasar
   * perhitungan persen, dan persen tidak ada urusannya dengan ini. */
  if (s_wmin_n >= WMIN_SLOTS &&
      (uint32_t)(millis() - s_tren_sah_ms) >= (uint32_t)WMIN_SLOTS * WMIN_SLOT_MS) {
    int oldest = s_wmin[s_wmin_i];               /* slot berikut = yang tertua */
    int newest = s_wmin[(s_wmin_i + WMIN_SLOTS - 1) % WMIN_SLOTS];
    if (newest - oldest >= 20) s_charging = true;
  }

  /* ---- 2. Probe sag: memegang fase CV ----
   * Di sinilah tren buta total: saat sel hampir penuh charger meregulasi di
   * 4,2 V dan tegangannya RATA, jadi tidak ada tren untuk dilihat. Yang tetap
   * berbeda adalah tanggapannya terhadap beban -- charger menahan, baterai
   * ambles. Terukur di board ini dengan backlight sebagai beban: dicolok
   * -2..+1 mV, di baterai +6..+13 mV.
   *
   * Jendela 300 ms cukup: tegangan mengikuti langkah beban dalam milidetik,
   * jauh lebih cepat daripada relaksasi kimia sel yang berskala detik. */
  if (s_probe_nunggu && (uint32_t)(millis() - s_probe_ms) >= PROBE_SETTLE_MS) {
    s_probe_nunggu = false;
    const uint32_t t0 = micros();
    int v1  = battery_baca_langsung_mv(NULL);
    s_probe_us = micros() - t0;      /* satu-satunya pembacaan mahal yang tersisa */
    int sag = s_probe_ke_berat ? (s_probe_v0 - v1) : (v1 - s_probe_v0);
    s_sag_mv  = sag;
    s_sag_ada = true;

    /* Zona mati 4 mV di antara kedua ambang sengaja dibiarkan TIDAK memutuskan
     * apa-apa. Pengulangan pengukuran ini +-2 mV (terbaca dari empat probe
     * berturut-turut saat dicolok: -2, 0, 0, +1), jadi nilai di antara 4 dan 5
     * tidak bisa dibedakan dari derau -- dan menebak di situ berarti ikonnya
     * berkedip, yaitu keluhan yang memulai semua ini.
     *
     * Sag KECIL boleh langsung menyalakan: sumber teregulasi tidak punya
     * penjelasan lain. Sag BESAR tidak simetris -- ia muncul baik di baterai
     * MAUPUN saat mengisi di fase CC, jadi ia hanya boleh memadamkan setelah
     * langkah-naik terakhir cukup lama berlalu sehingga aturan 1 sudah punya
     * jendela penuh untuk berbicara. Tanpa gerbang itu, mencolok charger ke
     * baterai yang benar-benar kosong akan memadamkan ikonnya sendiri beberapa
     * detik kemudian. */
    if (sag <= SAG_COLOK_MAX) {
      s_charging = true;
    } else if (sag >= SAG_BATERAI_MIN &&
               (uint32_t)(millis() - s_step_naik_ms) >=
                   (uint32_t)WMIN_SLOTS * WMIN_SLOT_MS) {
      s_charging = false;
    }
  }

  /* ---- 3. Langkah cepat: kabel dicolok / dicabut ----
   * Bukti paling langsung yang ada, karena itu dijalankan terakhir dan menang
   * atas dua aturan di atas. Terukur di board ini: mencabut 4105 -> 4001 mV,
   * mencolok lagi 3967 -> 4104 mV. Seratus milivolt ke atas, dalam satu-dua
   * detik. Tidak ada apa pun dalam pemakaian normal yang menggeser tegangan
   * sebesar itu secepat itu, jadi ambang 50 mV punya margin dua kali lipat dan
   * tetap jauh di atas sag beban (9 mV). */
  s_step[s_step_i] = (uint16_t)s_batt_mv;
  s_step_i = (s_step_i + 1) % STEP_SLOTS;
  if (s_step_n < STEP_SLOTS) s_step_n++;

  if (s_step_n >= STEP_SLOTS) {
    int lama = s_step[s_step_i];
    int baru = s_step[(s_step_i + STEP_SLOTS - 1) % STEP_SLOTS];
    if (baru - lama >= STEP_MV) {
      s_charging     = true;
      s_step_naik_ms = millis();
      s_tren_sah_ms  = millis();     /* bisukan tren: slotnya milik keadaan lama */
      s_sag_ada      = false;
    } else if (lama - baru >= STEP_MV) {
      s_charging     = false;
      s_tren_sah_ms  = millis();
      s_sag_ada      = false;
    }
  }

  s_base_mv = wmin;
  int p = mv_to_percent(s_base_mv);

  /* Histeresis ASIMETRIS. Versi sebelumnya memakai |selisih| > 1 di kedua arah,
   * sehingga penurunan 1% tidak pernah tampil -- baterai harus turun 2% dulu
   * sebelum angkanya bergerak, dan itulah sebab angkanya terasa "macet".
   * Sekarang: turun 1% langsung tampil (itu informasi yang dicari pemakai),
   * naik butuh 2% supaya tidak bergetar di ambang. */
  if (p < s_percent || p >= s_percent + 2 || s_percent == 0) s_percent = p;

  /* Catat satu titik per menit. */
  if (!s_hist_ms || (uint32_t)(millis() - s_hist_ms) >= 60000UL) {
    s_hist_ms = millis();
    s_hist[s_hist_i] = (uint16_t)s_raw_mv;
    s_hist_i = (s_hist_i + 1) % HIST_N;
    if (s_hist_n < HIST_N) s_hist_n++;
  }
}

/* Tulis riwayat ke buf, sampel tertua dulu, satuan mV di pin. */
void battery_history(char *buf, int n) {
  int off = 0;
  buf[0] = '\0';
  for (int k = 0; k < s_hist_n && off < n - 8; k++) {
    int idx = (s_hist_i - s_hist_n + k + 2 * HIST_N) % HIST_N;
    off += snprintf(buf + off, n - off, "%u ", (unsigned)s_hist[idx]);
  }
}

int battery_history_count(void) { return s_hist_n; }

int  battery_floor_mv(void)       { return s_base_mv; }
bool battery_charging(void)       { return s_charging; }
int  battery_percent(void)        { return s_percent; }
int  battery_millivolts(void)     { return s_batt_mv; }
int  battery_raw_millivolts(void) { return s_raw_mv; }
bool battery_valid(void)          { return s_valid; }
int  battery_spread_mv(void)      { return s_spread; }
int  battery_raw_counts(void)     { return s_counts; }

/* ---- bacaan seketika untuk probe sag ----
 * Sengaja memakai buffer lokal, bukan s_ring: mencampurkan sampel probe ke ring
 * berarti langkah beban yang kita buat sendiri ikut menggeser persen yang tampil
 * di layar, yaitu artefak yang justru sedang kita ukur. */
#define PROBE_N 31

int battery_baca_langsung_mv(int *sebaran_pin_mv) {
  int s[PROBE_N];
  for (int k = 0; k < PROBE_N; k++) s[k] = analogReadMilliVolts(BATT_ADC_PIN);
  qsort(s, PROBE_N, sizeof(int), cmp_int);
  if (sebaran_pin_mv) *sebaran_pin_mv = s[PROBE_N - 1] - s[0];
  return (int)(s[PROBE_N / 2] * BATT_DIVIDER + 0.5f);
}

/* Dipanggil TEPAT SEBELUM duty backlight diubah, oleh layar_set() dan oleh
 * penyalaan pertama di setup().
 *
 * Tidak ada probe aktif di sini, dan itu keputusan yang disengaja: build
 * kalibrasi mengedipkan backlight sendiri tiap 20 detik, dan itu tidak bisa
 * dibiarkan di firmware yang dipakai. Ternyata memang tidak perlu -- jam sudah
 * mengubah bebannya sendiri setiap kali layar mati atau menyala, jadi
 * pengukurannya cukup MENUMPANG pada langkah yang toh sudah terjadi. Nol
 * kedipan, nol daya tambahan.
 *
 * Efek sampingnya kebetulan persis yang diinginkan: nilainya diperbarui pada
 * detik layar dinyalakan -- yaitu saat pengguna benar-benar sedang menatap
 * ikonnya. Yang tampil selalu hasil ratusan milidetik lalu, bukan tren 3 menit
 * yang basi.
 *
 * Hanya menyimpan nilai "sebelum"; battery_update() yang menyelesaikannya 300 ms
 * kemudian, sehingga fungsi ini tidak pernah memblokir lebih dari ~3 ms dan
 * aman dipanggil dari konteks loop(). */
void battery_beban_akan_berubah(bool jadi_berat) {
  if (!s_valid) return;                /* belum ada acuan; abaikan saja */

  /* Nilai "sebelum" diambil dari s_batt_mv yang SUDAH ada, bukan dari burst ADC
   * baru. Versi pertama membaca 31 sampel di sini dan itu keliru tempat: fungsi
   * ini dipanggil dari layar_set(), yang ada di jalur tekan-tombol dan
   * bangun-layar -- persis dua interaksi yang paling terasa kalau tertunda.
   * Sekarang biayanya nol, dan yang tersisa hanya satu pembacaan 300 ms
   * kemudian di battery_update(), jauh dari jalur kritis.
   *
   * Boleh dipakai karena s_batt_mv memang berarti "tegangan pada beban yang
   * berlaku sebelum ini" -- beban belum berubah saat baris ini jalan. Syaratnya
   * EMA-nya sempat mengendap, karena itu gerbang 2 detik di bawah: dua
   * transisi berturut-turut yang rapat membuat nilai "sebelum" masih separuh
   * jalan dari langkah sebelumnya, dan sag-nya jadi mengada-ada. */
  const uint32_t sekarang = millis();
  if (s_probe_ms && (uint32_t)(sekarang - s_probe_ms) < 2000UL) {
    s_probe_nunggu = false;            /* terlalu rapat -- lewati, jangan tebak */
    s_probe_ms     = sekarang;
    return;
  }

  s_probe_v0       = s_batt_mv;
  s_probe_ms       = sekarang;
  s_probe_ke_berat = jadi_berat;
  s_probe_nunggu   = true;
}

int  battery_sag_mv(void)   { return s_sag_mv; }
bool battery_sag_valid(void){ return s_sag_ada; }

uint32_t battery_probe_us(void) { return s_probe_us; }
