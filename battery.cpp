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

  /* Naik/turun dinilai dari slot tertua vs terbaru, bukan dari nilai sesaat --
   * keduanya sama-sama "minimum berbeban" jadi bisa dibandingkan langsung. */
  if (s_wmin_n >= WMIN_SLOTS) {
    int oldest = s_wmin[s_wmin_i];               /* slot berikut = yang tertua */
    int newest = s_wmin[(s_wmin_i + WMIN_SLOTS - 1) % WMIN_SLOTS];
    s_charging = (newest - oldest) >= 20;
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
