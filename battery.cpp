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
#define HIST_N 12
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
    s_batt_mv = mv;                              /* pengukuran pertama langsung */
    s_valid = true;
  } else {
    /* EMA alpha 0.2: menahan riak sisa tanpa membuat tanggapan terasa lambat. */
    s_batt_mv = (s_batt_mv * 8 + mv * 2) / 10;
  }

  int p = mv_to_percent(s_batt_mv);

  /* Histeresis 1%: tanpa ini angka di layar bergetar naik-turun satu satuan
   * setiap detik ketika tegangan tepat di ambang. */
  if (p > s_percent + 1 || p < s_percent - 1 || s_percent == 0) s_percent = p;

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

int  battery_percent(void)        { return s_percent; }
int  battery_millivolts(void)     { return s_batt_mv; }
int  battery_raw_millivolts(void) { return s_raw_mv; }
bool battery_valid(void)          { return s_valid; }
int  battery_spread_mv(void)      { return s_spread; }
int  battery_raw_counts(void)     { return s_counts; }
