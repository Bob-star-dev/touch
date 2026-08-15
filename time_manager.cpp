#include <Arduino.h>
#include <string.h>
#include "time_manager.h"
#include "rtc.h"
#include "config.h"

/* Singkatan bahasa Inggris, huruf besar-kecil seperti lazimnya ("Wed", bukan
 * "WED"). Baris tanggal memakainya apa adanya; lihat status_baris() di
 * touch.ino. */
static const char *HARI[7] = {
  "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
};
static const char *BULAN[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

/* Basis interpolasi: waktu pada saat basis diambil + nilai millis() saat itu. */
static time_t     s_base_epoch = 0;      /* epoch lokal */
static uint32_t   s_base_ms    = 0;
static time_src_t s_src        = TIME_SRC_NONE;
static uint32_t   s_last_rtc_ms = 0;
static char       s_boot_info[80] = "belum diinisialisasi";

/* Titipan dari task jaringan. */
static volatile bool s_ntp_pending = false;
static struct tm     s_ntp_tm;
static portMUX_TYPE  s_mux = portMUX_INITIALIZER_UNLOCKED;

/* ---- konversi tm <-> epoch, keduanya diperlakukan sebagai UTC ----
 * Dipakai berpasangan (timegm-style + gmtime_r) sehingga zona waktu sudah
 * ikut di dalam nilai epoch-nya. Algoritma days_from_civil dari Howard Hinnant.
 */
static long days_from_civil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return (long)era * 146097 + (long)doe - 719468;
}

static time_t tm_to_epoch(const struct tm *t) {
  long days = days_from_civil(t->tm_year + 1900, (unsigned)(t->tm_mon + 1),
                              (unsigned)t->tm_mday);
  return (time_t)days * 86400 + t->tm_hour * 3600 + t->tm_min * 60 + t->tm_sec;
}

/* ---- basis ---- */
static void rebase(const struct tm *t, time_src_t src) {
  s_base_epoch = tm_to_epoch(t);
  s_base_ms    = millis();
  s_src        = src;
}

/* Waktu kompilasi, dipakai sebagai semaian sementara saja (lihat tm_begin). */
static void build_time(struct tm *t) {
  static const char *mon = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char m[4] = { __DATE__[0], __DATE__[1], __DATE__[2], 0 };
  const char *p = strstr(mon, m);
  memset(t, 0, sizeof(*t));
  t->tm_year = atoi(__DATE__ + 7) - 1900;
  t->tm_mon  = p ? (int)((p - mon) / 3) : 0;
  t->tm_mday = atoi(__DATE__ + 4);
  t->tm_hour = atoi(__TIME__);
  t->tm_min  = atoi(__TIME__ + 3);
  t->tm_sec  = atoi(__TIME__ + 6);
  /* tm_wday diisi ulang lewat gmtime_r setelah dinormalkan di rebase(). */
  time_t e = tm_to_epoch(t);
  gmtime_r(&e, t);
}

void tm_begin(void) {
  struct tm t;

  if (rtc_read(&t)) {
    rebase(&t, TIME_SRC_RTC);
    snprintf(s_boot_info, sizeof(s_boot_info),
             "dari RTC %04d-%02d-%02d %02d:%02d:%02d",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);

  } else if (rtc_ok()) {
    /* RTC ada tapi belum pernah diset (baterai cadangan kosong / board baru).
     * Disemai dari waktu kompilasi supaya jam langsung berjalan, bukan berhenti
     * di 00:00 sampai Wi-Fi dikonfigurasi. Ini SEMENTARA: begitu NTP berhasil,
     * nilainya ditimpa. Hapus blok ini kalau lebih suka jam kosong sampai NTP. */
    build_time(&t);
    rtc_write(&t);
    rebase(&t, TIME_SRC_RTC);
    snprintf(s_boot_info, sizeof(s_boot_info),
             "RTC kosong -> semaian kompilasi %04d-%02d-%02d %02d:%02d:%02d",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);

  } else {
    s_src = TIME_SRC_NONE;
    snprintf(s_boot_info, sizeof(s_boot_info), "RTC tidak ada, menunggu NTP");
  }
  s_last_rtc_ms = millis();
}

void tm_submit_ntp(const struct tm *local_time) {
  portENTER_CRITICAL(&s_mux);
  s_ntp_tm = *local_time;
  s_ntp_pending = true;
  portEXIT_CRITICAL(&s_mux);
}

/* Waktu dari HP. Dipanggil aw_jam.cpp saat opcode ANCHOR_WAKTU dijalankan --
 * yaitu di konteks loop, tempat menulis RTC lewat I2C memang aman.
 *
 * Perhatikan bahwa nilai yang masuk adalah epoch UTC (dokumen 5), sedangkan
 * seluruh berkas ini bekerja dengan "epoch lokal": epoch yang sudah digeser
 * TZ_OFFSET_SEC lalu dipecah gmtime_r. Pergeseran itu dilakukan di sini, satu
 * kali, supaya tidak ada pemanggil yang perlu tahu konvensi tersebut.
 *
 * Besar loncatannya ikut dicetak: kalau jam tangan terlihat "melompat" setiap
 * kali HP tersambung, angka itulah yang memberi tahu berapa jauh dan ke arah
 * mana. Selisih yang selalu kelipatan satu jam berarti TZ_OFFSET_SEC salah,
 * bukan RTC yang rusak. */
void tm_terapkan_epoch_utc(uint32_t epoch_utc) {
  time_t lokal = (time_t)epoch_utc + TZ_OFFSET_SEC;
  struct tm t;
  gmtime_r(&lokal, &t);

  long selisih = 0;
  if (s_src != TIME_SRC_NONE) {
    time_t sekarang = s_base_epoch + (time_t)((uint32_t)(millis() - s_base_ms) / 1000UL);
    selisih = (long)(lokal - sekarang);
  }

  rebase(&t, TIME_SRC_BLE);
  rtc_write(&t);                   /* tetap benar setelah HP pergi maupun reboot */
  s_last_rtc_ms = millis();

  Serial.printf("[time] disinkronkan dari HP: %04d-%02d-%02d %02d:%02d:%02d "
                "(geser %+ld dtk)\n",
                t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                t.tm_hour, t.tm_min, t.tm_sec, selisih);
}

void tm_tick(void) {
  /* 1. Terapkan hasil NTP kalau ada titipan dari task jaringan. */
  if (s_ntp_pending) {
    struct tm t;
    portENTER_CRITICAL(&s_mux);
    t = s_ntp_tm;
    s_ntp_pending = false;
    portEXIT_CRITICAL(&s_mux);

    rebase(&t, TIME_SRC_NTP);
    rtc_write(&t);                 /* RTC ikut dikoreksi -> tetap benar tanpa Wi-Fi */
    s_last_rtc_ms = millis();
    Serial.printf("[time] disinkronkan NTP: %04d-%02d-%02d %02d:%02d:%02d\n",
                  t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                  t.tm_hour, t.tm_min, t.tm_sec);
    return;
  }

  /* 2. Baca ulang RTC berkala untuk mengoreksi drift millis(). */
  if ((uint32_t)(millis() - s_last_rtc_ms) >= RTC_RESYNC_MS) {
    s_last_rtc_ms = millis();
    struct tm t;
    if (rtc_read(&t)) {
      /* Jangan turunkan derajat sumber: kalau sudah pernah NTP atau disetel dari
       * HP, RTC memang sudah memuat hasil itu -- ia ditulis balik di kedua jalur
       * -- jadi statusnya tetap seperti semula. */
      rebase(&t, (s_src == TIME_SRC_NTP || s_src == TIME_SRC_BLE)
                   ? s_src : TIME_SRC_RTC);
    }
  }
}

bool tm_now(struct tm *out) {
  if (s_src == TIME_SRC_NONE || !out) return false;
  time_t now = s_base_epoch + (time_t)((uint32_t)(millis() - s_base_ms) / 1000UL);
  gmtime_r(&now, out);
  return true;
}

bool tm_valid(void) {
  return s_src != TIME_SRC_NONE;
}

time_src_t tm_source(void) {
  return s_src;
}

const char *tm_boot_info(void) {
  return s_boot_info;
}

const char *tm_day_name(int wday) {
  return HARI[(wday >= 0 && wday < 7) ? wday : 0];
}

const char *tm_month_name(int mon_0_11) {
  return BULAN[(mon_0_11 >= 0 && mon_0_11 < 12) ? mon_0_11 : 0];
}
