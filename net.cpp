#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "net.h"
#include "weather.h"
#include "time_manager.h"
#include "config.h"
#include "aw_ble.h"

static volatile bool s_ntp_done = false;

/* Dua gerbang TERPISAH, jangan disatukan: NTP hanya butuh Wi-Fi, sedangkan
 * cuaca butuh Wi-Fi + API key. Kalau digabung, jam ikut tidak disinkronkan
 * hanya karena API key cuaca belum diisi. */
static bool wifi_configured(void) {
  return strcmp(WIFI_SSID, "GANTI_SSID") != 0 && strlen(WIFI_SSID) > 0;
}

static bool owm_configured(void) {
  return strcmp(OWM_API_KEY, "GANTI_API_KEY") != 0 && strlen(OWM_API_KEY) > 0;
}

static bool wifi_connect(void) {
  if (WiFi.status() == WL_CONNECTED) return true;

  Serial.printf("[wifi] menyambung ke \"%s\"", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  /* Menunggu dengan vTaskDelay, bukan delay(): CPU diserahkan ke scheduler
   * sehingga loop() dan LVGL tetap berjalan selama proses ini.
   *
   * Lamanya WIFI_TIMEOUT_MS, bukan 20 detik seperti dulu. Angka lama itu bukan
   * "sabar" melainkan mahal: setiap detiknya adalah detik ketika radio menyapu
   * kanal dan BLE tidak bisa menerima connect request (lihat config.h). Asosiasi
   * yang sehat selesai jauh di bawah 8 detik; sisanya cuma memperpanjang jendela
   * buruk untuk AP yang memang tidak ada. */
  const int langkah = (int)(WIFI_TIMEOUT_MS / 500UL);
  for (int i = 0; i < langkah && WiFi.status() != WL_CONNECTED; i++) {
    vTaskDelay(pdMS_TO_TICKS(500));
    if (i % 4 == 0) Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(" gagal");
    WiFi.disconnect(true);
    return false;
  }
  Serial.printf(" OK, IP %s\n", WiFi.localIP().toString().c_str());
  return true;
}

static bool ntp_sync(void) {
  configTime(TZ_OFFSET_SEC, TZ_DST_SEC,
             "pool.ntp.org", "time.google.com", "time.cloudflare.com");

  struct tm t;
  /* getLocalTime() menunggu sampai SNTP mengisi jam sistem. Blocking, tapi
   * di dalam task ini -- bukan di loop() -- jadi UI tidak terpengaruh. */
  if (!getLocalTime(&t, 10000)) {
    Serial.println("[ntp] gagal, RTC tetap dipakai");
    return false;
  }

  tm_submit_ntp(&t);       /* penulisan ke RTC dilakukan tm_tick() di loop */
  s_ntp_done = true;
  return true;
}

static void net_task(void *arg) {
  (void)arg;

  if (!AW_PAKAI_WIFI) {
    Serial.println("[net] Wi-Fi dimatikan (AW_PAKAI_WIFI 0 di config.h) -- "
                   "NTP & cuaca dilewati.");
    Serial.println("[net] waktu datang dari HP lewat ANCHOR_WAKTU, dan dari RTC "
                   "PCF85063 di antaranya.");
    vTaskDelete(NULL);
    return;
  }
  if (!wifi_configured()) {
    Serial.println("[net] WIFI_SSID di config.h belum diisi -- NTP & cuaca dilewati.");
    Serial.println("[net] jam tetap jalan dari RTC PCF85063.");
    vTaskDelete(NULL);
    return;
  }
  if (!owm_configured())
    Serial.println("[net] OWM_API_KEY belum diisi -- NTP jalan, cuaca dilewati.");

  uint32_t last_ntp = 0, last_try = 0;
  bool first = true;

  /* Backoff berlipat, pola yang sama dengan weather_backoff di bawah. AP yang
   * tidak ada satu menit lalu hampir pasti masih tidak ada sekarang, dan
   * percobaan yang sia-sia di sini dibayar dengan sambungan BLE yang gagal. */
  uint32_t wifi_backoff = WIFI_RETRY_MS;

  /* Jadwal cuaca berbasis "kapan boleh coba lagi", bukan "kapan terakhir
   * berhasil". Versi sebelumnya memakai last_weather=0 sebagai penanda gagal,
   * yang membuat percobaan langsung diulang tiap siklus 2 detik. */
  uint32_t weather_due_ms = 0;      /* 0 = coba sekarang */
  uint32_t weather_backoff = 0;

  for (;;) {
    uint32_t now = millis();

    /* GERBANG: BLE selalu menang.
     *
     * Selama masih ada entri yang belum sampai ke aplikasi -- atau HP sedang
     * tersambung -- radio ini milik BLE sepenuhnya. Hasil pengukuran yang
     * menunggu di buffer tidak bisa ditawar dengan ikon cuaca, dan jam yang
     * baru dinyalakan membawa hasil adalah jam yang seseorang sedang coba
     * sambungkan saat itu juga (lihat ADV_GESIT_MS di aw_ble.cpp).
     *
     * Gerbangnya di sini, bukan di dalam wifi_connect(): percobaan yang sudah
     * berjalan tidak bisa dibatalkan di tengah jalan, jadi yang benar adalah
     * tidak memulainya. */
    bool ble_sibuk = aw_ble_terhubung() || aw_ble_jumlah_tertunda() > 0;

    if (WiFi.status() != WL_CONNECTED && !ble_sibuk) {
      if (first || (uint32_t)(now - last_try) >= wifi_backoff) {
        last_try = millis();
        first = false;
        if (wifi_connect()) {
          wifi_backoff = WIFI_RETRY_MS;
        } else {
          wifi_backoff *= 2;
          if (wifi_backoff > WIFI_RETRY_MAX_MS) wifi_backoff = WIFI_RETRY_MAX_MS;
          Serial.printf("[wifi] coba lagi dalam %lu dtk\n",
                        (unsigned long)(wifi_backoff / 1000UL));
        }
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
      if (!s_ntp_done || (uint32_t)(millis() - last_ntp) >= NTP_RESYNC_MS) {
        if (ntp_sync()) last_ntp = millis();
      }
      if (owm_configured() && (int32_t)(millis() - weather_due_ms) >= 0) {
        if (weather_fetch()) {
          weather_backoff = 0;
          weather_due_ms  = millis() + WEATHER_PERIOD_MS;
        } else {
          /* Backoff berlipat: 30 s, 60 s, 120 s, ... sampai batas 15 menit. */
          weather_backoff = weather_backoff ? weather_backoff * 2
                                            : WEATHER_RETRY_MIN_MS;
          if (weather_backoff > WEATHER_RETRY_MAX_MS)
            weather_backoff = WEATHER_RETRY_MAX_MS;
          weather_due_ms = millis() + weather_backoff;
          Serial.printf("[cuaca] gagal, coba lagi dalam %lu s\n",
                        (unsigned long)(weather_backoff / 1000));
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void net_begin(void) {
  weather_begin();
  /* Prioritas 1 = sama dengan loopTask Arduino, jadi keduanya bergiliran adil.
   * Stack 8 KB: cukup untuk WiFi + HTTPClient + parser JSON. */
  xTaskCreate(net_task, "net", 8192, NULL, 1, NULL);
}

bool net_connected(void) {
  return WiFi.status() == WL_CONNECTED;
}

bool net_ntp_done(void) {
  return s_ntp_done;
}
