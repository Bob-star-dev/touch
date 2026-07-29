#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "weather.h"
#include "config.h"

static weather_t         s_data = { false, 0, "--" };
static SemaphoreHandle_t s_lock = NULL;

/* Peta kode kondisi OWM -> label pendek Indonesia.
 * Referensi kode: https://openweathermap.org/weather-conditions
 * Semua string di sini sudah diukur <= 74 px pada montserrat_10. */
static const char *cond_from_id(int id) {
  if (id >= 200 && id < 300) return "BADAI";
  if (id >= 300 && id < 400) return "GERIMIS";
  if (id >= 500 && id < 600) return (id <= 501) ? "HUJAN" : "HUJAN LEBAT";
  if (id >= 600 && id < 700) return "SALJU";
  if (id == 781)             return "BADAI";
  if (id >= 700 && id < 800) return "KABUT";
  if (id == 800)             return "CERAH";
  if (id == 801 || id == 802) return "BERAWAN";
  if (id >= 803 && id <= 804) return "MENDUNG";
  return "--";
}

void weather_begin(void) {
  if (!s_lock) s_lock = xSemaphoreCreateMutex();
}

void weather_get(weather_t *out) {
  if (!out) return;
  if (!s_lock) {                 /* belum siap: kembalikan nilai kosong */
    out->valid = false;
    out->temp_c = 0;
    strcpy(out->cond, "--");
    return;
  }
  xSemaphoreTake(s_lock, portMAX_DELAY);
  *out = s_data;
  xSemaphoreGive(s_lock);
}

bool weather_fetch(void) {
  if (WiFi.status() != WL_CONNECTED) return false;

  /* HTTP polos, bukan HTTPS. Alasannya: menghindari biaya heap/CPU TLS di board
   * ini, dan api.openweathermap.org masih melayani port 80. Konsekuensinya API
   * key ikut terkirim tanpa enkripsi -- risiko kecil untuk key cuaca gratis,
   * tapi memang bukan nol. Ganti ke WiFiClientSecure kalau itu penting. */
  String url = String("http://api.openweathermap.org/data/2.5/weather?q=")
             + OWM_CITY + "&units=metric&appid=" + OWM_API_KEY;

  WiFiClient client;
  HTTPClient http;
  http.setConnectTimeout(5000);
  http.setTimeout(8000);
  /* Jangan pakai keep-alive: kalau koneksi ditahan untuk permintaan berikutnya,
   * sumber dayanya baru dilepas 30 menit kemudian saat fetch berikut. */
  http.setReuse(false);
  if (!http.begin(client, url)) {
    Serial.println("[cuaca] http.begin() gagal");
    client.stop();
    return false;
  }

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("[cuaca] HTTP %d\n", code);
    if (code == HTTP_CODE_UNAUTHORIZED)
      Serial.println("[cuaca] 401 -> OWM_API_KEY salah, atau key baru belum aktif "
                     "(butuh 10 menit s/d 2 jam setelah daftar)");
    if (code == HTTP_CODE_NOT_FOUND)
      Serial.printf("[cuaca] 404 -> OWM tidak mengenal OWM_CITY \"%s\". "
                    "Nama resmi kadang beda (Solo -> \"Surakarta,ID\")\n", OWM_CITY);
    http.end();
    client.stop();          /* tutup socket eksplisit, jangan tunggu destruktor */
    return false;
  }

  /* Filter supaya hanya field yang dipakai yang diparse -- hemat RAM. */
  JsonDocument filter;
  filter["main"]["temp"] = true;
  filter["weather"][0]["id"] = true;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, http.getStream(),
                                             DeserializationOption::Filter(filter));
  http.end();
  client.stop();

  if (err) {
    Serial.printf("[cuaca] JSON gagal: %s\n", err.c_str());
    return false;
  }

  if (!doc["main"]["temp"].is<float>() || !doc["weather"][0]["id"].is<int>()) {
    Serial.println("[cuaca] field yang diharapkan tidak ada di respons");
    return false;
  }

  float temp = doc["main"]["temp"].as<float>();
  int   id   = doc["weather"][0]["id"].as<int>();

  xSemaphoreTake(s_lock, portMAX_DELAY);
  s_data.valid  = true;
  s_data.temp_c = (int)lroundf(temp);
  strncpy(s_data.cond, cond_from_id(id), sizeof(s_data.cond) - 1);
  s_data.cond[sizeof(s_data.cond) - 1] = '\0';
  xSemaphoreGive(s_lock);

#if NET_DEBUG
  Serial.printf("[cuaca] %.1f C, id=%d -> %s\n", temp, id, cond_from_id(id));
#endif
  return true;
}
