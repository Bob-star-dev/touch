#include <Arduino.h>
#include <Wire.h>
#include "SensorPCF85063.hpp"
#include "rtc.h"
#include "config.h"

static SensorPCF85063 s_rtc;
static bool s_ok = false;

/* Tahun minimum yang dianggap valid. RTC yang belum pernah diset biasanya
 * melaporkan 2000/2001, jadi nilai di bawah ini berarti "belum diset". */
#define RTC_MIN_YEAR 2020

void rtc_scan_bus(void) {
  Serial.print("[i2c] alamat menjawab:");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", a);
  }
  Serial.println();
}

bool rtc_begin(void) {
  /* Wire.begin() sudah dilakukan di setup() sketch. Lewatkan sda/scl (-1)
   * supaya SensorLib tidak menginisialisasi ulang pin dan mengubah clock bus
   * -- CST816T di board ini butuh 100 kHz dan tidak stabil di 400 kHz. */
  s_ok = s_rtc.begin(Wire);
  if (!s_ok) {
    Serial.println("[rtc] PCF85063 tidak terdeteksi di 0x51");
    rtc_scan_bus();
    return false;
  }
  Serial.println("[rtc] PCF85063 OK");
  return true;
}

bool rtc_ok(void) {
  return s_ok;
}

bool rtc_read(struct tm *out) {
  if (!s_ok || !out) return false;

  RTC_DateTime dt = s_rtc.getDateTime();
  if (dt.getYear() < RTC_MIN_YEAR) {
#if NET_DEBUG
    Serial.printf("[rtc] isi belum valid (tahun %u), menunggu NTP\n", dt.getYear());
#endif
    return false;
  }

  /* Di-nol-kan dulu: toUnixTime() milik SensorLib tidak mengisi tm_isdst/tm_yday,
   * dan struct tm yang setengah terisi bisa menyesatkan pemakainya. */
  memset(out, 0, sizeof(*out));
  out->tm_year = (int)dt.getYear() - 1900;
  out->tm_mon  = (int)dt.getMonth() - 1;
  out->tm_mday = dt.getDay();
  out->tm_hour = dt.getHour();
  out->tm_min  = dt.getMinute();
  out->tm_sec  = dt.getSecond();
  out->tm_wday = dt.getWeek();
  return true;
}

bool rtc_write(const struct tm *in) {
  if (!s_ok || !in) return false;

  RTC_DateTime dt(*in);          /* konstruktor dari struct tm */
  s_rtc.setDateTime(dt);
#if NET_DEBUG
  Serial.printf("[rtc] diset ke %04d-%02d-%02d %02d:%02d:%02d\n",
                in->tm_year + 1900, in->tm_mon + 1, in->tm_mday,
                in->tm_hour, in->tm_min, in->tm_sec);
#endif
  return true;
}
