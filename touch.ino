/*
 * LVGL 8.3 UI -- 5 halaman sesuai mockup 1baru.png + 2..5.png
 * (1baru.png menggantikan 1.png; satu-satunya perbedaan: label "BATERAI" hilang)
 * Board : Waveshare ESP32-C6-Touch-LCD-1.69  (240x280, ST7789V2 + CST816T)
 *
 * Libraries : lvgl 8.3.x, GFX Library for Arduino, SensorLib
 * lv_conf.h : /home/harjo/Arduino/libraries/lv_conf.h  (LV_TICK_CUSTOM=1, 16bpp)
 *
 * Halaman :
 *   1 Home     - ilustrasi, tombol daya sensor, jam + tanggal, header cuaca &
 *                baterai. Tombol daya ON menyalakan MAX30105 sekaligus membuka
 *                menu; OFF mematikan LED-nya dan tetap di halaman ini.
 *   2 Menu     - 4 kartu: Heart rate / SpO2 / Glukosa / Tekanan darah
 *   3 Detail   - Heart rate     : arc, EKG, bar 12 jam, chip statistik
 *   4 Detail   - SpO2           : ring, legend zona, grafik riwayat
 *   5 Detail   - Glukosa        : angka besar, bar rentang, tren harian
 *   6 Detail   - Tekanan darah  : angka sistol/diastol, bar zona, tren harian
 *
 * Halaman 6 tidak punya acuan mockup (fitur ditambahkan belakangan) --
 * geometrinya mengikuti pola halaman 5, bukan piksel dari gambar manapun.
 * Sisanya: geometri & warna diambil langsung dari mockup 480x560 (tepat 2x
 * layar), jadi semua koordinat di bawah = koordinat mockup / 2.
 *
 * Aset gambar (pesawat+bulan, ikon) ada di ui_assets.h, dibuat otomatis dari
 * mockup. Jalankan genassets.py kalau mockup berubah.
 *
 * Modul data (semua akses I2C ada di konteks loop, lihat net.h soal thread):
 *   rtc / time_manager  - PCF85063 + sinkronisasi NTP
 *   net / weather       - Wi-Fi, NTP, OpenWeatherMap
 *   battery             - ADC1 + kurva Li-Po
 *   ppg                 - MAX30105/30102: BPM, SpO2, glukosa, tekanan darah
 *                         EKSPERIMENTAL
 *   aw_proto/aw_store/  - AsaWatch: protokol BLE v1.1, ring buffer NVS, mesin
 *   aw_ble/aw_jam         status sesi. Spesifikasinya ada di
 *                         docs/asawatch-ble-untuk-jam-lvgl.md dan NORMATIF --
 *                         sisi aplikasi Flutter sudah diuji terhadapnya.
 *
 * PERINGATAN: nilai glukosa dan tekanan darah tidak punya dasar fisiologis
 * tervalidasi dan tidak boleh dipakai untuk keputusan medis apa pun. Lihat
 * ppg.h.
 */

#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include "TouchDrv.hpp"
#include "ui_assets.h"

#include "config.h"
#include "rtc.h"
#include "time_manager.h"
#include "weather.h"
#include "net.h"
#include "battery.h"
#include "ppg.h"
#include "aw_jam.h"

/* Subset digit-only dari montserrat_46/48 (cuma glyph 0-9 dan '-'), dipakai di
 * layar jam & glukosa yang tidak pernah menampilkan huruf. Font bawaan LVGL di
 * ukuran itu ikut membawa seluruh ASCII + ikon FontAwesome (~90 KB/font) yang
 * tak pernah dirender di sini; lihat font_digits_46.c / font_digits_48.c. */
extern "C" {
LV_FONT_DECLARE(font_digits_46)
LV_FONT_DECLARE(font_digits_48)
}

/* Tujuan navigasi satu tombol/kartu. Definisinya harus di sini, di atas fungsi
 * pertama: Arduino menyuntikkan prototipe otomatis tepat sebelum definisi fungsi
 * pertama, jadi tipe yang dipakai di parameter harus sudah dikenal sebelum itu. */
typedef struct {
  lv_obj_t  **target;   /* alamat handle layar (layarnya dibuat belakangan) */
  bool        forward;  /* arah animasi: true = slide ke kiri */
  const char *name;     /* untuk log serial */
} nav_target_t;

/* ---------------- Pin map ESP32-C6-Touch-LCD-1.69 ---------------- */
#define LCD_SCK    1
#define LCD_DIN    2
#define LCD_DC     3
#define LCD_RST    4
#define LCD_CS     5
#define LCD_BL     6

#define I2C_SCL    7
#define I2C_SDA    8
#define TOUCH_IRQ 11

/* ---- Daya baterai: latch + tombol PWR ----
 * Kedua nomor ini dari BSP resmi Waveshare untuk board ini
 * (Examples/ESP-IDF/01_factory/components/esp_bsp/bsp_pwr.h di repo
 * waveshareteam/ESP32-C6-Touch-LCD-1.69), dan dikonfirmasi ulang oleh contoh
 * Arduino-nya 03_battery_example.ino yang peta pin LCD + ADC baterainya sama
 * persis dengan berkas ini.
 *
 * BAT_EN adalah gerbang jalur baterai. Tombol PWR hanya menyambungkan baterai
 * SELAMA ditekan; supaya board tetap hidup setelah jari diangkat, firmware
 * harus menahan pin ini HIGH -- itulah "latch"-nya. Tanpa itu board mati
 * seketika saat tombol dilepas, dan gejalanya persis seperti board rusak.
 *
 * Kenapa baru sekarang perlu: memberi daya lewat pin 3V3/5V menyuap rail
 * SETELAH gerbang ini, jadi latch-nya tidak pernah relevan. Begitu daya masuk
 * lewat soket baterai MX1.25 resmi, seluruh arus lewat gerbang ini.
 *
 * PWR_KEY aktif LOW (ditekan = 0). Ia juga dibawa keluar ke header board, jadi
 * jangan pakai pad GPIO18 untuk hal lain -- ia berbagi jalur dengan tombol. */
#define BAT_EN    15
#define PWR_KEY   18

#define SCREEN_W 240
#define SCREEN_H 280

/* ---------------- Display driver ---------------- */
Arduino_DataBus *bus = new Arduino_HWSPI(LCD_DC, LCD_CS, LCD_SCK, LCD_DIN);
Arduino_GFX *gfx = new Arduino_ST7789(
  bus, LCD_RST, 0 /* rotation */, true /* IPS */,
  SCREEN_W, SCREEN_H,
  0 /* col offset 1 */, 20 /* row offset 1 */,
  0 /* col offset 2 */, 20 /* row offset 2 */);

TouchDrvCSTXXX touch;

/* ---------------- LVGL glue ---------------- */
#define BUF_LINES 60
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1 = NULL;
static lv_color_t *buf2 = NULL;
static lv_disp_drv_t disp_drv;
static lv_indev_drv_t indev_drv;

static int16_t last_touch_x = 0, last_touch_y = 0;

/* ---------------- Touch: dibaca saat IRQ ----------------
 * CST816T di board ini mengosongkan byte finger-count hampir seketika setelah
 * IRQ, jadi polling bebas hampir selalu melewatkannya (terukur: 621 IRQ tapi
 * hanya 20 hit dari 1372 polling). Data dibaca tepat saat IRQ, lalu status
 * "ditekan" dipertahankan sampai event lift-up atau timeout supaya drag mulus.
 */
#define TOUCH_ADDR      0x15
#define TOUCH_HOLD_MS   180   /* jaring pengaman kalau event lift-up terlewat */

/* LVGL membaca indev tiap LV_INDEV_DEF_READ_PERIOD (30 ms). Ketukan cepat bisa
 * naik-turun seluruhnya di antara dua polling sehingga LVGL tidak pernah melihat
 * press-nya sama sekali -- ketukan hilang tanpa jejak. Karena itu setiap press
 * ditahan minimal 60 ms (>= 2 siklus polling) sebelum lift-up dieksekusi. */
#define TOUCH_MIN_MS    60

static volatile bool touch_irq_flag = false;
static volatile uint32_t touch_irq_count = 0;
static bool touch_down = false;
static bool touch_release_pending = false;
static uint32_t touch_last_ms = 0;
static uint32_t touch_press_ms = 0;
static uint32_t touch_events = 0;
static uint32_t touch_reads = 0, touch_readerr = 0;

static void IRAM_ATTR touch_isr() {
  touch_irq_flag = true;
  touch_irq_count++;
}

static bool touch_raw_read(uint8_t *b) {
  Wire.beginTransmission(TOUCH_ADDR);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)TOUCH_ADDR, (uint8_t)7) < 7) return false;
  for (uint8_t i = 0; i < 7; i++) b[i] = Wire.read();
  return true;
}

static void touch_poll(void) {
  if (touch_irq_flag) {
    touch_irq_flag = false;
    uint8_t b[7];
    if (!touch_raw_read(b)) {
      touch_readerr++;
    } else {
      touch_reads++;
      uint8_t fingers = b[2] & 0x0F;
      uint8_t evt = b[3] >> 6;                     /* 0=down, 1=lift up, 2=contact */
      uint16_t x = ((b[3] & 0x0F) << 8) | b[4];
      uint16_t y = ((b[5] & 0x0F) << 8) | b[6];

      if (evt == 1) {
        touch_release_pending = true;              /* jari diangkat, tahan dulu */
      } else if ((fingers > 0 || evt == 2) && x < SCREEN_W && y < SCREEN_H) {
        last_touch_x = x;
        last_touch_y = y;
        if (!touch_down) {
          touch_press_ms = millis();
          Serial.printf("[touch] press (%d,%d)\n", x, y);
        }
        touch_down = true;
        touch_release_pending = false;
        touch_last_ms = millis();
        touch_events++;
      }
    }
  }

  if (touch_down) {
    uint32_t now = millis();
    /* lift-up hanya dieksekusi setelah press terlihat >= TOUCH_MIN_MS */
    if (touch_release_pending && (now - touch_press_ms) >= TOUCH_MIN_MS) {
      touch_down = false;
      touch_release_pending = false;
    } else if ((now - touch_last_ms) > TOUCH_HOLD_MS) {
      touch_down = false;                          /* jaring pengaman */
      touch_release_pending = false;
    }
  }
}

static void my_disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;
#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif
  lv_disp_flush_ready(drv);
}

static void my_touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  touch_poll();
  data->point.x = last_touch_x;
  data->point.y = last_touch_y;
  data->state = touch_down ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

/* Glyph non-ASCII yang tersedia di lv_font_montserrat_* bawaan LVGL.
 * Rentangnya 0x20-0x7F + 0xB0 + 0x2022, jadi cuma dua ini yang aman dipakai. */
#define TXT_DEG  "\xC2\xB0"      /* U+00B0 derajat */
#define TXT_DOT  "\xE2\x80\xA2"  /* U+2022 bullet  */

/* ================= Palet (diambil dari mockup) ================= */
/* 1 Home */
#define C_S1_BG     0x034B55
#define C_S1_HDR    0x04363D
#define C_GOLD      0xB79D2D
#define C_AMBER     0xECC94B
#define C_WHITE     0xF6F6F2
#define C_S1_MUTED  0xCFE3DE
#define C_S1_DIV    0x3D6A70
#define C_S1_LINE   0x9BBBBD
#define C_BATT      0xF2CC5B
#define C_DATE      0xE6F2EF
/* 2 Menu */
#define C_S2_BG     0x0E3D47
#define C_S2_BTN    0x155562
#define C_S2_FOOT   0x7FA3AC
#define C_HR_BG     0x5C2530
#define C_HR_TITLE  0xFFE3E3
#define C_HR_SUB    0xF0A8A8
#define C_SP_BG     0x1D3A63
#define C_SP_TITLE  0xE3EFFF
#define C_SP_SUB    0xA9C9F2
#define C_GL_BG     0x4D3A17
#define C_GL_TITLE  0xFFF3D6
#define C_GL_SUB    0xECCF94
#define C_BP_BG     0x3A1E52
#define C_BP_TITLE  0xEFE3FF
#define C_BP_SUB    0xC9A8F0
/* 3 Heart rate */
#define C_S3_BG     0x2A1A24
#define C_S3_DECO   0x39222E
#define C_S3_CARD   0x3D222D
#define C_S3_TRACK  0x4A2A36
#define C_S3_BAR    0x6E3A48
#define C_PINK      0xFF8A8A
#define C_S3_MUTE   0xD8A4A4
/* 4 SpO2 */
#define C_S4_BG     0x14253F
#define C_S4_DECO   0x1B3152
#define C_S4_CARD   0x1F3A61
#define C_S4_TITLE  0xEEF2FB
#define C_S4_MUTE   0xA9C9F2
#define C_BLUE      0x7DB8FF
/* 5 Glukosa */
#define C_S5_BG     0x33270F
#define C_S5_DECO   0x443413
#define C_S5_CARD   0x453413
#define C_S5_MUTE   0xD9BD85
#define C_AMBER2    0xFFCF70
/* 6 Tekanan darah (tanpa acuan mockup, lihat catatan di atas file) */
#define C_S6_BG     0x241A3D
#define C_S6_DECO   0x33234F
#define C_S6_CARD   0x3A2A54
#define C_S6_TITLE  0xF0E6FF
#define C_S6_MUTE   0xC7ADE8
#define C_PURPLE    0xB48CFF  /* sistol */
#define C_PURPLE2   0x7A5CC7  /* diastol */
/* zona */
#define C_GREEN     0x4FBF7A
#define C_GREEN2    0x7EE2A8
#define C_YELLOW    0xE8B34A
#define C_RED       0xE05B5B

/* ================= SUMBER DATA =================
 * Semua nilai di UI kini berasal dari perangkat keras nyata:
 *   jam/hari/tanggal/bulan/tahun  -> RTC PCF85063 + sinkronisasi NTP
 *   cuaca + suhu                  -> OpenWeatherMap lewat Wi-Fi
 *   kapasitas baterai             -> ADC1 + kurva Li-Po
 *   BPM + SpO2                    -> MAX30105/30102 (PPG, Red+IR)
 *   glukosa                       -> MAX30105/30102, EKSPERIMENTAL (lihat ppg.h)
 *
 * Tidak ada lagi angka yang dikarang. Kalau sebuah nilai belum tersedia --
 * sensor tidak menempel, cuaca belum terambil -- label menampilkan "--", bukan
 * angka contoh. Pada layar kesehatan, angka contoh yang tampak nyata lebih
 * berbahaya daripada tanda hubung.
 */

/* ================= Handle layar & widget ================= */
static lv_obj_t *scr_home, *scr_menu, *scr_hr, *scr_spo2, *scr_glu, *scr_bp;
static lv_obj_t *scr_meas;   /* layar "sedang mengukur", dokumen 14 */

/* home */
static lv_obj_t *lbl_hh, *lbl_mm, *lbl_wthr, *lbl_batt;
/* Hari dan tanggal kini satu label. Keduanya dulu bertumpuk (dua baris, 32 px);
 * setelah tombol daya masuk ke halaman ini tinggi itu tidak tersedia lagi, dan
 * keduanya toh selalu berubah pada saat yang sama -- pergantian hari. */
static lv_obj_t *lbl_date;
static lv_obj_t *lbl_cond;   /* caption kondisi cuaca di header */
static lv_obj_t *lbl_ble;    /* ikon Bluetooth di header, tiga keadaan          */
static lv_obj_t *lbl_pending;/* jumlah entri belum di-ack, sembunyi kalau nol   */
static lv_obj_t *lbl_sesi;   /* pil status sesi tepat di atas tombol            */
static lv_obj_t *btn_utama, *ic_utama;   /* satu tombol, dua peran (lihat di bawah) */
/* layar sedang mengukur */
static lv_obj_t *lbl_meas_judul, *lbl_meas_sub, *lbl_meas_waktu, *bar_meas;
static lv_obj_t *dot_meas[4], *lbl_meas_metrik[4];
/* menu */
static lv_obj_t *lbl_card_hr, *lbl_card_sp, *lbl_card_gl, *lbl_card_bp;
/* detail */
static lv_obj_t *arc_hr, *lbl_hr_big, *ring_sp, *lbl_sp_big, *lbl_glu_big;
static lv_obj_t *lbl_bp_big;
static lv_obj_t *dot_live;
/* Handle tambahan supaya chip statistik, teks zona, dan penanda bar ikut
 * mengikuti data nyata. Ini hanya menyimpan pointer objek yang sudah ada --
 * posisi, ukuran, font, dan warnanya tidak diubah. Perlu, karena angka utama
 * yang nyata di atas angka yang dikarang di bawahnya justru menyesatkan. */
static lv_obj_t *chip_hr[3], *chip_sp[3], *chip_gl[3], *chip_bp[3];
static lv_obj_t *lbl_hr_zone, *lbl_glu_status, *lbl_bp_status;
static lv_obj_t *mark_hr, *mark_glu, *mark_bp;

/* ================= Helper pembuat widget ================= */

/* Kotak polos tanpa style bawaan tema. */
static lv_obj_t *mk_box(lv_obj_t *parent, int x, int y, int w, int h,
                        uint32_t bg, int radius) {
  lv_obj_t *o = lv_obj_create(parent);
  lv_obj_remove_style_all(o);
  lv_obj_set_pos(o, x, y);
  lv_obj_set_size(o, w, h);
  lv_obj_set_style_bg_color(o, lv_color_hex(bg), 0);
  lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(o, radius, 0);
  lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  return o;
}

/* Jadikan objek bisa disentuh.
 *
 * ext = piksel perluasan area sentuh di luar batas visual. Tombol back cuma
 * 24x24 px (~2.5 mm di panel 1.69") -- terlalu kecil untuk ujung jari, jadi
 * areanya diperluas tanpa mengubah tampilan. */
static void mk_touchable(lv_obj_t *o, int ext) {
  lv_obj_add_flag(o, LV_OBJ_FLAG_CLICKABLE);
  if (ext) lv_obj_set_ext_click_area(o, ext);
  /* umpan balik visual: sedikit transparan saat ditekan supaya jelas kena */
  lv_obj_set_style_bg_opa(o, LV_OPA_70, LV_STATE_PRESSED);
}

static lv_obj_t *mk_label(lv_obj_t *parent, const char *txt, const lv_font_t *font,
                          uint32_t color, int x, int y) {
  lv_obj_t *l = lv_label_create(parent);
  lv_label_set_text(l, txt);
  lv_obj_set_style_text_font(l, font, 0);
  lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
  lv_obj_set_pos(l, x, y);
  return l;
}

static lv_obj_t *mk_img(lv_obj_t *parent, const lv_img_dsc_t *src, int x, int y) {
  lv_obj_t *i = lv_img_create(parent);
  lv_img_set_src(i, src);
  lv_obj_set_pos(i, x, y);
  return i;
}

/* Lingkaran dekoratif di latar (tidak menangkap sentuhan). */
static void mk_deco(lv_obj_t *scr, int cx, int cy, int r, uint32_t color) {
  lv_obj_t *o = mk_box(scr, cx - r, cy - r, r * 2, r * 2, color, LV_RADIUS_CIRCLE);
  lv_obj_move_background(o);
}

/* Bar tersegmen (zona hijau/kuning/merah) + penanda putih. */
static lv_obj_t *mk_zonebar(lv_obj_t *parent, int x, int y, int w, int h,
                            const int *seg_w, const uint32_t *seg_c, int n,
                            int marker_x) {
  int cx = x;
  for (int i = 0; i < n; i++) {
    int r = (i == 0 || i == n - 1) ? h / 2 : 0;
    mk_box(parent, cx, y, seg_w[i], h, seg_c[i], r);
    cx += seg_w[i];
  }
  return mk_box(parent, x + marker_x, y - 1, 3, h + 2, 0xFFFFFF, 1);
}

/* Tiga chip statistik di bagian bawah halaman detail. */
static void mk_chips(lv_obj_t *parent, const char *a, const char *b, const char *c,
                     uint32_t bg, uint32_t fg, lv_obj_t **out) {
  const char *txt[3] = { a, b, c };
  for (int i = 0; i < 3; i++) {
    lv_obj_t *chip = mk_box(parent, 10 + i * 75, 235, 70, 30, bg, 10);
    lv_obj_t *l = mk_label(chip, txt[i], &lv_font_montserrat_12, fg, 0, 0);
    lv_obj_center(l);
    if (out) out[i] = l;
  }
}

/* Chip kecil di kanan atas halaman detail. */
static lv_obj_t *mk_pill(lv_obj_t *parent, int x, int w, const char *txt,
                         uint32_t bg, uint32_t fg) {
  lv_obj_t *p = mk_box(parent, x, 12, w, 16, bg, 8);
  lv_obj_t *l = mk_label(p, txt, &lv_font_montserrat_10, fg, 0, 0);
  lv_obj_center(l);
  return p;
}

/* ================= Animasi tepi layar: indikator proses pengukuran =================
 * Titik kecil yang berjalan pelan mengelilingi tepi layar, dipasang di keempat
 * halaman detail (HR/SpO2/Glukosa/Tensi). Warnanya cuma memetakan ppg_state_t
 * yang sudah dihitung ppg.cpp, tidak ada logika deteksi baru di sini:
 *   PPG_NO_CONTACT             -> merah   (belum ada kulit menempel)
 *   PPG_SETTLING/PPG_ACQUIRING -> kuning  (sinyal ada, detak belum konsisten)
 *   PPG_STABLE                 -> hijau   (bacaan sudah valid)
 * PPG_ABSENT/PPG_OFF menyembunyikan titiknya -- tidak ada proses untuk
 * diperlihatkan.
 *
 * Timer terpisah dari refresh_cb (500 ms) dengan sengaja: refresh_cb jarang
 * supaya label besar tidak digambar ulang tanpa alasan (lihat catatan di
 * atasnya), tapi gerakan titik ini justru perlu banyak frame per detik supaya
 * terlihat mulus, bukan meloncat.
 */
#define SCAN_DOT_SIZE   10       /* diameter titik, px */
#define SCAN_INSET       3       /* jarak lintasan dari tepi layar, px */
#define SCAN_CORNER_R   18       /* radius sudut lintasan, px */
#define SCAN_STEP   0.0035f      /* kemajuan/frame -> satu putaran ~11 s @ 40 ms/frame */

static lv_obj_t *scan_dot;
static float scan_progress = 0;

/* Titik pada lintasan persegi bersudut bulat di tepi layar, t di rentang
 * [0,1). Jalan searah jarum jam mulai dari tengah sisi atas. */
static lv_point_t scan_point_on_perimeter(float t) {
  /* HALF_PI sudah didefinisikan Arduino.h (1.5707963267948966...), dipakai
   * langsung supaya tidak bentrok nama. */
  int x0 = SCAN_INSET, y0 = SCAN_INSET;
  int x1 = SCREEN_W - SCAN_INSET, y1 = SCREEN_H - SCAN_INSET;
  int r  = SCAN_CORNER_R;
  float sw   = (float)((x1 - x0) - 2 * r);   /* panjang sisi atas/bawah */
  float sh   = (float)((y1 - y0) - 2 * r);   /* panjang sisi kiri/kanan */
  float arc  = r * HALF_PI;                  /* satu sudut, seperempat lingkaran */
  float perim = 2 * sw + 2 * sh + 4 * arc;
  float d = t * perim;

  lv_point_t p;
  if (d < sw) {                                        /* atas: kiri -> kanan */
    p.x = x0 + r + (int)d;  p.y = y0;
  } else if ((d -= sw) < arc) {                         /* sudut kanan-atas */
    float th = -HALF_PI + (d / arc) * HALF_PI;
    p.x = (x1 - r) + (int)(r * cosf(th));  p.y = (y0 + r) + (int)(r * sinf(th));
  } else if ((d -= arc) < sh) {                         /* kanan: atas -> bawah */
    p.x = x1;  p.y = y0 + r + (int)d;
  } else if ((d -= sh) < arc) {                         /* sudut kanan-bawah */
    float th = (d / arc) * HALF_PI;
    p.x = (x1 - r) + (int)(r * cosf(th));  p.y = (y1 - r) + (int)(r * sinf(th));
  } else if ((d -= arc) < sw) {                         /* bawah: kanan -> kiri */
    p.x = x1 - r - (int)d;  p.y = y1;
  } else if ((d -= sw) < arc) {                         /* sudut kiri-bawah */
    float th = HALF_PI + (d / arc) * HALF_PI;
    p.x = (x0 + r) + (int)(r * cosf(th));  p.y = (y1 - r) + (int)(r * sinf(th));
  } else if ((d -= arc) < sh) {                         /* kiri: bawah -> atas */
    p.x = x0;  p.y = y1 - r - (int)d;
  } else {                                              /* sudut kiri-atas */
    d -= sh;
    float th = PI + (d / arc) * HALF_PI;
    p.x = (x0 + r) + (int)(r * cosf(th));  p.y = (y0 + r) + (int)(r * sinf(th));
  }
  return p;
}

static lv_obj_t *mk_scan_dot(lv_obj_t *parent) {
  lv_obj_t *o = lv_obj_create(parent);
  lv_obj_remove_style_all(o);
  lv_obj_set_size(o, SCAN_DOT_SIZE, SCAN_DOT_SIZE);
  lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(o, lv_color_hex(C_RED), 0);
  lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE);   /* jangan pernah menelan sentuhan */
  lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
  return o;
}

static void scan_anim_cb(lv_timer_t *tm) {
  (void)tm;
  lv_obj_t *act = lv_scr_act();
  bool on_detail = (act == scr_hr || act == scr_spo2 || act == scr_glu ||
                    act == scr_bp || act == scr_meas);
  if (!on_detail) {
    lv_obj_add_flag(scan_dot, LV_OBJ_FLAG_HIDDEN);
    return;
  }

  ppg_data_t p;
  ppg_get(&p);
  if (p.state == PPG_ABSENT || p.state == PPG_OFF) {
    lv_obj_add_flag(scan_dot, LV_OBJ_FLAG_HIDDEN);
    return;
  }

  if (lv_obj_get_parent(scan_dot) != act) {
    lv_obj_set_parent(scan_dot, act);
    lv_obj_move_foreground(scan_dot);
  }
  lv_obj_clear_flag(scan_dot, LV_OBJ_FLAG_HIDDEN);

  uint32_t color = C_RED;
  if (p.state == PPG_SETTLING || p.state == PPG_ACQUIRING) color = C_YELLOW;
  else if (p.state == PPG_STABLE)                          color = C_GREEN2;
  lv_obj_set_style_bg_color(scan_dot, lv_color_hex(color), 0);

  scan_progress += SCAN_STEP;
  if (scan_progress >= 1.0f) scan_progress -= 1.0f;

  lv_point_t pt = scan_point_on_perimeter(scan_progress);
  lv_obj_set_pos(scan_dot, pt.x - SCAN_DOT_SIZE / 2, pt.y - SCAN_DOT_SIZE / 2);
}

/* ================= Navigasi ================= */
/* Abaikan event yang datang saat animasi pindah layar masih jalan, supaya satu
 * ketukan tidak memicu dua transisi bertumpuk. */
static uint32_t nav_lock_ms = 0;

static void go(lv_obj_t *target, bool forward, const char *name) {
  if (millis() - nav_lock_ms < 220) return;
  nav_lock_ms = millis();
  Serial.printf("[nav] -> %s\n", name);
  lv_scr_load_anim(target,
                   forward ? LV_SCR_LOAD_ANIM_MOVE_LEFT : LV_SCR_LOAD_ANIM_MOVE_RIGHT,
                   180, 0, false);
}

/* ---- Navigasi khusus ketukan ----
 * Kartu dan tombol back TIDAK boleh aktif karena geseran: dulu memakai
 * LV_EVENT_PRESSED, jadi menggeser jari melewati kartu SpO2 langsung membuka
 * halamannya. LV_EVENT_CLICKED bawaan LVGL juga tidak menolongnya, sebab LVGL
 * hanya membatalkan klik kalau ada leluhur yang scrollable -- di UI ini semua
 * scroll dimatikan, sehingga CLICKED tetap terkirim setelah geseran panjang.
 * Karena itu jaraknya diukur sendiri: press dicatat, lalu saat release jarak
 * tempuhnya dibandingkan dengan TAP_SLOP. */
#define TAP_SLOP 12   /* px; ketukan normal bergetar < 5 px, geseran > 30 px */

static const nav_target_t NAV_HOME = { &scr_home, false, "home" };
static const nav_target_t NAV_MENU = { &scr_menu, false, "menu (back)" };
static const nav_target_t NAV_MENU_DARI_HOME = { &scr_menu, true, "menu" };
static const nav_target_t NAV_HR   = { &scr_hr,   true,  "heart rate" };
static const nav_target_t NAV_SPO2 = { &scr_spo2, true,  "spo2" };
static const nav_target_t NAV_GLU  = { &scr_glu,  true,  "glukosa" };
static const nav_target_t NAV_BP   = { &scr_bp,   true,  "tekanan darah" };

static int16_t tap_x0, tap_y0;

static void tap_press_cb(lv_event_t *e) {
  (void)e;
  tap_x0 = last_touch_x;
  tap_y0 = last_touch_y;
}

static void tap_release_cb(lv_event_t *e) {
  const nav_target_t *t = (const nav_target_t *)lv_event_get_user_data(e);
  int dx = last_touch_x - tap_x0;
  int dy = last_touch_y - tap_y0;
  if (dx * dx + dy * dy > TAP_SLOP * TAP_SLOP) {
    Serial.printf("[nav] geseran %d px, %s dibatalkan\n",
                  (int)sqrtf((float)(dx * dx + dy * dy)), t->name);
    return;
  }
  go(*t->target, t->forward, t->name);
}

/* Pasang navigasi ketuk-saja pada sebuah objek. */
static void mk_tap_nav(lv_obj_t *o, int ext, const nav_target_t *t) {
  mk_touchable(o, ext);
  lv_obj_add_event_cb(o, tap_press_cb,   LV_EVENT_PRESSED,  (void *)t);
  lv_obj_add_event_cb(o, tap_release_cb, LV_EVENT_RELEASED, (void *)t);
}

/* Header standar halaman 2..5: tombol back + judul. */
static lv_obj_t *mk_header(lv_obj_t *scr, const char *title, uint32_t btn_bg,
                           const nav_target_t *back) {
  lv_obj_t *btn = mk_box(scr, 10, 10, 24, 24, btn_bg, 8);
  mk_tap_nav(btn, 14, back);        /* target sentuh efektif jadi ~52x52 */
  lv_obj_t *ic = mk_label(btn, LV_SYMBOL_LEFT, &lv_font_montserrat_12, 0xFFFFFF, 0, 0);
  lv_obj_center(ic);
  return mk_label(scr, title, &lv_font_montserrat_16, 0xFFFFFF, 43, 12);
}

/* ================= Tombol utama: satu tombol, dua peran =================
 * Satu-satunya kendali di halaman home, di tengah, dan artinya ditentukan oleh
 * status sesi -- bukan oleh dua tombol terpisah.
 *
 *   IDLE     "Cek manual". Menekannya mengukur sekali jalan, dan hasilnya
 *            berhenti di layar jam: tidak ada entri sampel, tidak ada event,
 *            tidak ada satu byte pun ke aplikasi.
 *   ARMED    "Selesai Makan", hijau dan menonjol. Ini keadaan yang pengguna
 *            tunggu, dan menekannya adalah SUMBER TUNGGAL t0.
 *   RUNNING  ambar, tidak menerima tekanan lagi; pil di atasnya berubah jadi
 *            hitung mundur ke pengukuran berikutnya.
 *
 * Menggabungkan keduanya justru memperkuat jaminan yang paling penting di
 * dokumen 12: tidak ada sesi tanpa foto makanan. Di IDLE tombol ini memang
 * menyala dan memang bisa ditekan, tetapi yang dijalankannya cek manual --
 * BUKAN memulai sesi. Sesi tetap hanya bisa lahir setelah aplikasi mengirim
 * ARM_SESI, persis seperti sebelumnya.
 *
 * Yang perlu dijaga saat menyentuh kode ini: peran tombol harus selalu terbaca
 * dari layar, bukan dihafal. Itu tugas ikon + warna di sini dan pil status di
 * atasnya -- keduanya harus berubah bersamaan, karena satu tombol yang
 * mengerjakan dua hal berbeda tanpa penanda adalah cara termudah membuat
 * pengguna mengira pengukuran manualnya terkirim ke aplikasi.
 */
static void utama_btn_refresh(void) {
  /* Style hanya disentuh saat keadaan benar-benar berpindah. Fungsi ini
   * dipanggil dari refresh_cb tiap 500 ms, dan lv_obj_set_style_*() selalu
   * meng-invalidate objeknya -- tanpa penjaga ini tombolnya digambar ulang dua
   * kali per detik tanpa alasan. */
  static int last = -1;
  int st = jam_status();
  if (st == last) return;
  last = st;

  uint32_t bg, border, fg;
  const char *ikon;
  if (st == AW_SESI_ARMED) {
    bg = C_GREEN;  border = C_GREEN2; fg = 0xFFFFFF; ikon = LV_SYMBOL_OK;
  } else if (st == AW_SESI_RUNNING) {
    bg = C_S1_HDR; border = C_AMBER;  fg = C_AMBER;  ikon = LV_SYMBOL_REFRESH;
  } else {
    /* Peran cek manual. Ikonnya LV_SYMBOL_POWER seperti tombol daya lama,
     * karena inilah yang menggantikan fungsinya: menyalakan sensor sekarang,
     * atas kehendak pengguna. */
    bg = C_S1_HDR; border = C_S1_DIV; fg = C_S1_LINE; ikon = LV_SYMBOL_POWER;
  }
  lv_obj_set_style_bg_color(btn_utama, lv_color_hex(bg), 0);
  lv_obj_set_style_border_color(btn_utama, lv_color_hex(border), 0);
  lv_obj_set_style_text_color(ic_utama, lv_color_hex(fg), 0);
  lv_label_set_text(ic_utama, ikon);
}

/* Ketuk-saja, dengan pengukuran jarak yang sama seperti kartu menu: kedua peran
 * tombol ini menyalakan perangkat keras dan salah satunya memulai sesi dua jam,
 * jadi geseran yang kebetulan berakhir di atasnya jelas bukan maksud pengguna. */
static void utama_release_cb(lv_event_t *e) {
  (void)e;
  int dx = last_touch_x - tap_x0;
  int dy = last_touch_y - tap_y0;
  if (dx * dx + dy * dy > TAP_SLOP * TAP_SLOP) {
    Serial.printf("[nav] geseran %d px, tombol utama dibatalkan\n",
                  (int)sqrtf((float)(dx * dx + dy * dy)));
    return;
  }

  /* Percabangan peran ada di sini, satu tempat. aw_jam tetap memeriksa
   * syaratnya sendiri -- jam_cek_manual() menolak kalau ada sesi, dan
   * jam_tekan_tombol() menolak kalau belum ARMED -- jadi baris ini soal maksud
   * pengguna, bukan soal keamanan. */
  if (jam_status() == AW_SESI_IDLE) jam_cek_manual();
  else                              jam_tekan_tombol();
  utama_btn_refresh();
}

/* ================= Halaman 1 : Home ================= */
static void build_home(void) {
  scr_home = lv_obj_create(NULL);
  lv_obj_remove_style_all(scr_home);
  lv_obj_set_style_bg_color(scr_home, lv_color_hex(C_S1_BG), 0);
  lv_obj_set_style_bg_opa(scr_home, LV_OPA_COVER, 0);
  lv_obj_clear_flag(scr_home, LV_OBJ_FLAG_SCROLLABLE);

  /* --- header --- */
  lv_obj_t *hdr = mk_box(scr_home, 0, 0, SCREEN_W, 31, C_S1_HDR, 0);
  mk_box(scr_home, 0, 31, SCREEN_W, 1, C_GOLD, 0);           /* garis emas */

  /* Baris header disetel supaya pusat kapital "29C" dan "82%" sama-sama jatuh
   * di y=13 seperti mockup: montserrat_12 punya line_height 15 dan base_line 3,
   * jadi kapital mulai di (y_label + 4) dan pusatnya di (y_label + 8). */
  mk_img(hdr, &img_weather, 12, 5);
  lbl_wthr = mk_label(hdr, "--" TXT_DEG "C", &lv_font_montserrat_12, C_WHITE, 46, 5);
  /* Handle disimpan supaya teksnya bisa diisi data OWM. Posisi, font, dan warna
   * tidak diubah -- hanya pointernya yang kini dipegang. */
  lbl_cond = mk_label(hdr, "--", &lv_font_montserrat_10, C_WHITE, 46, 17);

  mk_box(hdr, 120, 7, 1, 18, C_S1_DIV, 0);                   /* pemisah */

  /* Ikon Bluetooth, TIGA keadaan -- bukan dua (dokumen 14):
   *   redup       tidak tersambung
   *   biru pucat  tersambung, tapi aplikasi belum menulis CCCD
   *   biru terang tersambung DAN dilanggani -- hanya ini yang berarti data
   *               sedang benar-benar mengalir keluar
   * Perbedaan kedua dan ketiga itu yang paling sering dikira sama; kalau
   * digabung, jam yang tersambung tapi diam terlihat identik dengan jam yang
   * bekerja normal.
   *
   * x=132 aman: persen baterai dirata-kanan ke x=190 dan string terlebar yang
   * mungkin ("100%") mulai di ~159. */
  lbl_ble = mk_label(hdr, LV_SYMBOL_BLUETOOTH, &lv_font_montserrat_12,
                     C_S1_DIV, 132, 8);

  /* Entri yang belum di-ack. Informasi yang menenangkan, bukan peringatan:
   * buffer 64 entri setara ~16 sesi, jadi angka kecil di sini normal.
   * Disembunyikan saat nol supaya header tidak ramai tanpa alasan. */
  lbl_pending = mk_label(hdr, "", &lv_font_montserrat_10, C_S1_MUTED, 145, 9);
  lv_obj_add_flag(lbl_pending, LV_OBJ_FLAG_HIDDEN);

  /* Kelompok baterai. Angkanya dirata-kanan ke x=190 supaya jarak 7 px ke ikon
   * tetap sama walau lebarnya berubah ("9%" vs "100%"). Ukuran font mengikuti
   * mockup: kapital 8 px / lebar 25 px = montserrat_12, sepasang dengan "29C". */
  lbl_batt = mk_label(hdr, "82%", &lv_font_montserrat_12, C_BATT, 0, 5);
  lv_obj_set_width(lbl_batt, 190);
  lv_obj_set_style_text_align(lbl_batt, LV_TEXT_ALIGN_RIGHT, 0);

  /* 1baru.png menghapus label "BATERAI" -- tinggal persentase + ikon. */
  mk_img(hdr, &img_battery, 197, 8);

  /* ================= Tata letak vertikal halaman home =================
   * Urutan: ilustrasi -> pil status -> tombol sesi -> jam -> hari+tanggal.
   *
   * Ruang di bawah header hanya 248 px (y=32..279) dan sekarang harus memuat
   * satu elemen tambahan. Anggarannya, memakai tinggi kotak font sebenarnya:
   *   pesawat 122 + tombol 46 + jam 52 (font_digits_48) + baris tanggal 15
   *   = 235, menyisakan 13 px untuk SELURUH jarak antar-elemen.
   *
   * Karena itu dua hal berubah, dan keduanya memang menghasilkan tempat:
   *   - hari dan tanggal digabung jadi satu baris (hemat 17 px). Keduanya toh
   *     selalu berubah pada saat yang sama, yaitu saat tanggal berganti.
   *   - pemisah garis-berlian-garis dilepas (hemat 10 px). Tempatnya persis
   *     yang kini ditempati tombol daya, dan tombol itu sendiri sudah menjadi
   *     pemisah visual antara ilustrasi dan blok jam.
   *
   * Jarak antar kotak terbaca rapat di koordinat, tapi tidak di layar: kotak
   * font_digits_48 punya 9 px kosong di atas ink digit dan 9 px di bawahnya,
   * dan montserrat_12 punya 4 px di atas kapital. Jarak visual dari ink jam ke
   * kapital tanggal misalnya 14 px, bukan 1 px seperti yang terbaca dari
   * selisih koordinatnya.
   */

  /* --- ilustrasi pesawat + bulan (152x122) ---
   * Sekaligus jalan menuju menu kesehatan. Tombol home sudah dipakai sesi, dan
   * menambah tombol kelima ke layar 240x280 yang sudah padat akan lebih buruk
   * daripada memakai gambar yang toh sudah ada di sana. Ketuk-saja, sehingga
   * geseran tidak membukanya. */
  lv_obj_t *img_ilus = mk_img(scr_home, &img_plane, 44, 33);
  mk_tap_nav(img_ilus, 0, &NAV_MENU_DARI_HOME);

  /* --- pil status sesi, menempel di atas tombol ---
   * 19 px terakhir dari pita ilustrasi. Ia memang menutupi sedikit bagian bawah
   * gambar, dan itu pilihan sadar: tanpa baris ini tombol di bawahnya tidak
   * punya penjelasan sama sekali, dan pengguna yang menekan tombol redup lalu
   * tidak terjadi apa-apa akan menyimpulkan jamnya rusak. */
  lv_obj_t *pil = mk_box(scr_home, 25, 136, 190, 19, C_S1_HDR, 9);
  lbl_sesi = mk_label(pil, "", &lv_font_montserrat_10, C_S1_MUTED, 0, 0);
  lv_obj_set_width(lbl_sesi, 186);
  lv_obj_set_style_text_align(lbl_sesi, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_center(lbl_sesi);

  /* --- tombol utama, di tengah tepat di bawah ilustrasi ---
   * Tetap 46x46 di x=97 seperti sebelumnya: ini satu-satunya kendali di layar
   * ini, jadi ia memang milik sumbu tengah. Perannya yang berganti mengikuti
   * status sesi, bukan posisinya. */
  btn_utama = mk_box(scr_home, 97, 159, 46, 46, C_S1_HDR, LV_RADIUS_CIRCLE);
  lv_obj_set_style_border_width(btn_utama, 2, 0);
  lv_obj_set_style_border_color(btn_utama, lv_color_hex(C_S1_DIV), 0);
  lv_obj_set_style_border_opa(btn_utama, LV_OPA_COVER, 0);
  mk_touchable(btn_utama, 14);          /* target sentuh efektif jadi ~74x74 */
  lv_obj_add_event_cb(btn_utama, tap_press_cb,     LV_EVENT_PRESSED,  NULL);
  lv_obj_add_event_cb(btn_utama, utama_release_cb, LV_EVENT_RELEASED, NULL);

  ic_utama = mk_label(btn_utama, LV_SYMBOL_POWER, &lv_font_montserrat_22,
                      C_S1_LINE, 0, 0);
  lv_obj_center(ic_utama);

  /* --- jam: "10" [kotak][kotak] "24" ---
   * Kotak titik dua tetap di +16 dan +32 dari y label: jarak relatifnya
   * terhadap ink digit (y+9..y+43) sama persis seperti tata letak sebelumnya. */
  lbl_hh = mk_label(scr_home, "10", &font_digits_48, 0xFFFFFF, 0, 206);
  lv_obj_set_width(lbl_hh, 112);
  lv_obj_set_style_text_align(lbl_hh, LV_TEXT_ALIGN_RIGHT, 0);

  mk_box(scr_home, 116, 222, 8, 8, C_AMBER, 1);
  mk_box(scr_home, 116, 238, 8, 8, C_AMBER, 1);

  lbl_mm = mk_label(scr_home, "24", &font_digits_48, 0xFFFFFF, 128, 206);

  /* --- hari + tanggal, satu baris ---
   * letter_space 1, bukan 2: string terpanjang yang mungkin muncul
   * ("MINGGU " TXT_DOT " 28 SEPTEMBER 2026") terukur 215 px di montserrat_12
   * dengan spasi 1, tapi 240 px dengan spasi 2 -- tepat selebar layar, tanpa
   * margin sama sekali. Diukur dari tabel advance width font-nya, bukan
   * dikira-kira.
   *
   * y=259 menyisakan 6 px di bawah, sama seperti tata letak lama -- panel 1.69"
   * ini bersudut membulat, jadi baris terakhir sengaja tidak dirapatkan ke tepi.
   * Kotak jam (206..257) karenanya juga tidak menimpa kotak baris ini, dan
   * jarak ink digit ke kapital tanggal jadi 14 px -- persis seperti sebelumnya,
   * karena kotak font_digits_48 sendiri menyumbang 9 px kosong di bawah
   * angkanya. */
  lbl_date = mk_label(scr_home, "KAMIS " TXT_DOT " 23 JULI 2026",
                      &lv_font_montserrat_12, C_DATE, 0, 259);
  lv_obj_set_style_text_letter_space(lbl_date, 1, 0);
  lv_obj_set_width(lbl_date, SCREEN_W);
  lv_obj_set_style_text_align(lbl_date, LV_TEXT_ALIGN_CENTER, 0);

  utama_btn_refresh();
}

/* ================= Layar "sedang mengukur" =================
 * Dokumen 14: dengan sensor sungguhan satu pengukuran makan puluhan detik dan
 * pengguna harus diam, jadi ia perlu layarnya sendiri -- bukan sekadar ikon di
 * pojok. Layar ini muncul sendiri saat pengukuran mulai dan menghilang sendiri
 * saat selesai; tidak ada tombol batal, karena dokumen memang tidak memberi
 * jam wewenang membatalkan pengukuran (itu milik opcode BATAL_SESI dari
 * aplikasi), dan pengukuran toh berhenti sendiri dalam satu menit.
 *
 * Daftar keempat metrik bukan hiasan: ia menjawab pertanyaan yang paling wajar
 * saat menunggu, yaitu "ini sedang jalan atau menggantung". Titik yang satu per
 * satu berubah hijau memperlihatkan kemajuan yang sebenarnya, dan yang belum
 * hijau saat waktu habis persis metrik yang nanti dikirim sebagai sentinel 0.
 */
static void build_meas(void) {
  scr_meas = lv_obj_create(NULL);
  lv_obj_remove_style_all(scr_meas);
  lv_obj_set_style_bg_color(scr_meas, lv_color_hex(C_S1_BG), 0);
  lv_obj_set_style_bg_opa(scr_meas, LV_OPA_COVER, 0);
  lv_obj_clear_flag(scr_meas, LV_OBJ_FLAG_SCROLLABLE);

  mk_deco(scr_meas, 220, 40, 44, C_S1_HDR);
  mk_deco(scr_meas, 18, 245, 40, C_S1_HDR);

  lbl_meas_judul = mk_label(scr_meas, "Mengukur", &lv_font_montserrat_16,
                            0xFFFFFF, 0, 26);
  lv_obj_set_width(lbl_meas_judul, SCREEN_W);
  lv_obj_set_style_text_align(lbl_meas_judul, LV_TEXT_ALIGN_CENTER, 0);

  lbl_meas_sub = mk_label(scr_meas, "Tempelkan jari, jangan bergerak",
                          &lv_font_montserrat_10, C_S1_MUTED, 0, 50);
  lv_obj_set_width(lbl_meas_sub, SCREEN_W);
  lv_obj_set_style_text_align(lbl_meas_sub, LV_TEXT_ALIGN_CENTER, 0);

  /* Empat baris metrik. Urutannya mengikuti urutan matangnya di ppg.cpp --
   * detak lebih dulu, glukosa dan tensi paling akhir -- supaya titik-titiknya
   * menyala kira-kira dari atas ke bawah dan terbaca sebagai kemajuan. */
  static const char *NAMA[4] = { "Detak jantung", "SpO2", "Glukosa", "Tekanan darah" };
  for (int i = 0; i < 4; i++) {
    int y = 78 + i * 30;
    lv_obj_t *baris = mk_box(scr_meas, 20, y, 200, 24, C_S1_HDR, 8);
    dot_meas[i] = mk_box(baris, 0, 0, 10, 10, C_S1_DIV, LV_RADIUS_CIRCLE);
    lv_obj_align(dot_meas[i], LV_ALIGN_LEFT_MID, 10, 0);
    lbl_meas_metrik[i] = mk_label(baris, NAMA[i], &lv_font_montserrat_12,
                                  C_S1_MUTED, 0, 0);
    lv_obj_align(lbl_meas_metrik[i], LV_ALIGN_LEFT_MID, 28, 0);
  }

  /* Bar kemajuan waktu. Dibuat dari dua kotak polos, bukan lv_bar: satu-satunya
   * yang berubah adalah lebar kotak dalam, dan lv_bar membawa animasi + style
   * indicator yang tidak dipakai di sini. */
  mk_box(scr_meas, 20, 212, 200, 6, C_S1_HDR, 3);
  bar_meas = mk_box(scr_meas, 20, 212, 0, 6, C_AMBER, 3);

  lbl_meas_waktu = mk_label(scr_meas, "0s", &lv_font_montserrat_12,
                            C_S1_MUTED, 0, 228);
  lv_obj_set_width(lbl_meas_waktu, SCREEN_W);
  lv_obj_set_style_text_align(lbl_meas_waktu, LV_TEXT_ALIGN_CENTER, 0);
}

/* ================= Halaman 2 : Menu kesehatan ================= */
/* Satu kartu menu: latar berwarna, ikon, judul, nilai, chevron.
 *
 * Tinggi kartu diparameterkan (dulu tetap 64 px) sejak kartu ke-4 (Tekanan
 * darah) ditambahkan -- 4 kartu 64 px + footer tidak muat di layar 280 px
 * tinggi, jadi seluruh kartu diperkecil bersama-sama, bukan cuma yang baru. */
static lv_obj_t *mk_card(lv_obj_t *scr, int y, int h, uint32_t bg, const char *title,
                         const char *val, uint32_t c_title, uint32_t c_sub,
                         const nav_target_t *nav, lv_obj_t **out_val) {
  lv_obj_t *card = mk_box(scr, 10, y, 220, h, bg, 12);
  mk_tap_nav(card, 0, nav);         /* 220 px lebar, sudah cukup besar */

  mk_label(card, title, &lv_font_montserrat_14, c_title, 45, 6);
  *out_val = mk_label(card, val, &lv_font_montserrat_14, c_sub, 45, 24);

  lv_obj_t *ch = mk_label(card, LV_SYMBOL_RIGHT, &lv_font_montserrat_12, c_sub, 0, 0);
  lv_obj_align(ch, LV_ALIGN_RIGHT_MID, -10, 0);
  return card;
}

static void build_menu(void) {
  scr_menu = lv_obj_create(NULL);
  lv_obj_remove_style_all(scr_menu);
  lv_obj_set_style_bg_color(scr_menu, lv_color_hex(C_S2_BG), 0);
  lv_obj_set_style_bg_opa(scr_menu, LV_OPA_COVER, 0);
  lv_obj_clear_flag(scr_menu, LV_OBJ_FLAG_SCROLLABLE);

  mk_header(scr_menu, "Menu kesehatan", C_S2_BTN, &NAV_HOME);

  /* 4 kartu tinggi 52 px, jarak 60 px (gap 8 px): 42, 102, 162, 222 ->
   * kartu terakhir berakhir di 274, menyisakan 6 px ke tepi bawah layar
   * bundar -- sama seperti margin bawah di halaman home. Footer
   * "3 sensor aktif..." dihapus: tidak ada lagi ruang untuknya, dan dengan
   * 4 metrik dari sensor yang sama, "3 sensor" sudah tidak akurat juga. */
  static const int CARD_H = 52, CARD_Y0 = 42, CARD_DY = 60;
  lv_obj_t *c1 = mk_card(scr_menu, CARD_Y0,              CARD_H, C_HR_BG, "Heart rate", "-- bpm",
                         C_HR_TITLE, C_HR_SUB, &NAV_HR,   &lbl_card_hr);
  lv_obj_t *c2 = mk_card(scr_menu, CARD_Y0 + CARD_DY,     CARD_H, C_SP_BG, "SpO2", "-- %",
                         C_SP_TITLE, C_SP_SUB, &NAV_SPO2, &lbl_card_sp);
  lv_obj_t *c3 = mk_card(scr_menu, CARD_Y0 + CARD_DY * 2, CARD_H, C_GL_BG, "Glukosa", "-- mg/dL",
                         C_GL_TITLE, C_GL_SUB, &NAV_GLU,  &lbl_card_gl);
  lv_obj_t *c4 = mk_card(scr_menu, CARD_Y0 + CARD_DY * 3, CARD_H, C_BP_BG, "Tekanan darah", "--/-- mmHg",
                         C_BP_TITLE, C_BP_SUB, &NAV_BP,   &lbl_card_bp);

  /* ikon kartu: dipusatkan vertikal lewat align, bukan koordinat y tetap --
   * tahan terhadap perubahan tinggi kartu di masa depan. */
  lv_obj_t *i1 = mk_img(c1, &img_heart_sm, 12, 0); lv_obj_align(i1, LV_ALIGN_LEFT_MID, 12, 0);
  lv_obj_t *i2 = mk_img(c2, &img_drop,     14, 0); lv_obj_align(i2, LV_ALIGN_LEFT_MID, 14, 0);
  lv_obj_t *i3 = mk_box(c3, 0, 0, 16, 16, C_AMBER2, LV_RADIUS_CIRCLE); lv_obj_align(i3, LV_ALIGN_LEFT_MID, 14, 0);
  lv_obj_t *i4 = mk_box(c4, 0, 0, 16, 16, C_PURPLE, LV_RADIUS_CIRCLE); lv_obj_align(i4, LV_ALIGN_LEFT_MID, 14, 0);
}

/* ================= Halaman 3 : Heart rate ================= */
/* Gelombang EKG: 4 denyut, koordinat relatif terhadap posisi objek line. */
#define ECG_BEATS 4
#define ECG_PTS   (ECG_BEATS * 6 + 1)
static lv_point_t ecg_pts[ECG_PTS];

static void ecg_build(void) {
  /* satu denyut: datar, takik kecil, spike naik, palung, kembali datar */
  static const int dx[6] = { 0, 12, 16, 20, 26, 30 };
  static const int dy[6] = { 0, 0,   3, -11,  8,  0 };
  int i = 0;
  for (int b = 0; b < ECG_BEATS; b++) {
    int x0 = b * 55;
    for (int k = 0; k < 6; k++) {
      ecg_pts[i].x = x0 + dx[k];
      ecg_pts[i].y = 12 + dy[k];
      i++;
    }
  }
  ecg_pts[i].x = ECG_BEATS * 55;
  ecg_pts[i].y = 12;
}

static void build_hr(void) {
  scr_hr = lv_obj_create(NULL);
  lv_obj_remove_style_all(scr_hr);
  lv_obj_set_style_bg_color(scr_hr, lv_color_hex(C_S3_BG), 0);
  lv_obj_set_style_bg_opa(scr_hr, LV_OPA_COVER, 0);
  lv_obj_clear_flag(scr_hr, LV_OBJ_FLAG_SCROLLABLE);

  mk_deco(scr_hr, 220, 48, 48, C_S3_DECO);
  mk_deco(scr_hr, 15, 230, 45, C_S3_DECO);

  mk_header(scr_hr, "Heart rate", C_S3_CARD, &NAV_MENU);

  /* penanda "live" */
  dot_live = mk_box(scr_hr, 198, 19, 6, 6, C_GREEN2, LV_RADIUS_CIRCLE);
  mk_label(scr_hr, "live", &lv_font_montserrat_12, C_GREEN2, 208, 15);

  /* arc + ikon hati */
  arc_hr = lv_arc_create(scr_hr);
  lv_obj_set_pos(arc_hr, 16, 50);
  lv_obj_set_size(arc_hr, 68, 68);
  lv_arc_set_rotation(arc_hr, 135);
  lv_arc_set_bg_angles(arc_hr, 0, 270);
  lv_arc_set_range(arc_hr, 0, 100);
  lv_arc_set_value(arc_hr, 0);
  lv_obj_remove_style(arc_hr, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(arc_hr, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_width(arc_hr, 7, LV_PART_MAIN);
  lv_obj_set_style_arc_width(arc_hr, 7, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(arc_hr, lv_color_hex(C_S3_TRACK), LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc_hr, lv_color_hex(C_PINK), LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(arc_hr, true, LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(arc_hr, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(arc_hr, 0, 0);
  /* Posisi dihitung, bukan dikira: aset sekarang 21x20 dengan pusat hati di
   * (9.5, 9.0) di dalamnya, jadi (40,75) menempatkan pusat hati di (49.5, 84.0)
   * -- pusat arc ada di (16+34, 50+34) = (50, 84). Sebelumnya aset 30x30 di
   * (35,69) menaruh hati di (45, 78.5) karena padding crop-nya tidak simetris. */
  mk_img(scr_hr, &img_heart_lg, 40, 75);

  lbl_hr_big = mk_label(scr_hr, "--", &lv_font_montserrat_30, 0xFFFFFF, 96, 64);
  mk_label(scr_hr, "bpm", &lv_font_montserrat_14, C_S3_MUTE, 151, 84);

  lbl_hr_zone = mk_label(scr_hr, "Zona: -- " TXT_DOT " -- target",
                         &lv_font_montserrat_10, C_PINK, 94, 103);

  static const int    zw[3] = { 36, 37, 37 };
  static const uint32_t zc[3] = { C_GREEN, C_YELLOW, C_RED };
  mark_hr = mk_zonebar(scr_hr, 94, 118, 110, 9, zw, zc, 3, 26);

  /* EKG */
  ecg_build();
  lv_obj_t *ln = lv_line_create(scr_hr);
  lv_line_set_points(ln, ecg_pts, ECG_PTS);
  lv_obj_set_pos(ln, 10, 139);
  lv_obj_set_style_line_color(ln, lv_color_hex(C_PINK), 0);
  lv_obj_set_style_line_width(ln, 2, 0);
  lv_obj_set_style_line_rounded(ln, false, 0);

  mk_label(scr_hr, "24 jam terakhir", &lv_font_montserrat_12, C_S3_MUTE, 10, 165);

  /* bar 12 jam terakhir, batang terakhir disorot */
  static const int bh[12] = { 17, 15, 19, 14, 23, 27, 21, 17, 25, 20, 16, 23 };
  for (int i = 0; i < 12; i++) {
    mk_box(scr_hr, 11 + i * 18, 214 - bh[i], 12, bh[i],
           i == 11 ? C_PINK : C_S3_BAR, 3);
  }

  mk_chips(scr_hr, "Ist. --", "Avg --", "Max --", C_S3_CARD, C_HR_TITLE, chip_hr);
}

/* ================= Halaman 4 : SpO2 ================= */
static lv_chart_series_t *sp_ser;
static lv_obj_t *sp_chart;

static void build_spo2(void) {
  scr_spo2 = lv_obj_create(NULL);
  lv_obj_remove_style_all(scr_spo2);
  lv_obj_set_style_bg_color(scr_spo2, lv_color_hex(C_S4_BG), 0);
  lv_obj_set_style_bg_opa(scr_spo2, LV_OPA_COVER, 0);
  lv_obj_clear_flag(scr_spo2, LV_OBJ_FLAG_SCROLLABLE);

  mk_deco(scr_spo2, 15, 65, 38, C_S4_DECO);
  mk_deco(scr_spo2, 215, 235, 48, C_S4_DECO);

  /* judul "SpO2" -- angka 2 dibuat subscript dengan label kecil terpisah */
  mk_header(scr_spo2, "SpO", C_S4_CARD, &NAV_MENU);
  mk_label(scr_spo2, "2", &lv_font_montserrat_10, 0xFFFFFF, 78, 21);

  mk_pill(scr_spo2, 152, 78, "terukur 10:20", C_S4_CARD, C_GREEN2);

  /* ring besar */
  ring_sp = lv_arc_create(scr_spo2);
  lv_obj_set_pos(ring_sp, 24, 56);
  lv_obj_set_size(ring_sp, 93, 93);
  lv_arc_set_rotation(ring_sp, 270);
  lv_arc_set_bg_angles(ring_sp, 0, 360);
  lv_arc_set_range(ring_sp, 0, 100);
  lv_arc_set_value(ring_sp, 0);
  lv_obj_remove_style(ring_sp, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(ring_sp, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_width(ring_sp, 9, LV_PART_MAIN);
  lv_obj_set_style_arc_width(ring_sp, 9, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(ring_sp, lv_color_hex(C_S4_DECO), LV_PART_MAIN);
  lv_obj_set_style_arc_color(ring_sp, lv_color_hex(C_BLUE), LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(ring_sp, true, LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(ring_sp, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(ring_sp, 0, 0);

  /* Ukuran font diambil dari mockup 4.png, bukan dikira-kira. Terukur di sana:
   * "98%" lebar 45.5 px dengan tinggi cap 15 px. montserrat_22 memberi 46.3 px
   * dan cap 16 px -- selisih di bawah 1 px. montserrat_30 yang dipakai
   * sebelumnya menghasilkan 63.1 px, 39% lebih lebar dari desainnya.
   *
   * Itu juga sebab "100%" meleset: pada font 30 lebarnya 76.4 px, sementara
   * ring hanya membentang x=24..117 dan kotak legenda mulai di x=132 -- jadi
   * angkanya keluar ring lalu menabrak legenda. Pada font 22 lebarnya 56.1 px,
   * masuk nyaman di ruang dalam ring yang di puncak angka hanya ~65 px. */
  lbl_sp_big = mk_label(scr_spo2, "--", &lv_font_montserrat_22, 0xFFFFFF, 0, 0);
  /* Lebar dipatok selebar ring dengan teks di-center. Tanpa ini angkanya
   * bergeser setiap kali jumlah digit berubah: lv_obj_align_to() menghitung
   * posisi SEKALI saat dipanggil -- dan saat itu isinya masih "--" (16.9 px) --
   * lalu tidak pernah menghitung ulang ketika label melebar. Dengan lebar
   * tetap, yang bergeser isi labelnya, bukan kotaknya. */
  lv_obj_set_width(lbl_sp_big, 93);
  lv_obj_set_style_text_align(lbl_sp_big, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align_to(lbl_sp_big, ring_sp, LV_ALIGN_CENTER, 0, -7);

  /* "oksigen" ikut dikecilkan ke montserrat_10 (40.2 px, mockup 37.5 px).
   * Kalau dibiarkan di montserrat_14 ia jadi 56.2 px -- LEBIH LEBAR dari
   * nilainya sendiri (46.3 px), sehingga hierarki visualnya terbalik.
   * dy +13, bukan +16: terukur dari mockup, pusat ink-nya +14 dari pusat ring
   * dan pusat ink label ada +1 px di bawah pusat kotaknya. */
  lv_obj_t *ok = mk_label(scr_spo2, "oksigen", &lv_font_montserrat_10, C_BLUE, 0, 0);
  lv_obj_align_to(ok, ring_sp, LV_ALIGN_CENTER, 0, 13);

  /* legend zona */
  static const char *lg[3] = { "95-100 normal", "90-94 rendah", "<90 kritis" };
  static const uint32_t lc[3] = { C_GREEN2, C_YELLOW, C_RED };
  for (int i = 0; i < 3; i++) {
    mk_box(scr_spo2, 132, 84 + i * 16, 9, 9, lc[i], 2);
    mk_label(scr_spo2, lg[i], &lv_font_montserrat_10, C_S4_MUTE, 146, 81 + i * 16);
  }

  mk_label(scr_spo2, "Riwayat 7 pengukuran", &lv_font_montserrat_12, C_S4_MUTE, 11, 157);

  /* grafik riwayat */
  sp_chart = lv_chart_create(scr_spo2);
  lv_obj_set_pos(sp_chart, 7, 172);
  lv_obj_set_size(sp_chart, 226, 34);
  lv_chart_set_type(sp_chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(sp_chart, 7);
  lv_chart_set_range(sp_chart, LV_CHART_AXIS_PRIMARY_Y, 92, 100);
  lv_chart_set_div_line_count(sp_chart, 0, 0);
  lv_obj_set_style_bg_opa(sp_chart, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(sp_chart, 0, 0);
  lv_obj_set_style_pad_all(sp_chart, 5, 0);
  lv_obj_set_style_line_width(sp_chart, 2, LV_PART_ITEMS);
  lv_obj_set_style_size(sp_chart, 6, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(sp_chart, lv_color_hex(C_BLUE), LV_PART_INDICATOR);
  lv_obj_clear_flag(sp_chart, LV_OBJ_FLAG_SCROLLABLE);
  sp_ser = lv_chart_add_series(sp_chart, lv_color_hex(C_BLUE), LV_CHART_AXIS_PRIMARY_Y);
  static const int spv[7] = { 95, 96, 95, 97, 96, 98, 98 };
  for (int i = 0; i < 7; i++) lv_chart_set_value_by_id(sp_chart, sp_ser, i, spv[i]);

  mk_chips(scr_spo2, "Min --", "Avg --", "--", C_S4_CARD, C_S4_TITLE, chip_sp);
}

/* ================= Halaman 5 : Glukosa ================= */
static lv_chart_series_t *gl_ser;
static lv_obj_t *gl_chart;

static void build_glu(void) {
  scr_glu = lv_obj_create(NULL);
  lv_obj_remove_style_all(scr_glu);
  lv_obj_set_style_bg_color(scr_glu, lv_color_hex(C_S5_BG), 0);
  lv_obj_set_style_bg_opa(scr_glu, LV_OPA_COVER, 0);
  lv_obj_clear_flag(scr_glu, LV_OBJ_FLAG_SCROLLABLE);

  mk_deco(scr_glu, 220, 55, 45, C_S5_DECO);
  mk_deco(scr_glu, 15, 235, 45, C_S5_DECO);

  mk_header(scr_glu, "Glukosa darah", C_S5_CARD, &NAV_MENU);
  mk_pill(scr_glu, 168, 72, "stabil " LV_SYMBOL_RIGHT, C_S5_CARD, C_GREEN2);

  lbl_glu_big = mk_label(scr_glu, "--", &font_digits_46, 0xFFFFFF, 12, 42);
  mk_label(scr_glu, "mg/dL", &lv_font_montserrat_14, C_S5_MUTE, 72, 66);

  /* Bar digeser ke x=122 (mockup: 110) karena montserrat_46 lebih lebar dari
   * font mockup, jadi "96" + "mg/dL" butuh 12 px ekstra. */
  static const int    gw[3] = { 19, 53, 25 };
  static const uint32_t gc[3] = { C_YELLOW, C_GREEN, C_RED };
  mark_glu = mk_zonebar(scr_glu, 122, 62, 97, 7, gw, gc, 3, 32);

  /* Teks ini semula mengklaim "Normal" tanpa melihat data. Sekarang mengikuti
   * nilai sebenarnya -- klaim klinis yang salah lebih buruk daripada "--". */
  lbl_glu_status = mk_label(scr_glu, "Menunggu pengukuran",
                            &lv_font_montserrat_10, C_GREEN2, 10, 91);

  mk_label(scr_glu, "Tren hari ini", &lv_font_montserrat_12, C_S5_MUTE, 10, 111);

  /* panel + grafik tren */
  mk_box(scr_glu, 10, 128, 224, 30, C_S5_CARD, 6);
  gl_chart = lv_chart_create(scr_glu);
  lv_obj_set_pos(gl_chart, 10, 126);
  lv_obj_set_size(gl_chart, 224, 40);
  lv_chart_set_type(gl_chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(gl_chart, 7);
  lv_chart_set_range(gl_chart, LV_CHART_AXIS_PRIMARY_Y, 80, 132);
  lv_chart_set_div_line_count(gl_chart, 0, 0);
  lv_obj_set_style_bg_opa(gl_chart, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(gl_chart, 0, 0);
  lv_obj_set_style_pad_all(gl_chart, 5, 0);
  lv_obj_set_style_line_width(gl_chart, 2, LV_PART_ITEMS);
  lv_obj_set_style_size(gl_chart, 6, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(gl_chart, lv_color_hex(C_AMBER2), LV_PART_INDICATOR);
  lv_obj_clear_flag(gl_chart, LV_OBJ_FLAG_SCROLLABLE);
  gl_ser = lv_chart_add_series(gl_chart, lv_color_hex(C_AMBER2), LV_CHART_AXIS_PRIMARY_Y);
  static const int glv[7] = { 86, 90, 124, 96, 104, 112, 108 };
  for (int i = 0; i < 7; i++) lv_chart_set_value_by_id(gl_chart, gl_ser, i, glv[i]);

  /* label sumbu waktu */
  mk_label(scr_glu, "06.00", &lv_font_montserrat_10, C_S5_MUTE, 10, 175);
  lv_obj_t *m = mk_label(scr_glu, "08.00", &lv_font_montserrat_10, C_S5_MUTE, 0, 175);
  lv_obj_set_width(m, SCREEN_W);
  lv_obj_set_style_text_align(m, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_t *r = mk_label(scr_glu, "10.00", &lv_font_montserrat_10, C_S5_MUTE, 0, 175);
  lv_obj_set_width(r, 230);
  lv_obj_set_style_text_align(r, LV_TEXT_ALIGN_RIGHT, 0);

  mk_chips(scr_glu, "Min --", "Maks --", "PI --", C_S5_CARD, C_GL_TITLE, chip_gl);
}

/* ================= Halaman 6 : Tekanan darah =================
 * Tidak ada mockup untuk halaman ini (fitur ditambahkan belakangan). Geometri
 * di bawah SENGAJA meniru rima vertikal halaman 5 (Glukosa) piksel demi piksel
 * -- itu satu-satunya halaman yang sudah terbukti muat di layar 240x280 dengan
 * struktur setara (angka besar + bar zona + status + tren + chip), jadi lebih
 * aman menyalin proporsinya daripada menerka ulang dari nol. */
static lv_chart_series_t *bp_ser_sbp, *bp_ser_dbp;
static lv_obj_t *bp_chart;

static void build_bp(void) {
  scr_bp = lv_obj_create(NULL);
  lv_obj_remove_style_all(scr_bp);
  lv_obj_set_style_bg_color(scr_bp, lv_color_hex(C_S6_BG), 0);
  lv_obj_set_style_bg_opa(scr_bp, LV_OPA_COVER, 0);
  lv_obj_clear_flag(scr_bp, LV_OBJ_FLAG_SCROLLABLE);

  mk_deco(scr_bp, 220, 55, 45, C_S6_DECO);
  mk_deco(scr_bp, 15, 235, 45, C_S6_DECO);

  mk_header(scr_bp, "Tekanan darah", C_S6_CARD, &NAV_MENU);
  /* Lencana permanen, bukan data yang bisa jadi basi seperti pill "terukur
   * 10:20" di halaman SpO2 -- di sini lebih penting selalu terlihat mengingat
   * modelnya bahkan belum dapat satu titik kalibrasi pun (lihat ppg.cpp). */
  mk_pill(scr_bp, 152, 78, "eksperimental", C_S6_CARD, C_S6_MUTE);

  /* "120/80" pakai montserrat_30 biasa, bukan font_digits: font_digits cuma
   * berisi glyph 0-9 dan '-', tidak ada '/' untuk memisahkan sistol/diastol. */
  lbl_bp_big = mk_label(scr_bp, "--/--", &lv_font_montserrat_30, 0xFFFFFF, 12, 42);
  mk_label(scr_bp, "mmHg", &lv_font_montserrat_14, C_S6_MUTE, 150, 60);

  static const int    bw[4] = { 14, 40, 25, 18 };
  static const uint32_t bc[4] = { C_BLUE, C_GREEN, C_YELLOW, C_RED };
  mark_bp = mk_zonebar(scr_bp, 122, 62, 97, 7, bw, bc, 4, 32);

  lbl_bp_status = mk_label(scr_bp, "Menunggu pengukuran",
                           &lv_font_montserrat_10, C_S6_MUTE, 10, 91);

  mk_label(scr_bp, "Tren hari ini", &lv_font_montserrat_12, C_S6_MUTE, 10, 111);

  /* panel + grafik tren, dua seri (sistol/diastol) */
  mk_box(scr_bp, 10, 128, 224, 30, C_S6_CARD, 6);
  bp_chart = lv_chart_create(scr_bp);
  lv_obj_set_pos(bp_chart, 10, 126);
  lv_obj_set_size(bp_chart, 224, 40);
  lv_chart_set_type(bp_chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(bp_chart, 7);
  lv_chart_set_range(bp_chart, LV_CHART_AXIS_PRIMARY_Y, 50, 160);
  lv_chart_set_div_line_count(bp_chart, 0, 0);
  lv_obj_set_style_bg_opa(bp_chart, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(bp_chart, 0, 0);
  lv_obj_set_style_pad_all(bp_chart, 5, 0);
  lv_obj_set_style_line_width(bp_chart, 2, LV_PART_ITEMS);
  /* Titik indikator dimatikan (size 0), beda dengan chart SpO2/glukosa yang
   * cuma satu seri: dengan dua seri di sini titiknya akan memakai satu warna
   * default yang sama untuk sistol maupun diastol dan malah membingungkan --
   * warna garis saja sudah cukup membedakan keduanya. */
  lv_obj_set_style_size(bp_chart, 0, LV_PART_INDICATOR);
  lv_obj_clear_flag(bp_chart, LV_OBJ_FLAG_SCROLLABLE);
  bp_ser_sbp = lv_chart_add_series(bp_chart, lv_color_hex(C_PURPLE), LV_CHART_AXIS_PRIMARY_Y);
  bp_ser_dbp = lv_chart_add_series(bp_chart, lv_color_hex(C_PURPLE2), LV_CHART_AXIS_PRIMARY_Y);
  static const int sbpv[7] = { 118, 121, 125, 119, 123, 127, 120 };
  static const int dbpv[7] = { 76,  78,  80,  77,  79,  82,  78  };
  for (int i = 0; i < 7; i++) {
    lv_chart_set_value_by_id(bp_chart, bp_ser_sbp, i, sbpv[i]);
    lv_chart_set_value_by_id(bp_chart, bp_ser_dbp, i, dbpv[i]);
  }

  /* label sumbu waktu */
  mk_label(scr_bp, "06.00", &lv_font_montserrat_10, C_S6_MUTE, 10, 175);
  lv_obj_t *m = mk_label(scr_bp, "08.00", &lv_font_montserrat_10, C_S6_MUTE, 0, 175);
  lv_obj_set_width(m, SCREEN_W);
  lv_obj_set_style_text_align(m, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_t *r = mk_label(scr_bp, "10.00", &lv_font_montserrat_10, C_S6_MUTE, 0, 175);
  lv_obj_set_width(r, 230);
  lv_obj_set_style_text_align(r, LV_TEXT_ALIGN_RIGHT, 0);

  mk_chips(scr_bp, "Sistol --", "Diastol --", "Nadi --", C_S6_CARD, C_S6_TITLE, chip_bp);
}

/* ================= Timer: perbarui isi layar =================
 * Semua label diperbarui HANYA saat nilainya berubah. lv_label_set_text()
 * selalu meng-invalidate area labelnya, dan beberapa di antaranya memakai
 * montserrat_46/48 -- redraw dua kali per detik tanpa alasan membuat animasi
 * transisi layar tersendat.
 */

/* Tulis label hanya kalau isinya beda. Mengembalikan true kalau berubah. */
static bool set_if_changed(lv_obj_t *lbl, const char *txt) {
  const char *cur = lv_label_get_text(lbl);
  if (cur && strcmp(cur, txt) == 0) return false;
  lv_label_set_text(lbl, txt);
  return true;
}

/* Perbarui seluruh tampilan kesehatan.
 *
 * Sumbernya jam_snapshot(), bukan ppg_get() langsung: sejak MAX30105 hanya
 * menyala selama pengukuran, membaca sensor langsung berarti keempat halaman
 * detail menampilkan "--" hampir sepanjang waktu. jam_snapshot() mengisinya
 * dengan pengukuran yang sedang berjalan, atau hasil pengukuran terakhir kalau
 * tidak ada yang berjalan -- sementara statistik sesi (chip min/avg/maks) tetap
 * datang dari sensor apa adanya, karena angka itu hanya berarti selama sensor
 * benar-benar mencacah. */
static void update_health_ui(void) {
  ppg_data_t p;
  jam_snapshot(&p);
  /* 48 byte, bukan 24: "Zona: rendah <bullet> 100% target" = 28 byte (bullet
   * UTF-8 3 byte) dan "Estimasi dalam rentang (70-140)" = 31 byte. */
  char b[48];

  /* ---- kartu menu ---- */
  if (p.bpm_valid) snprintf(b, sizeof(b), "%d bpm", (int)lroundf(p.bpm));
  else             snprintf(b, sizeof(b), "-- bpm");
  set_if_changed(lbl_card_hr, b);

  if (p.spo2_valid) snprintf(b, sizeof(b), "%d %%", (int)lroundf(p.spo2));
  else              snprintf(b, sizeof(b), "-- %%");
  set_if_changed(lbl_card_sp, b);

  if (p.glu_valid) snprintf(b, sizeof(b), "%d mg/dL", (int)lroundf(p.glucose));
  else             snprintf(b, sizeof(b), "-- mg/dL");
  set_if_changed(lbl_card_gl, b);

  if (p.bp_valid) snprintf(b, sizeof(b), "%d/%d mmHg",
                           (int)lroundf(p.sbp), (int)lroundf(p.dbp));
  else            snprintf(b, sizeof(b), "--/-- mmHg");
  set_if_changed(lbl_card_bp, b);

  /* ---- halaman 3: heart rate ---- */
  if (p.bpm_valid) {
    int bpm = (int)lroundf(p.bpm);
    snprintf(b, sizeof(b), "%d", bpm);
    set_if_changed(lbl_hr_big, b);
    lv_arc_set_value(arc_hr, bpm > 100 ? 100 : bpm);

    /* Zona dihitung dari nilai sebenarnya, bukan teks tetap. */
    const char *zona = bpm < 60  ? "rendah"
                     : bpm < 100 ? "ringan"
                     : bpm < 140 ? "sedang" : "berat";
    int target = (int)((bpm * 100L) / 190);      /* 190 = perkiraan HR maks */
    snprintf(b, sizeof(b), "Zona: %s " TXT_DOT " %d%% target", zona, target);
    set_if_changed(lbl_hr_zone, b);
    lv_obj_set_x(mark_hr, 94 + (bpm > 200 ? 110 : bpm * 110 / 200));
  } else {
    set_if_changed(lbl_hr_big, "--");
    lv_arc_set_value(arc_hr, 0);
    set_if_changed(lbl_hr_zone, "Zona: -- " TXT_DOT " -- target");
    /* Penanda dikembalikan ke posisi awalnya. Kalau tidak, ia tetap menunjuk
     * nilai terakhir di bar zona sementara angkanya sudah "--" -- bar yang
     * menunjuk sesuatu itu klaim, dan saat sensor dimatikan klaim itu palsu. */
    lv_obj_set_x(mark_hr, 94 + 26);
  }

  /* ---- halaman 4: SpO2 ---- */
  if (p.spo2_valid) {
    int sp = (int)lroundf(p.spo2);
    snprintf(b, sizeof(b), "%d%%", sp);
    set_if_changed(lbl_sp_big, b);
    lv_arc_set_value(ring_sp, sp);
  } else {
    set_if_changed(lbl_sp_big, "--");
    lv_arc_set_value(ring_sp, 0);
  }

  /* ---- halaman 5: glukosa (EKSPERIMENTAL) ---- */
  if (p.glu_valid) {
    int gl = (int)lroundf(p.glucose);
    snprintf(b, sizeof(b), "%d", gl);
    set_if_changed(lbl_glu_big, b);
    /* Status mengikuti nilai. Sengaja TIDAK memakai centang "Normal" seperti
     * sebelumnya: nilainya berasal dari model yang belum tervalidasi, jadi
     * klaim klinis tidak pantas. Kata "estimasi" dipertahankan agar terlihat. */
    snprintf(b, sizeof(b), "Estimasi %s (70-140)",
             gl < 70 ? "rendah" : gl <= 140 ? "dalam rentang" : "tinggi");
    set_if_changed(lbl_glu_status, b);
    lv_obj_set_x(mark_glu, 122 + (gl < 40 ? 0 : gl > 240 ? 97
                                  : (gl - 40) * 97 / 200));
  } else {
    set_if_changed(lbl_glu_big, "--");
    set_if_changed(lbl_glu_status, "Menunggu pengukuran");
    lv_obj_set_x(mark_glu, 122 + 32);
  }

  /* ---- halaman 6: tekanan darah (EKSPERIMENTAL) ---- */
  if (p.bp_valid) {
    int sb = (int)lroundf(p.sbp);
    int db = (int)lroundf(p.dbp);
    snprintf(b, sizeof(b), "%d/%d", sb, db);
    set_if_changed(lbl_bp_big, b);
    /* Status mengikuti sistol saja, sama seperti zona HR yang cuma memakai
     * satu angka -- diastol tetap terlihat di angka besar & chip, cukup
     * tidak ikut menentukan kategori supaya logikanya tetap sederhana. */
    const char *kat = sb < 90  ? "rendah"
                    : sb < 120 ? "normal"
                    : sb < 140 ? "meningkat" : "tinggi";
    snprintf(b, sizeof(b), "Estimasi %s (normal ~90-120)", kat);
    set_if_changed(lbl_bp_status, b);
    int sb_clamped = sb < 70 ? 70 : (sb > 160 ? 160 : sb);
    lv_obj_set_x(mark_bp, 122 + (sb_clamped - 70) * 97 / 90);
  } else {
    set_if_changed(lbl_bp_big, "--/--");
    set_if_changed(lbl_bp_status, "Menunggu pengukuran");
    lv_obj_set_x(mark_bp, 122 + 32);
  }

  /* ---- chip statistik sesi ---- */
  if (p.stats_valid) {
    snprintf(b, sizeof(b), "Ist. %d", p.bpm_min);  set_if_changed(chip_hr[0], b);
    snprintf(b, sizeof(b), "Avg %d",  p.bpm_avg);  set_if_changed(chip_hr[1], b);
    snprintf(b, sizeof(b), "Max %d",  p.bpm_max);  set_if_changed(chip_hr[2], b);

    snprintf(b, sizeof(b), "Min %d", p.spo2_min);  set_if_changed(chip_sp[0], b);
    snprintf(b, sizeof(b), "Avg %d", p.spo2_avg);  set_if_changed(chip_sp[1], b);
    set_if_changed(chip_sp[2], p.spo2_min >= 95 ? "Normal" : "Rendah");

    snprintf(b, sizeof(b), "Min %d",  p.glu_min);  set_if_changed(chip_gl[0], b);
    snprintf(b, sizeof(b), "Maks %d", p.glu_max);  set_if_changed(chip_gl[1], b);
    snprintf(b, sizeof(b), "PI %.1f", p.pi);       set_if_changed(chip_gl[2], b);

    snprintf(b, sizeof(b), "Sistol %d",  p.sbp_avg); set_if_changed(chip_bp[0], b);
    snprintf(b, sizeof(b), "Diastol %d", p.dbp_avg); set_if_changed(chip_bp[1], b);
    /* Tekanan nadi (pulse pressure) = sistol - diastol, dari rata-rata sesi
     * -- bukan chip generik ketiga seperti "PI" di glukosa, karena angka ini
     * memang turunan langsung dua kolom di atasnya, bukan fitur kalibrasi. */
    snprintf(b, sizeof(b), "Nadi %d", p.sbp_avg - p.dbp_avg); set_if_changed(chip_bp[2], b);
  } else {
    /* Tanpa cabang ini chip membeku di angka sesi sebelumnya. Paling terasa
     * saat sensor dimatikan lewat tombol daya: angka utama jadi "--" sementara
     * "Avg 72" tetap terpampang di bawahnya. Teksnya dikembalikan persis ke
     * yang dipasang mk_chips() saat layar dibangun. */
    set_if_changed(chip_hr[0], "Ist. --");
    set_if_changed(chip_hr[1], "Avg --");
    set_if_changed(chip_hr[2], "Max --");

    set_if_changed(chip_sp[0], "Min --");
    set_if_changed(chip_sp[1], "Avg --");
    set_if_changed(chip_sp[2], "--");

    set_if_changed(chip_gl[0], "Min --");
    set_if_changed(chip_gl[1], "Maks --");
    set_if_changed(chip_gl[2], "PI --");

    set_if_changed(chip_bp[0], "Sistol --");
    set_if_changed(chip_bp[1], "Diastol --");
    set_if_changed(chip_bp[2], "Nadi --");
  }

  /* LED "live" hanya berkedip saat pengukuran benar-benar berjalan. */
  static bool blink = false;
  blink = !blink;
  if (p.state == PPG_STABLE)
    lv_obj_set_style_opa(dot_live, blink ? LV_OPA_COVER : LV_OPA_40, 0);
  else
    lv_obj_set_style_opa(dot_live, LV_OPA_20, 0);
}

/* ================= Tampilan sesi & koneksi =================
 * Daftar keadaan yang wajib punya tampilan ada di dokumen 14. Yang TIDAK
 * diikuti dari sana cuma satu, dan sengaja: dokumen meminta layar menandai
 * "waktu tidak pasti" selama anchor belum terpasang, karena jam acuannya tidak
 * punya RTC dan jam dindingnya memang tebakan. Jam INI punya PCF85063 + NTP,
 * jadi angka jam di layar sudah benar tanpa anchor mana pun dan menandainya
 * "tidak pasti" justru berbohong ke arah sebaliknya. Yang tetap diikuti persis:
 * di KAWAT jam tidak pernah mengirim wall clock -- hanya uptime_s + boot_id --
 * dan flag anchor tetap dilaporkan apa adanya di paket Info & Status.
 */
static void update_sesi_ui(void) {
  /* ---- ikon Bluetooth: tiga keadaan ---- */
  static int last_ble = -1;
  int c = jam_siap_notifikasi() ? 2 : (jam_terhubung() ? 1 : 0);
  if (c != last_ble) {
    last_ble = c;
    lv_obj_set_style_text_color(
      lbl_ble, lv_color_hex(c == 2 ? C_BLUE : c == 1 ? C_S4_MUTE : C_S1_DIV), 0);
  }

  /* ---- entri tertunda ---- */
  static int last_pending = -1;
  int pend = jam_tertunda();
  if (pend != last_pending) {
    last_pending = pend;
    if (pend > 0) {
      lv_label_set_text_fmt(lbl_pending, "%d", pend);
      lv_obj_clear_flag(lbl_pending, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(lbl_pending, LV_OBJ_FLAG_HIDDEN);
    }
  }

  /* ---- pindah layar otomatis saat mengukur ----
   * Sekali-jalan dengan percobaan ulang, bukan dipaksa tiap tick: go() menolak
   * permintaan yang datang saat animasi pindah layar masih berjalan, jadi satu
   * panggilan saja bisa hilang begitu saja. Setelah layarnya sampai, keinginan
   * dilepas -- kalau tidak, pengguna yang membuka menu selagi mengukur akan
   * ditarik kembali tiap setengah detik. */
  static bool last_ukur = false;
  static lv_obj_t *scr_sebelum = NULL;
  static lv_obj_t *scr_ingin = NULL;
  static bool punya_lalu[4] = { false, false, false, false };
  bool ukur = jam_sedang_mengukur();
  if (ukur != last_ukur) {
    last_ukur = ukur;
    if (ukur) {
      /* Keempat titik dikembalikan abu-abu di AWAL tiap pengukuran. Tanpa ini
       * titik hijau dari pengukuran sebelumnya terbawa: pengukuran baru mulai
       * dengan tampilan "semua sudah dapat" padahal belum satu pun. */
      for (int i = 0; i < 4; i++) {
        punya_lalu[i] = false;
        lv_obj_set_style_bg_color(dot_meas[i], lv_color_hex(C_S1_DIV), 0);
        lv_obj_set_style_text_color(lbl_meas_metrik[i], lv_color_hex(C_S1_MUTED), 0);
      }
      scr_sebelum = lv_scr_act();
      scr_ingin = scr_meas;
    } else {
      scr_ingin = (scr_sebelum && scr_sebelum != scr_meas) ? scr_sebelum : scr_home;
    }
  }
  if (scr_ingin) {
    if (lv_scr_act() == scr_ingin) scr_ingin = NULL;
    else go(scr_ingin, ukur, ukur ? "sedang mengukur" : "selesai mengukur");
  }

  /* ---- isi layar pengukuran ---- */
  if (ukur) {
    char b[32];
    /* Judulnya menyebut terus terang pengukuran mana ini. Kedua jenis memakai
     * layar yang sama persis, jadi baris inilah satu-satunya yang memberi tahu
     * pengguna apakah angkanya akan sampai ke aplikasi atau tidak. */
    if (jam_ukur_lokal())           snprintf(b, sizeof(b), "Cek manual");
    else if (jam_ukur_index() == 0) snprintf(b, sizeof(b), "Pengukuran awal");
    else snprintf(b, sizeof(b), "Pengukuran ke-%u", (unsigned)jam_ukur_index());
    set_if_changed(lbl_meas_judul, b);
    set_if_changed(lbl_meas_sub, jam_ukur_lokal()
                     ? "Hasil hanya tampil di jam, tidak dikirim"
                     : "Tempelkan jari, jangan bergerak");

    const bool punya[4] = { jam_ukur_punya_bpm(), jam_ukur_punya_spo2(),
                            jam_ukur_punya_glukosa(), jam_ukur_punya_tensi() };
    for (int i = 0; i < 4; i++) {
      if (punya[i] == punya_lalu[i]) continue;
      punya_lalu[i] = punya[i];
      lv_obj_set_style_bg_color(dot_meas[i],
                                lv_color_hex(punya[i] ? C_GREEN2 : C_S1_DIV), 0);
      lv_obj_set_style_text_color(lbl_meas_metrik[i],
                                  lv_color_hex(punya[i] ? C_WHITE : C_S1_MUTED), 0);
    }

    /* Baris detak jantung membawa cacahannya. Ini baris yang paling lama
     * menunggu, jadi ia yang harus menjelaskan apa yang sedang ditunggu --
     * tanpa angka ini, layar diam belasan detik tanpa alasan yang terlihat. */
    uint16_t dtk = jam_ukur_detak(), perlu = jam_ukur_detak_perlu();
    if (dtk >= perlu) snprintf(b, sizeof(b), "Detak jantung");
    else snprintf(b, sizeof(b), "Detak jantung %u/%u", (unsigned)dtk, (unsigned)perlu);
    set_if_changed(lbl_meas_metrik[0], b);

    /* Bar mengikuti detak, bukan detik: sejak batas waktu dilepas, waktu
     * berjalan tidak lagi mengukur kemajuan apa pun. */
    lv_obj_set_width(bar_meas, perlu ? (200 * (int)(dtk > perlu ? perlu : dtk) / (int)perlu) : 0);

    snprintf(b, sizeof(b), "%us", (unsigned)jam_ukur_detik());
    set_if_changed(lbl_meas_waktu, b);
  }

  /* ---- pil status sesi di home ----
   * Pesan penolakan harus menyebut alasannya. "Belum disiapkan aplikasi" untuk
   * tombol cek manual yang terkunci karena sesi berjalan bukan cuma tidak
   * membantu -- ia menyesatkan ke arah yang berlawanan. */
  static uint32_t tolak_sampai_ms = 0;
  static const char *tolak_teks = "";
  uint8_t tolak = jam_umpan_balik_ditolak();
  if (tolak != JAM_TOLAK_TIDAK_ADA) {
    tolak_sampai_ms = millis() + 2500;
    switch (tolak) {
      case JAM_TOLAK_BELUM_ARM:    tolak_teks = "Belum disiapkan aplikasi"; break;
      case JAM_TOLAK_SESI_AKTIF:   tolak_teks = "Cek manual terkunci saat sesi"; break;
      case JAM_TOLAK_SESI_BERJALAN: tolak_teks = "Sesi sudah berjalan"; break;
      case JAM_TOLAK_SEDANG_UKUR:  tolak_teks = "Masih mengukur"; break;
      case JAM_TOLAK_BATERAI:      tolak_teks = "Baterai terlalu lemah"; break;
      case JAM_TOLAK_SENSOR:       tolak_teks = "Sensor tidak terdeteksi"; break;
      default:                     tolak_teks = ""; break;
    }
  }

  char b[64];
  uint32_t warna;
  if ((int32_t)(millis() - tolak_sampai_ms) < 0) {
    snprintf(b, sizeof(b), "%s", tolak_teks);
    warna = C_AMBER;
  } else if (jam_status() == AW_SESI_ARMED) {
    snprintf(b, sizeof(b), "Siap " TXT_DOT " tekan setelah makan");
    warna = C_GREEN2;
  } else if (jam_status() == AW_SESI_RUNNING) {
    uint32_t t0 = jam_t0_uptime(), skrg = jam_uptime();
    uint32_t target = (skrg < t0 + AW_JADWAL_IDX2_S) ? t0 + AW_JADWAL_IDX2_S
                                                     : t0 + AW_JADWAL_IDX3_S;
    int ke = (skrg < t0 + AW_JADWAL_IDX2_S) ? 2 : 3;
    /* Hitung mundur selalu dari uptime_s absolut, tidak pernah dari sisa waktu
     * yang diakumulasikan sendiri (dokumen 12 & 14). */
    long sisa = (long)target - (long)skrg;
    if (sisa < 0) sisa = 0;
    if (sisa >= 60) snprintf(b, sizeof(b), "Ukur ke-%d dalam %ld mnt", ke, sisa / 60);
    else            snprintf(b, sizeof(b), "Ukur ke-%d dalam %ld dtk", ke, sisa);
    warna = C_AMBER;
  } else {
    /* IDLE. Pil ini yang memberi tahu peran tombol sekarang -- tanpanya, satu
     * tombol yang mengerjakan dua hal berbeda hanya bisa dihafal, dan pengguna
     * yang salah hafal akan mengira cek manualnya terkirim ke aplikasi. */
    snprintf(b, sizeof(b), "Tekan untuk cek manual");
    warna = C_S1_MUTED;
  }
  if (set_if_changed(lbl_sesi, b))
    lv_obj_set_style_text_color(lbl_sesi, lv_color_hex(warna), 0);
}

static void refresh_cb(lv_timer_t *tm) {
  (void)tm;
  /* Konteks loop: di sinilah RTC dibaca dan hasil NTP diterapkan. */
  tm_tick();

  /* ---- halaman 1: jam & tanggal dari RTC/NTP ----
   * Timer ini 500 ms, tapi label hanya disentuh saat nilainya benar-benar
   * berubah. lv_label_set_text() selalu meng-invalidate area labelnya, dan
   * jam itu montserrat_48 -- redraw dua kali per detik tanpa alasan bikin
   * animasi transisi layar tersendat. */
  struct tm t;
  bool have_time = tm_now(&t);
  if (have_time) {
    static int last_min = -1, last_mday = -1;

    if (t.tm_min != last_min) {
      last_min = t.tm_min;
      lv_label_set_text_fmt(lbl_hh, "%02d", t.tm_hour);
      lv_label_set_text_fmt(lbl_mm, "%02d", t.tm_min);
    }
    /* Hari dan tanggal satu label, jadi satu penulisan. Tetap dikunci ke
     * perubahan tm_mday: hari dan tanggal selalu berganti bersamaan. */
    if (t.tm_mday != last_mday) {
      last_mday = t.tm_mday;
      lv_label_set_text_fmt(lbl_date, "%s " TXT_DOT " %d %s %d",
                            tm_day_name(t.tm_wday), t.tm_mday,
                            tm_month_name(t.tm_mon), t.tm_year + 1900);
    }
  }

  /* ---- header: cuaca & suhu dari OpenWeatherMap ---- */
  weather_t w;
  weather_get(&w);
  if (w.valid) {
    static int  last_temp = -999;
    static char last_cond[16] = "";
    if (w.temp_c != last_temp) {
      last_temp = w.temp_c;
      lv_label_set_text_fmt(lbl_wthr, "%d" TXT_DEG "C", w.temp_c);
    }
    if (strcmp(w.cond, last_cond) != 0) {
      strncpy(last_cond, w.cond, sizeof(last_cond) - 1);
      lv_label_set_text(lbl_cond, w.cond);
    }
  }

  battery_update();
  if (battery_valid()) {
    static int last_batt = -1;
    int bp = battery_percent();
    if (bp != last_batt) {
      last_batt = bp;
      lv_label_set_text_fmt(lbl_batt, "%d%%", bp);
    }
  }

  update_health_ui();
  update_sesi_ui();

  /* Tombol disamakan dengan status sesi yang sebenarnya. Perlu karena status
   * itu bertahan saat pengguna pindah halaman -- dan karena ia juga berubah
   * tanpa jari: ARM datang dari aplikasi, dan kembali ke IDLE bisa terjadi
   * karena timeout 4 jam. Fungsinya sendiri tidak melakukan apa-apa kalau
   * tidak ada perubahan. */
  utama_btn_refresh();

  /* Heartbeat tiap 5 s. Sengaja jarang: USB CDC board ini re-enumerate setelah
   * reset sehingga print di setup() hampir selalu hilang, jadi baris inilah
   * bukti UI benar-benar jalan -- sekalian statistik touch untuk debug. */
  static uint8_t n = 0;
  if (++n >= 10) {
    n = 0;

    /* Diagnostik boot dicetak di heartbeat PERTAMA, bukan di setup(): USB CDC
     * board ini re-enumerate setelah reset sehingga apa pun yang dicetak
     * setup() hampir selalu hilang sebelum host siap menerima. */
    static bool diag_done = false;
    if (!diag_done) {
      diag_done = true;
      Serial.printf("[diag] PCF85063 terdeteksi: %s\n", rtc_ok() ? "YA" : "TIDAK");
      Serial.printf("[diag] waktu saat boot: %s\n", tm_boot_info());
      rtc_scan_bus();
    }

    static const char *SRC[] = { "none", "rtc", "ntp" };
    Serial.printf("[hb] %02d:%02d:%02d src=%s wifi=%d ble=%d sesi=%d tunda=%d  "
                  "cuaca=%s %dC  "
                  "touch irq=%lu err=%lu evt=%lu  heap=%lu\n",
                  have_time ? t.tm_hour : 0, have_time ? t.tm_min : 0,
                  have_time ? t.tm_sec : 0,
                  SRC[tm_source()], net_connected() ? 1 : 0,
                  jam_siap_notifikasi() ? 2 : (jam_terhubung() ? 1 : 0),
                  (int)jam_status(), (int)jam_tertunda(),
                  w.valid ? w.cond : "--", w.temp_c,
                  (unsigned long)touch_irq_count, (unsigned long)touch_readerr,
                  (unsigned long)touch_events, (unsigned long)ESP.getFreeHeap());
    ppg_data_t pp; ppg_get(&pp);
    long dir, dred, dthr; uint32_t dn, dp;
    ppg_diag(&dir, &dred, &dn, &dthr, &dp);
    Serial.printf("[ppg]  %s%s  bpm=%s%.0f  spo2=%s%.1f  glukosa*=%s%.0f  "
                  "td*=%s%.0f/%.0f\n",
                  ppg_state_text(),
                  pp.held ? " [tahan]" : "",
                  pp.bpm_valid ? "" : "(-)", pp.bpm,
                  pp.spo2_valid ? "" : "(-)", pp.spo2,
                  pp.glu_valid ? "" : "(-)", pp.glucose,
                  pp.bp_valid ? "" : "(-)", pp.sbp, pp.dbp);
    /* Baris mentah: ir/red dan sampel. sampel yang diam di satu angka berarti
     * FIFO tidak menghasilkan apa pun -- biasanya sensor tidak dicatu daya. */
    Serial.printf("[ppg] mentah ir=%ld red=%ld (ambang %ld)  sampel=%lu  poll=%lu\n",
                  dir, dred, dthr, (unsigned long)dn, (unsigned long)dp);
    Serial.printf("[batt] counts=%d/4095  raw=%d mV (sebaran %d mV)  "
                  "baterai=%d mV  floor=%d mV%s  %d%%\n",
                  battery_raw_counts(), battery_raw_millivolts(),
                  battery_spread_mv(), battery_millivolts(),
                  battery_floor_mv(), battery_charging() ? " [mengisi]" : "",
                  battery_percent());
    if (battery_history_count() > 1) {
      /* 192, bukan 128: riwayat kini 30 titik x 5 karakter = 150. battery_history()
       * memang memotong dengan aman, tapi 128 akan memangkas sepertiga datanya. */
      char hb[192];
      battery_history(hb, sizeof(hb));
      Serial.printf("[batt] riwayat/menit (mV di pin, tertua dulu): %s\n", hb);
    }
  }
}

/* ================= Tombol PWR: tekan sekali hidup, tekan lagi mati =========
 * Menyalakan board BUKAN pekerjaan firmware dan tidak akan pernah bisa jadi:
 * saat board mati, tidak ada yang berjalan untuk mendengarkan tombol. Itu
 * murni kerja rangkaian -- menekan PWR menyambungkan baterai, board boot, lalu
 * setup() mengunci BAT_EN sehingga jalurnya tidak lagi bergantung pada jari.
 *
 * Yang dikerjakan di sini cuma separuh keduanya: mendengarkan tekanan
 * BERIKUTNYA, lalu melepas latch itu.
 *
 * TEKANAN PERTAMA HARUS DIABAIKAN, dan ini jebakan yang tidak kelihatan sampai
 * dicoba: tekanan yang menyalakan board masih berlangsung saat loop() mulai
 * jalan. Kalau tombol langsung didengarkan, tekanan itu terbaca sebagai
 * "tekan lagi" dan board mematikan dirinya sendiri sepersekian detik setelah
 * menyala -- terlihat persis seperti board yang gagal boot. Karena itu tombol
 * baru aktif setelah pernah terlihat DILEPAS. BSP Waveshare menyelesaikan ini
 * dengan cara yang sama (menunggu di while sebelum memasang handler).
 */
#define PWR_DEBOUNCE_MS  50   /* level harus stabil selama ini sebelum dipercaya */

static bool     pwr_siap = false;      /* tombol sudah pernah dilepas sejak boot */
static int      pwr_level_lalu = HIGH;
static uint32_t pwr_stabil_ms = 0;

static void pwr_matikan(void) {
  Serial.println("[pwr] tombol PWR ditekan -- mematikan");

  /* Layar dipadamkan lebih dulu: jari pengguna umumnya masih menempel di
   * tombol pada titik ini, dan layar yang langsung gelap adalah umpan balik
   * bahwa tekanannya diterima. */
  ledcWrite(LCD_BL, 0);

  /* Sensor padam + ring buffer dipaksa tersimpan. Harus SEBELUM latch dilepas:
   * aw_store menunda tulis flash 3 detik, jadi tanpa ini setiap entri dari 3
   * detik terakhir -- termasuk sampel yang baru saja selesai diukur -- ikut
   * hilang bersama dayanya. */
  jam_siap_mati();

  /* Lepas latch. Board TIDAK langsung padam kalau jari masih menekan: selama
   * itu tombolnya sendiri yang menyambungkan baterai. Padamnya terjadi saat
   * jari diangkat, dan itu memang perilaku yang benar -- bukan keterlambatan
   * yang perlu diakali. */
  digitalWrite(BAT_EN, LOW);

  /* Tidak ada jalan kembali dari sini, jadi jangan biarkan loop() melanjutkan
   * seolah tidak terjadi apa-apa. */
  uint32_t t0 = millis();
  bool dicatat = false;
  for (;;) {
    if (!dicatat && (uint32_t)(millis() - t0) >= 2000) {
      dicatat = true;
      /* Masih hidup 2 detik setelah latch dilepas berarti dayanya datang dari
       * USB, bukan baterai -- melepas BAT_EN tidak memutus jalur itu. Board
       * sengaja dibiarkan gelap dan diam: itu keadaan "mati" paling jujur yang
       * bisa dicapai selama USB tertancap. Karena itu pengujian tombol ini
       * HARUS dilakukan dengan USB tercabut. */
      Serial.println("[pwr] masih hidup setelah latch dilepas -- board dicatu USB, "
                     "bukan baterai. Cabut USB atau tekan RST.");
    }
    delay(100);
  }
}

static void pwr_poll(void) {
  int level = digitalRead(PWR_KEY);      /* LOW = sedang ditekan */

  /* Setiap perubahan level memulai ulang jendela debounce; hanya level yang
   * sudah diam selama PWR_DEBOUNCE_MS yang dipercaya. */
  if (level != pwr_level_lalu) {
    pwr_level_lalu = level;
    pwr_stabil_ms  = millis();
    return;
  }
  if ((uint32_t)(millis() - pwr_stabil_ms) < PWR_DEBOUNCE_MS) return;

  if (!pwr_siap) {
    if (level == HIGH) {
      pwr_siap = true;
      Serial.println("[pwr] tombol dilepas -- tekan sekali lagi untuk mematikan");
    }
    return;
  }

  if (level == LOW) pwr_matikan();       /* tidak pernah kembali */
}

/* ---------------- Setup / Loop ---------------- */
void setup() {
  /* ================= PALING AWAL, sebelum apa pun =================
   * Selama baris ini belum jalan, board hidup HANYA karena jari pengguna masih
   * menekan tombol PWR. Semua yang ditaruh di atasnya -- termasuk Serial.begin()
   * dan delay(200) di bawah -- adalah waktu tambahan yang harus dihabiskan
   * pengguna sambil menahan tombol. Contoh resmi Waveshare menaruhnya di posisi
   * yang sama persis, sebagai dua baris pertama setup(). */
  pinMode(BAT_EN, OUTPUT);
  digitalWrite(BAT_EN, HIGH);

  /* INPUT_PULLUP, bukan INPUT: tombol menarik pin ke GND saat ditekan, jadi
   * keadaan lepasnya butuh pull-up. Board ini punya pull-up eksternal, tetapi
   * meminta yang internal juga tidak merugikan dan membuat pin ini tidak pernah
   * mengambang seandainya rakitannya berbeda. */
  pinMode(PWR_KEY, INPUT_PULLUP);

  Serial.begin(115200);
  delay(200);
  Serial.println("\n[boot] ESP32-C6 LVGL dashboard");

  /* Backlight: PWM 5 kHz / 8 bit lewat LEDC. Tetap gelap dulu supaya tidak
   * ada flash putih, sekaligus menjaga panel tenang selama kalibrasi touch. */
  ledcAttach(LCD_BL, 5000, 8);
  ledcWrite(LCD_BL, 0);

  /* ================= URUTAN INIT PENTING =================
   * Touch HARUS diinisialisasi SEBELUM gfx->begin().
   *
   * CST816T mengkalibrasi baseline kapasitifnya saat start. Kalau gfx->begin()
   * berjalan lebih dulu (toggle LCD_RST + burst SPI ke panel yang menempel di
   * belakang sensor), chip mengunci baseline yang salah dan melaporkan ghost
   * touch permanen: register beku di finger=1 (~tengah layar) dengan IRQ
   * membanjir ~80/detik. LVGL lalu tidak pernah melihat transisi press->release
   * sehingga layar terasa mati total.
   *
   * Terukur lewat uji A/B terkontrol pada board ini:
   *   touch.begin() -> gfx->begin() : fingers=0, irq/s=0     (bersih, 48 detik)
   *   gfx->begin() -> touch.begin() : fingers=1, irq/s=80    (ghost, konsisten)
   *
   * Jangan pula memanggil touch.disableAutoSleep(): tidak menyembuhkan ghost,
   * dan soft-sleep (reg 0xE5) justru mematikan chip permanen karena pin RST
   * touch tidak terhubung ke MCU di board ini.
   * ======================================================= */
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);   /* 400k tidak stabil untuk CST816T di board ini */
  pinMode(TOUCH_IRQ, INPUT_PULLUP);
  if (!touch.begin(Wire, CST816_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
    Serial.println("[err] CST816 tidak terdeteksi (UI tetap jalan tanpa sentuh)");
  } else {
    Serial.printf("[ok] touch: %s\n", touch.getModelName());
  }
  delay(150);              /* beri waktu kalibrasi baseline selesai */
  attachInterrupt(digitalPinToInterrupt(TOUCH_IRQ), touch_isr, FALLING);

  if (!gfx->begin()) Serial.println("[err] gfx->begin() gagal");
  gfx->fillScreen(RGB565_BLACK);

  lv_init();

  size_t buf_px = SCREEN_W * BUF_LINES;
  buf1 = (lv_color_t *)heap_caps_malloc(buf_px * sizeof(lv_color_t), MALLOC_CAP_8BIT);
  buf2 = (lv_color_t *)heap_caps_malloc(buf_px * sizeof(lv_color_t), MALLOC_CAP_8BIT);
  if (!buf1) {
    Serial.println("[err] alokasi draw buffer gagal");
    while (1) delay(1000);
  }
  if (!buf2) Serial.println("[warn] buf2 gagal, jalan dengan single buffer");
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, buf_px);

  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_W;
  disp_drv.ver_res = SCREEN_H;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touch_read;
  lv_indev_drv_register(&indev_drv);

  /* RTC berbagi bus I2C dengan touch, dan Wire sudah di-begin di atas. */
  rtc_begin();
  tm_begin();
  battery_begin();
  ppg_begin();

  build_home();
  build_menu();
  build_hr();
  build_spo2();
  build_glu();
  build_bp();
  build_meas();
  lv_scr_load(scr_home);

  scan_dot = mk_scan_dot(scr_home);   /* dipindah antar-layar sendiri di scan_anim_cb */

  lv_timer_create(refresh_cb, 500, NULL);
  lv_timer_create(scan_anim_cb, 40, NULL);

  lv_timer_handler();
  ledcWrite(LCD_BL, 204);   /* ~80% */

  /* Paling akhir: UI sudah tampil sebelum radio mulai menyita CPU dan heap.
   *
   * AsaWatch dulu, baru Wi-Fi. Urutannya penting untuk heap: init NimBLE
   * meminta blok yang relatif besar sekaligus, dan lebih mudah didapat sebelum
   * stack Wi-Fi memfragmentasi heap dengan buffer-buffernya. Keduanya memang
   * berbagi satu radio 2.4 GHz di C6, tapi itu diurus coexistence di lapisan
   * bawah -- bukan urusan sketch.
   *
   * jam_mulai() sendiri punya urutan internal yang tidak boleh dibalik
   * (NVS -> boot_id naik -> muat ring -> sesi dipaksa IDLE -> BLE -> event
   * BOOT); lihat aw_jam.cpp. */
  jam_mulai();
  net_begin();

  Serial.printf("[ok] setup selesai, free heap = %lu\n", (unsigned long)ESP.getFreeHeap());
}

void loop() {
  touch_poll();          /* tangkap IRQ secepat mungkin, jangan tunggu LVGL */
  pwr_poll();            /* satu digitalRead; tombol mati harus selalu responsif */

  /* PPG ditunda selama masih ada IRQ touch yang belum diproses. Keduanya berbagi
   * bus I2C, dan satu transaksi FIFO MAX30105 (~1 ms di 100 kHz) cukup untuk
   * menunda pembacaan touch yang datanya hilang hampir seketika setelah IRQ.
   * Touch selalu menang; PPG cuma mundur satu iterasi (~2 ms) dan FIFO-nya
   * punya cadangan 320 ms, jadi tidak ada sampel yang hilang. */
  if (!touch_irq_flag) ppg_update();

  /* Logika protokol berjalan di task yang SAMA dengan lv_timer_handler()
   * (dokumen 13.2). Itu yang membuat callback tombol LVGL boleh memanggil
   * jam_tekan_tombol() langsung dan pembaca status UI boleh membaca variabel
   * sesi langsung -- tanpa mutex, tanpa antrean kedua. Yang menyeberang task
   * tinggal satu: antrean perintah BLE di aw_ble. */
  jam_putar();

  lv_timer_handler();
  delay(2);
}
