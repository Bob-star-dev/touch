/*
 * LVGL 8.3 UI -- 5 halaman sesuai mockup 1baru.png + 2..5.png
 * (1baru.png menggantikan 1.png; satu-satunya perbedaan: label "BATERAI" hilang)
 * Board : Waveshare ESP32-C6-Touch-LCD-1.69  (240x280, ST7789V2 + CST816T)
 *
 * Libraries : lvgl 8.3.x, GFX Library for Arduino, SensorLib
 * lv_conf.h : /home/harjo/Arduino/libraries/lv_conf.h  (LV_TICK_CUSTOM=1, 16bpp)
 *
 * Halaman :
 *   1 Home     - jam + tanggal, header cuaca & baterai        (tap layar -> menu)
 *   2 Menu     - 3 kartu: Heart rate / SpO2 / Glukosa
 *   3 Detail   - Heart rate : arc, EKG, bar 12 jam, chip statistik
 *   4 Detail   - SpO2       : ring, legend zona, grafik riwayat
 *   5 Detail   - Glukosa    : angka besar, bar rentang, tren harian
 *
 * Geometri & warna diambil langsung dari mockup 480x560 (tepat 2x layar),
 * jadi semua koordinat di bawah = koordinat mockup / 2.
 *
 * Aset gambar (pesawat+bulan, ikon) ada di ui_assets.h, dibuat otomatis dari
 * mockup. Jalankan genassets.py kalau mockup berubah.
 *
 * Modul data (semua akses I2C ada di konteks loop, lihat net.h soal thread):
 *   rtc / time_manager  - PCF85063 + sinkronisasi NTP
 *   net / weather       - Wi-Fi, NTP, OpenWeatherMap
 *   battery             - ADC1 + kurva Li-Po
 *   ppg                 - MAX30105/30102: BPM, SpO2, glukosa EKSPERIMENTAL
 *
 * PERINGATAN: nilai glukosa tidak punya dasar fisiologis tervalidasi dan tidak
 * boleh dipakai untuk keputusan medis apa pun. Lihat ppg.h.
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
static lv_obj_t *scr_home, *scr_menu, *scr_hr, *scr_spo2, *scr_glu;

/* home */
static lv_obj_t *lbl_hh, *lbl_mm, *lbl_day, *lbl_date, *lbl_wthr, *lbl_batt;
static lv_obj_t *lbl_cond;   /* caption kondisi cuaca di header */
/* menu */
static lv_obj_t *lbl_card_hr, *lbl_card_sp, *lbl_card_gl;
/* detail */
static lv_obj_t *arc_hr, *lbl_hr_big, *ring_sp, *lbl_sp_big, *lbl_glu_big;
static lv_obj_t *dot_live;
/* Handle tambahan supaya chip statistik, teks zona, dan penanda bar ikut
 * mengikuti data nyata. Ini hanya menyimpan pointer objek yang sudah ada --
 * posisi, ukuran, font, dan warnanya tidak diubah. Perlu, karena angka utama
 * yang nyata di atas angka yang dikarang di bawahnya justru menyesatkan. */
static lv_obj_t *chip_hr[3], *chip_sp[3], *chip_gl[3];
static lv_obj_t *lbl_hr_zone, *lbl_glu_status;
static lv_obj_t *mark_hr, *mark_glu;

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

/* Layar home: sentuhan apa pun (termasuk geseran) membuka menu -- di sini
 * geseran memang dibolehkan, dan berhentinya cukup sampai halaman 2. */
static void to_menu_cb(lv_event_t *e)  { (void)e; go(scr_menu, true,  "menu"); }

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
static const nav_target_t NAV_HR   = { &scr_hr,   true,  "heart rate" };
static const nav_target_t NAV_SPO2 = { &scr_spo2, true,  "spo2" };
static const nav_target_t NAV_GLU  = { &scr_glu,  true,  "glukosa" };

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

/* ================= Halaman 1 : Home ================= */
static void build_home(void) {
  scr_home = lv_obj_create(NULL);
  lv_obj_remove_style_all(scr_home);
  lv_obj_set_style_bg_color(scr_home, lv_color_hex(C_S1_BG), 0);
  lv_obj_set_style_bg_opa(scr_home, LV_OPA_COVER, 0);
  lv_obj_clear_flag(scr_home, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(scr_home, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(scr_home, to_menu_cb, LV_EVENT_PRESSED, NULL);

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

  /* Kelompok baterai. Angkanya dirata-kanan ke x=190 supaya jarak 7 px ke ikon
   * tetap sama walau lebarnya berubah ("9%" vs "100%"). Ukuran font mengikuti
   * mockup: kapital 8 px / lebar 25 px = montserrat_12, sepasang dengan "29C". */
  lbl_batt = mk_label(hdr, "82%", &lv_font_montserrat_12, C_BATT, 0, 5);
  lv_obj_set_width(lbl_batt, 190);
  lv_obj_set_style_text_align(lbl_batt, LV_TEXT_ALIGN_RIGHT, 0);

  /* 1baru.png menghapus label "BATERAI" -- tinggal persentase + ikon. */
  mk_img(hdr, &img_battery, 197, 8);

  /* --- ilustrasi pesawat + bulan --- */
  mk_img(scr_home, &img_plane, 44, 50);

  /* --- pemisah bergaya: garis - berlian - garis --- */
  mk_box(scr_home, 84, 184, 26, 1, C_S1_LINE, 0);
  mk_img(scr_home, &img_diamond, 115, 179);
  mk_box(scr_home, 130, 184, 26, 1, C_S1_LINE, 0);

  /* --- hari --- */
  lbl_day = mk_label(scr_home, "KAMIS", &lv_font_montserrat_12, C_S1_MUTED, 0, 191);
  lv_obj_set_style_text_letter_space(lbl_day, 3, 0);
  lv_obj_set_width(lbl_day, SCREEN_W);
  lv_obj_set_style_text_align(lbl_day, LV_TEXT_ALIGN_CENTER, 0);

  /* --- jam: "10" [kotak][kotak] "24" --- */
  lbl_hh = mk_label(scr_home, "10", &lv_font_montserrat_48, 0xFFFFFF, 0, 204);
  lv_obj_set_width(lbl_hh, 112);
  lv_obj_set_style_text_align(lbl_hh, LV_TEXT_ALIGN_RIGHT, 0);

  mk_box(scr_home, 116, 220, 8, 8, C_AMBER, 1);
  mk_box(scr_home, 116, 236, 8, 8, C_AMBER, 1);

  lbl_mm = mk_label(scr_home, "24", &lv_font_montserrat_48, 0xFFFFFF, 128, 204);

  /* --- tanggal --- */
  lbl_date = mk_label(scr_home, "23 JULI 2026", &lv_font_montserrat_14, C_DATE, 0, 257);
  lv_obj_set_style_text_letter_space(lbl_date, 2, 0);
  lv_obj_set_width(lbl_date, SCREEN_W);
  lv_obj_set_style_text_align(lbl_date, LV_TEXT_ALIGN_CENTER, 0);
}

/* ================= Halaman 2 : Menu kesehatan ================= */
/* Satu kartu menu: latar berwarna, ikon, judul, nilai, chevron. */
static lv_obj_t *mk_card(lv_obj_t *scr, int y, uint32_t bg, const char *title,
                         const char *val, uint32_t c_title, uint32_t c_sub,
                         const nav_target_t *nav, lv_obj_t **out_val) {
  lv_obj_t *card = mk_box(scr, 10, y, 220, 64, bg, 12);
  mk_tap_nav(card, 0, nav);         /* 220x64, sudah cukup besar */

  mk_label(card, title, &lv_font_montserrat_14, c_title, 45, 7);
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

  lv_obj_t *c1 = mk_card(scr_menu, 44,  C_HR_BG, "Heart rate", "-- bpm",
                         C_HR_TITLE, C_HR_SUB, &NAV_HR,   &lbl_card_hr);
  lv_obj_t *c2 = mk_card(scr_menu, 116, C_SP_BG, "SpO2", "-- %",
                         C_SP_TITLE, C_SP_SUB, &NAV_SPO2, &lbl_card_sp);
  lv_obj_t *c3 = mk_card(scr_menu, 188, C_GL_BG, "Glukosa", "-- mg/dL",
                         C_GL_TITLE, C_GL_SUB, &NAV_GLU,  &lbl_card_gl);

  /* ikon kartu: posisi vertikal ditengahkan manual terhadap tinggi kartu 64 */
  mk_img(c1, &img_heart_sm, 12, 22);
  mk_img(c2, &img_drop,     14, 21);
  mk_box(c3, 14, 24, 16, 16, C_AMBER2, LV_RADIUS_CIRCLE);

  lv_obj_t *f = mk_label(scr_menu, "3 sensor aktif " TXT_DOT " sinkron 2 mnt lalu",
                         &lv_font_montserrat_10, C_S2_FOOT, 0, 265);
  lv_obj_set_width(f, SCREEN_W);
  lv_obj_set_style_text_align(f, LV_TEXT_ALIGN_CENTER, 0);
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

  lbl_glu_big = mk_label(scr_glu, "--", &lv_font_montserrat_46, 0xFFFFFF, 12, 42);
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

/* Perbarui seluruh tampilan kesehatan dari data PPG. */
static void update_health_ui(void) {
  ppg_data_t p;
  ppg_get(&p);
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
  }

  /* LED "live" hanya berkedip saat pengukuran benar-benar berjalan. */
  static bool blink = false;
  blink = !blink;
  if (p.state == PPG_STABLE)
    lv_obj_set_style_opa(dot_live, blink ? LV_OPA_COVER : LV_OPA_40, 0);
  else
    lv_obj_set_style_opa(dot_live, LV_OPA_20, 0);
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
    if (t.tm_mday != last_mday) {
      last_mday = t.tm_mday;
      lv_label_set_text(lbl_day, tm_day_name(t.tm_wday));
      lv_label_set_text_fmt(lbl_date, "%d %s %d",
                            t.tm_mday, tm_month_name(t.tm_mon), t.tm_year + 1900);
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
    Serial.printf("[hb] %02d:%02d:%02d src=%s wifi=%d  cuaca=%s %dC  "
                  "touch irq=%lu err=%lu evt=%lu  heap=%lu\n",
                  have_time ? t.tm_hour : 0, have_time ? t.tm_min : 0,
                  have_time ? t.tm_sec : 0,
                  SRC[tm_source()], net_connected() ? 1 : 0,
                  w.valid ? w.cond : "--", w.temp_c,
                  (unsigned long)touch_irq_count, (unsigned long)touch_readerr,
                  (unsigned long)touch_events, (unsigned long)ESP.getFreeHeap());
    ppg_data_t pp; ppg_get(&pp);
    long dir, dred, dthr; uint32_t dn, dp;
    ppg_diag(&dir, &dred, &dn, &dthr, &dp);
    Serial.printf("[ppg]  %s%s  bpm=%s%.0f  spo2=%s%.1f  glukosa*=%s%.0f\n",
                  ppg_state_text(),
                  pp.held ? " [tahan]" : "",
                  pp.bpm_valid ? "" : "(-)", pp.bpm,
                  pp.spo2_valid ? "" : "(-)", pp.spo2,
                  pp.glu_valid ? "" : "(-)", pp.glucose);
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

/* ---------------- Setup / Loop ---------------- */
void setup() {
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
  lv_scr_load(scr_home);

  lv_timer_create(refresh_cb, 500, NULL);

  lv_timer_handler();
  ledcWrite(LCD_BL, 204);   /* ~80% */

  /* Paling akhir: UI sudah tampil sebelum Wi-Fi mulai menyita CPU. */
  net_begin();

  Serial.printf("[ok] setup selesai, free heap = %lu\n", (unsigned long)ESP.getFreeHeap());
}

void loop() {
  touch_poll();          /* tangkap IRQ secepat mungkin, jangan tunggu LVGL */

  /* PPG ditunda selama masih ada IRQ touch yang belum diproses. Keduanya berbagi
   * bus I2C, dan satu transaksi FIFO MAX30105 (~1 ms di 100 kHz) cukup untuk
   * menunda pembacaan touch yang datanya hilang hampir seketika setelah IRQ.
   * Touch selalu menang; PPG cuma mundur satu iterasi (~2 ms) dan FIFO-nya
   * punya cadangan 320 ms, jadi tidak ada sampel yang hilang. */
  if (!touch_irq_flag) ppg_update();

  lv_timer_handler();
  delay(2);
}
