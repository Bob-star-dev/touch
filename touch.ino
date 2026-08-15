/*
 * AsaWatch -- wajah jam SATU HALAMAN, LVGL 8.3
 * Board : Waveshare ESP32-C6-Touch-LCD-1.69  (240x280, ST7789V2 + CST816T)
 *
 * Acuan piksel: wajah_jam_modular_240x280.png, dibuat pada ukuran layar
 * sebenarnya -- jadi setiap koordinat di berkas ini diukur langsung dari
 * gambar itu, bukan dari mockup 2x yang dibagi dua seperti versi sebelumnya.
 *
 * KENAPA SATU HALAMAN
 * Versi lama punya 7 layar dan sebuah menu. Itu masuk akal selama ada layar
 * sentuh. Pada board ini sentuhannya tidak berfungsi (CST816T tidak menjawab
 * di I2C sama sekali), dan menavigasi 7 layar lewat satu tombol BOOT berarti
 * pengguna harus menekan berkali-kali hanya untuk melihat satu angka. Jadi
 * seluruh informasi dipadatkan ke satu wajah yang tidak perlu dinavigasi:
 * empat kartu metrik terlihat bersamaan, dan tidak ada lagi yang tersembunyi.
 *
 * Sisi aplikasilah yang kini memegang kendali (BLE), sesuai keputusan pemilik
 * perangkat. Jam menjadi sensor + buffer + penampil.
 *
 * SATU TOMBOL YANG TETAP HARUS ADA
 * Dokumen protokol bagian 3 menyatakan: "Jam adalah satu-satunya sumber t0.
 * Aplikasi tidak punya tombol yang setara, dan tidak boleh menghitung t0 dari
 * waktu pesan tiba." Karena itu menghapus SEMUA kendali dari jam akan memutus
 * jaminan inti protokolnya -- sesi tidak akan pernah bisa dimulai. Perannya
 * dipindah ke tekan-lama tombol BOOT, yang tidak memakan tempat di layar.
 * Lihat boot_poll().
 *
 * YANG HILANG DIBANDING VERSI 7 LAYAR, dan itu memang konsekuensi desain ini:
 * cuaca, ikon Bluetooth, pil status sesi, grafik riwayat, chip statistik, dan
 * layar "sedang mengukur". Yang paling penting di antaranya -- status sesi dan
 * kemajuan pengukuran -- tidak dibuang begitu saja: keduanya pindah ke baris
 * tanggal, yang berubah peran menjadi baris status saat ada sesuatu yang
 * sedang terjadi. Lihat status_baris().
 *
 * Libraries : lvgl 8.3.x, GFX Library for Arduino, SensorLib
 * lv_conf.h : /home/harjo/Arduino/libraries/lv_conf.h  (LV_TICK_CUSTOM=1, 16bpp)
 *
 * Modul data (semua akses I2C ada di konteks loop, lihat net.h soal thread):
 *   rtc / time_manager  - PCF85063 + sinkronisasi NTP
 *   net                 - Wi-Fi + NTP
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
#include "net.h"
#include "battery.h"
#include "ppg.h"
#include "aw_jam.h"

/* Subset digit-only dari montserrat_48 (cuma glyph '-' '.' '/' dan 0-9),
 * dipakai untuk jam besar. Font bawaan LVGL di ukuran itu ikut membawa seluruh
 * ASCII + ikon FontAwesome (~90 KB) yang tak pernah dirender di sini.
 * font_digits_46 tetap ada di direktori sketsa tapi tidak lagi dipakai. */
extern "C" {
LV_FONT_DECLARE(font_digits_48)
}

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
 * PWR_KEY aktif LOW (ditekan = 0). Ia juga dibawa keluar ke header board, jadi
 * jangan pakai pad GPIO18 untuk hal lain -- ia berbagi jalur dengan tombol. */
#define BAT_EN    15
#define PWR_KEY   18

/* ---- Tombol BOOT: satu-satunya kendali yang tersisa ----
 * Nomor pin dari contoh Arduino resmi Waveshare 02_button_example.ino untuk
 * board ini, yang menamainya "BOOT Button" dan memakai OneButton(pin, true) --
 * artinya aktif LOW, sama seperti PWR_KEY.
 *
 * Ini juga strapping pin: DITAHAN SAAT RESET, chip masuk mode download alih-alih
 * menjalankan sketch. Saat runtime itu tidak berlaku, jadi aman dibaca di
 * loop(). */
#define BOOT_KEY   9

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
 * Dipertahankan utuh walau board ini tidak punya sentuh yang berfungsi, supaya
 * satu firmware yang sama tetap jalan di board yang sentuhannya baik. Wajah
 * satu halaman ini memang tidak punya sasaran sentuh apa pun -- tidak ada yang
 * bisa diketuk -- jadi jalur ini kini murni tidak berefek, bukan alternatif
 * kendali.
 *
 * CST816T di board yang sehat mengosongkan byte finger-count hampir seketika
 * setelah IRQ, jadi polling bebas hampir selalu melewatkannya. Data dibaca
 * tepat saat IRQ. */
#define TOUCH_ADDR      0x15
#define TOUCH_HOLD_MS   180
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
        touch_release_pending = true;
      } else if ((fingers > 0 || evt == 2) && x < SCREEN_W && y < SCREEN_H) {
        last_touch_x = x;
        last_touch_y = y;
        if (!touch_down) touch_press_ms = millis();
        touch_down = true;
        touch_release_pending = false;
        touch_last_ms = millis();
        touch_events++;
      }
    }
  }

  if (touch_down) {
    uint32_t now = millis();
    if (touch_release_pending && (now - touch_press_ms) >= TOUCH_MIN_MS) {
      touch_down = false;
      touch_release_pending = false;
    } else if ((now - touch_last_ms) > TOUCH_HOLD_MS) {
      touch_down = false;
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

/* Glyph non-ASCII yang tersedia di lv_font_montserrat_* bawaan LVGL. */
#define TXT_DEG  "\xC2\xB0"      /* U+00B0 derajat */
#define TXT_DOT  "\xE2\x80\xA2"  /* U+2022 bullet  */

/* ================= Palet =================
 * Setiap nilai di bawah adalah warna yang benar-benar diambil dari piksel
 * wajah_jam_modular_240x280.png, bukan tafsiran. */
#define C_BG_ATAS   0x06070B   /* latar di tepi atas   */
#define C_BG_BAWAH  0x0D0C18   /* latar di tepi bawah  */
#define C_KARTU     0x171A21
#define C_KARTU_BRD 0x23262D
#define C_PUTIH     0xFFFFFF   /* angka besar & jam    */
#define C_TANGGAL   0xB9C1D0   /* baris tanggal        */
#define C_REDUP     0x8E96A6   /* judul kartu, satuan, persen baterai */
#define C_ISI       0xA2E32B   /* petir "sedang mengisi" */
#define C_TRACK     0x2E3444   /* jalur gelap di belakang cincin */
#define C_CINCIN_HR 0xFF3B5C
#define C_CINCIN_SP 0xA2E32B
#define C_CINCIN_GL 0x22D3EE

/* ================= Geometri =================
 * Diukur dari gambar desain, bukan dikira-kira:
 *   kartu kiri  x=10..115   kartu kanan x=124..229   (lebar 106, jarak 8)
 *   baris atas  y=101..180  baris bawah y=191..270   (tinggi 80, jarak 10)
 *   cincin      bbox x=11..79, y=15..83 -> pusat (45,49), jari-jari luar 34
 *   pita cincin 7 px, jari-jari luar 34 / 25 / 16 */
#define KARTU_W   106
#define KARTU_H   80
#define KARTU_X1  10
#define KARTU_X2  124
#define KARTU_Y1  101
#define KARTU_Y2  191

#define CINCIN_CX 45
#define CINCIN_CY 49
#define CINCIN_W  7

/* ================= SUMBER DATA =================
 * Tidak ada angka yang dikarang di layar ini. Kalau sebuah nilai belum
 * tersedia -- jari tidak menempel, sensor tidak terpasang, ADC belum stabil --
 * yang tampil adalah "--", bukan angka contoh. Pada layar kesehatan, angka
 * contoh yang tampak nyata lebih berbahaya daripada tanda hubung.
 *
 *   jam / hari / tanggal   -> RTC PCF85063 + sinkronisasi NTP
 *   persen baterai         -> ADC1 + kurva Li-Po (battery.cpp)
 *   detak, SpO2, glukosa,  -> MAX30105/30102 lewat jam_snapshot()
 *   tekanan darah             glukosa & tekanan EKSPERIMENTAL, lihat ppg.h
 */

/* ================= Handle widget ================= */
static lv_obj_t *scr_wajah;

static lv_obj_t *lbl_jam_hh, *lbl_jam_mm;   /* "14" dan "35"                  */
static lv_obj_t *lbl_status;                /* baris tanggal / status         */
static lv_obj_t *lbl_batt, *lbl_ikon_batt;

static lv_obj_t *cincin_hr, *cincin_sp, *cincin_gl;

static lv_obj_t *lbl_hr, *lbl_sp, *lbl_gl, *lbl_bp;          /* angka besar   */
static lv_obj_t *sat_hr, *sat_sp, *sat_gl;                   /* satuan        */

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

/* Sejajarkan satuan pada GARIS DASAR angkanya, bukan pada tepi kotaknya.
 *
 * Dihitung dari metrik font, bukan dari angka ajaib hasil coba-coba: jarak dari
 * tepi atas kotak baris ke garis dasar adalah (line_height - base_line), jadi
 * selisih kedua font itulah pergeseran yang membuat kedua garis dasar berimpit.
 * Kalau ukuran font angka diganti nanti, satuannya ikut benar dengan
 * sendirinya. Dipanggil ulang tiap kali teks angkanya berubah, karena lebar
 * label ikut berubah ("98" jadi "100") dan satuan harus menempel di kanannya. */
static void satuan_sejajar(lv_obj_t *satuan, lv_obj_t *nilai, const lv_font_t *fn) {
  if (!satuan) return;
  int dy = (int)(fn->line_height - fn->base_line)
         - (int)(lv_font_montserrat_12.line_height - lv_font_montserrat_12.base_line);
  lv_obj_align_to(satuan, nilai, LV_ALIGN_OUT_RIGHT_TOP, 4, dy);
}

/* Satu kartu metrik: ikon, judul, angka besar, satuan.
 *
 * font_nilai diparameterkan karena kartu tekanan memuat enam karakter
 * ("118/76") sedangkan yang lain paling banyak tiga -- memaksakan satu ukuran
 * untuk keempatnya akan membuat tekanan meluber keluar kartu. Gambar desainnya
 * sendiri sudah merender kartu itu lebih kecil (tinggi ink 21 px lawan 23 px),
 * jadi ini mengikuti desain, bukan menyimpang darinya. */
static lv_obj_t *mk_kartu(lv_obj_t *scr, int x, int y, const lv_img_dsc_t *ikon,
                          const char *judul, const char *satuan,
                          const lv_font_t *font_nilai,
                          lv_obj_t **out_nilai, lv_obj_t **out_satuan) {
  lv_obj_t *k = mk_box(scr, x, y, KARTU_W, KARTU_H, C_KARTU, 14);
  lv_obj_set_style_border_width(k, 1, 0);
  lv_obj_set_style_border_color(k, lv_color_hex(C_KARTU_BRD), 0);
  lv_obj_set_style_border_opa(k, LV_OPA_COVER, 0);

  mk_img(k, ikon, 10, 9);
  mk_label(k, judul, &lv_font_montserrat_12, C_REDUP, 33, 8);

  *out_nilai = mk_label(k, "--", font_nilai, C_PUTIH, 10, 26);
  if (satuan) {
    *out_satuan = mk_label(k, satuan, &lv_font_montserrat_12, C_REDUP, 0, 0);
    satuan_sejajar(*out_satuan, *out_nilai, font_nilai);
  } else if (out_satuan) {
    *out_satuan = NULL;
  }
  return k;
}

/* Satu cincin kemajuan.
 *
 * Knob-nya dilepas dan flag klik dimatikan: lv_arc bawaan LVGL adalah kendali
 * yang bisa diseret, dan di sini ia murni penampil. Tanpa itu, sentuhan di
 * board yang sehat bisa menggeser nilainya dan membuat cincin menampilkan
 * angka yang tidak berasal dari sensor mana pun. */
static lv_obj_t *mk_cincin(lv_obj_t *scr, int r_luar, uint32_t warna) {
  int d = r_luar * 2 + 1;
  lv_obj_t *a = lv_arc_create(scr);
  lv_obj_remove_style(a, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(a, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(a, d, d);
  lv_obj_set_pos(a, CINCIN_CX - r_luar, CINCIN_CY - r_luar);
  lv_arc_set_rotation(a, 270);          /* 0% mulai di jam 12 */
  lv_arc_set_bg_angles(a, 0, 360);
  lv_arc_set_range(a, 0, 1000);
  lv_arc_set_value(a, 0);
  lv_obj_set_style_arc_width(a, CINCIN_W, LV_PART_MAIN);
  lv_obj_set_style_arc_width(a, CINCIN_W, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(a, lv_color_hex(C_TRACK), LV_PART_MAIN);
  lv_obj_set_style_arc_color(a, lv_color_hex(warna), LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(a, true, LV_PART_MAIN);
  lv_obj_set_style_arc_rounded(a, true, LV_PART_INDICATOR);
  return a;
}

/* ================= Cincin: apa yang sebenarnya ditampilkan =================
 * Ketiga cincin adalah pengukur tiga metrik terhadap rentang tampilan di
 * bawah. Rentang ini SEMATA-MATA untuk menggambar, bukan ambang medis, dan
 * tidak boleh dibaca sebagai "normal" atau "berbahaya" -- ia cuma menentukan
 * berapa penuh sebuah cincin tergambar.
 *
 * Kalau metriknya belum sah, cincinnya tinggal jalur gelap. Itu memang yang
 * diinginkan: cincin kosong berarti "belum ada data", bukan "nilainya nol".
 * Cincin sengaja tidak pernah menahan nilai terakhir seperti kartu, karena
 * bentuk visual yang penuh tanpa angka pendampingnya mustahil dibedakan dari
 * pengukuran yang sedang berlangsung. */
#define HR_MIN   40.0f
#define HR_MAKS 180.0f
#define SP_MIN   85.0f
#define SP_MAKS 100.0f
#define GL_MIN   70.0f
#define GL_MAKS 200.0f

static void cincin_set(lv_obj_t *a, bool sah, float v, float lo, float hi) {
  int nilai = 0;
  if (sah) {
    float f = (v - lo) / (hi - lo);
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    nilai = (int)(f * 1000.0f + 0.5f);
  }
  if (lv_arc_get_value(a) != nilai) lv_arc_set_value(a, nilai);
}

/* ================= Wajah jam ================= */
static void build_wajah(void) {
  scr_wajah = lv_obj_create(NULL);
  lv_obj_remove_style_all(scr_wajah);
  lv_obj_clear_flag(scr_wajah, LV_OBJ_FLAG_SCROLLABLE);
  /* Latarnya gradien tipis seperti di desain (tepi atas lebih gelap daripada
   * tepi bawah). Bedanya cuma beberapa digit, tapi tanpa itu layar terlihat
   * datar dan kartu-kartunya kehilangan kedalaman. */
  lv_obj_set_style_bg_color(scr_wajah, lv_color_hex(C_BG_ATAS), 0);
  lv_obj_set_style_bg_grad_color(scr_wajah, lv_color_hex(C_BG_BAWAH), 0);
  lv_obj_set_style_bg_grad_dir(scr_wajah, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_bg_opa(scr_wajah, LV_OPA_COVER, 0);

  /* --- tiga cincin, dari luar ke dalam --- */
  cincin_hr = mk_cincin(scr_wajah, 34, C_CINCIN_HR);
  cincin_sp = mk_cincin(scr_wajah, 25, C_CINCIN_SP);
  cincin_gl = mk_cincin(scr_wajah, 16, C_CINCIN_GL);

  /* --- baris tanggal / status ---
   * letter_space 1: tanpa itu "SAB 15 AGU" terbaca merapat jadi satu blok di
   * montserrat_12, dan desainnya sendiri memang merender baris ini dengan huruf
   * yang berjarak. x=82, bukan 86, untuk memberi ruang tambahan ke kelompok
   * baterai di kanan -- cincin berakhir di x=79 jadi 82 masih aman. */
  lbl_status = mk_label(scr_wajah, "--", &lv_font_montserrat_12, C_TANGGAL, 82, 13);
  lv_obj_set_style_text_letter_space(lbl_status, 1, 0);

  /* --- kelompok baterai: "94%" lalu ikonnya, tepi kanan dikunci di x=232 ---
   * Posisinya dihitung ulang tiap kali teksnya berubah (lihat batt_tata()),
   * sebab "9%" dan "100%" berbeda belasan piksel sedangkan tepi kanannya harus
   * tetap sejajar dengan tepi kanan kartu di bawahnya. */
  lbl_batt      = mk_label(scr_wajah, "--%", &lv_font_montserrat_12, C_REDUP, 190, 13);
  lbl_ikon_batt = mk_label(scr_wajah, LV_SYMBOL_BATTERY_EMPTY, &lv_font_montserrat_12,
                           C_REDUP, 221, 13);

  /* --- jam besar ---
   * font_digits_48 punya 9 px kosong di atas ink digit, jadi label di y=33
   * menaruh ink di y=42 -- persis seperti desain. "14" dirata-kanan ke x=153
   * dan "35" mulai di x=172; celah di antaranya diisi titik dua. */
  lbl_jam_hh = mk_label(scr_wajah, "--", &font_digits_48, C_PUTIH, 0, 33);
  lv_obj_set_width(lbl_jam_hh, 153);
  lv_obj_set_style_text_align(lbl_jam_hh, LV_TEXT_ALIGN_RIGHT, 0);

  /* Titik dua digambar sebagai dua kotak, bukan glyph: font_digits_48 memang
   * tidak punya ':' (rentangnya cuma 45..57, yaitu '-' '.' '/' dan 0-9). */
  mk_box(scr_wajah, 158, 51, 8, 8, C_PUTIH, 2);
  mk_box(scr_wajah, 158, 68, 8, 8, C_PUTIH, 2);

  lbl_jam_mm = mk_label(scr_wajah, "--", &font_digits_48, C_PUTIH, 172, 33);

  /* --- empat kartu metrik --- */
  mk_kartu(scr_wajah, KARTU_X1, KARTU_Y1, &ic_detak,   "DETAK",   "BPM",
           &lv_font_montserrat_30, &lbl_hr, &sat_hr);
  mk_kartu(scr_wajah, KARTU_X2, KARTU_Y1, &ic_spo2,    "SpO2",    "%",
           &lv_font_montserrat_30, &lbl_sp, &sat_sp);
  mk_kartu(scr_wajah, KARTU_X1, KARTU_Y2, &ic_glukosa, "GLUKOSA", "mg/dL",
           &lv_font_montserrat_30, &lbl_gl, &sat_gl);
  mk_kartu(scr_wajah, KARTU_X2, KARTU_Y2, &ic_tekanan, "TEKANAN", NULL,
           &lv_font_montserrat_26, &lbl_bp, NULL);
}

/* ================= Pembaruan isi ================= */

/* true kalau teksnya benar-benar berubah. lv_label_set_text() selalu
 * meng-invalidate area labelnya, jadi menulis ulang teks yang sama memaksa
 * gambar ulang dua kali per detik tanpa alasan. */
static bool set_jika_beda(lv_obj_t *lbl, const char *txt) {
  if (strcmp(lv_label_get_text(lbl), txt) == 0) return false;
  lv_label_set_text(lbl, txt);
  return true;
}

static void nilai_set(lv_obj_t *lbl, lv_obj_t *satuan, const lv_font_t *fn,
                      bool sah, const char *fmt, float a, float b) {
  char buf[16];
  if (!sah)                 snprintf(buf, sizeof(buf), "--");
  else if (b < 0.0f)        snprintf(buf, sizeof(buf), fmt, a);
  else                      snprintf(buf, sizeof(buf), fmt, a, b);
  if (set_jika_beda(lbl, buf)) satuan_sejajar(satuan, lbl, fn);
}

/* ---- Kelompok baterai: persen + ikon ----
 * Ikonnya BUKAN gambar tetap. Ia mengikuti persen yang sama dengan angkanya,
 * jadi dua penanda itu tidak akan pernah saling bertentangan -- ikon penuh di
 * sebelah tulisan "20%" adalah persis jenis kebohongan kecil yang membuat
 * seluruh tampilan tidak bisa dipercaya.
 *
 * Saat mengisi, ikonnya BERGANTI jadi petir alih-alih menambah lambang ketiga
 * di sebelahnya. Itu bukan sekadar selera: baris ini cuma punya ~50 px di
 * kanan tanggal, dan lambang tambahan akan menabrak "SAB 15 AGU" pada tanggal
 * yang namanya panjang. Petir menggantikan tempat, bukan meminta tempat baru.
 *
 * Ambangnya sengaja tidak rata: yang penting dibedakan adalah ujung bawah
 * (baterai hampir habis), bukan ujung atas. */
static const char *batt_ikon(int persen, bool mengisi) {
  if (mengisi)     return LV_SYMBOL_CHARGE;
  if (persen >= 90) return LV_SYMBOL_BATTERY_FULL;
  if (persen >= 65) return LV_SYMBOL_BATTERY_3;
  if (persen >= 40) return LV_SYMBOL_BATTERY_2;
  if (persen >= 15) return LV_SYMBOL_BATTERY_1;
  return LV_SYMBOL_BATTERY_EMPTY;
}

static void batt_tata(bool mengisi) {
  /* Lebar label baru sah setelah tata letaknya dihitung ulang; tanpa ini
   * lv_obj_get_width() masih mengembalikan lebar teks yang lama. */
  lv_obj_update_layout(scr_wajah);
  lv_obj_set_pos(lbl_ikon_batt, 232 - lv_obj_get_width(lbl_ikon_batt), 13);
  lv_obj_align_to(lbl_batt, lbl_ikon_batt, LV_ALIGN_OUT_LEFT_MID, -4, 0);
  lv_obj_set_style_text_color(lbl_ikon_batt,
                              lv_color_hex(mengisi ? C_ISI : C_REDUP), 0);
}

/* ---- Baris tanggal yang merangkap baris status ----
 * Ini satu-satunya area teks bebas di wajah ini, jadi ia memikul tiga hal
 * sekaligus. Urutan prioritasnya penting dan disengaja:
 *
 *   1. Pesan sesaat (2 detik) -- jawaban atas tombol yang baru saja ditekan.
 *      Paling atas karena ini satu-satunya umpan balik yang dimiliki tombol
 *      BOOT; tanpa itu menekan tombol terasa seperti tidak terjadi apa-apa.
 *   2. Kemajuan pengukuran -- pengguna harus diam selama puluhan detik, dan
 *      dokumen 14 mewajibkan ia tahu bahwa sesuatu sedang berjalan serta
 *      seberapa jauh. Yang ditampilkan detak tercacah, bukan detik berjalan,
 *      sebab itulah gerbang sebenarnya berakhirnya pengukuran.
 *   3. Tanggal -- keadaan tenang.
 */
static uint32_t pesan_sampai_ms = 0;
static char     pesan_teks[24]  = "";

static void status_pesan(const char *txt) {
  strncpy(pesan_teks, txt, sizeof(pesan_teks) - 1);
  pesan_teks[sizeof(pesan_teks) - 1] = '\0';
  pesan_sampai_ms = millis() + 2000;
}

static void status_baris(void) {
  char buf[40];

  if (pesan_sampai_ms && (int32_t)(millis() - pesan_sampai_ms) < 0) {
    if (set_jika_beda(lbl_status, pesan_teks))
      lv_obj_set_style_text_color(lbl_status, lv_color_hex(C_PUTIH), 0);
    return;
  }
  pesan_sampai_ms = 0;

  if (jam_sedang_mengukur()) {
    snprintf(buf, sizeof(buf), "MENGUKUR %u/%u",
             (unsigned)jam_ukur_detak(), (unsigned)jam_ukur_detak_perlu());
    if (set_jika_beda(lbl_status, buf))
      lv_obj_set_style_text_color(lbl_status, lv_color_hex(C_ISI), 0);
    return;
  }

  struct tm t;
  if (!tm_now(&t)) {
    if (set_jika_beda(lbl_status, "--"))
      lv_obj_set_style_text_color(lbl_status, lv_color_hex(C_TANGGAL), 0);
    return;
  }
  /* "SAB 15 AGU": tiga huruf pertama nama hari dan bulan. Dalam bahasa
   * Indonesia potongan itu selalu jatuh benar (SABTU->SAB, AGUSTUS->AGU,
   * SEPTEMBER->SEP), jadi tidak perlu tabel singkatan terpisah. */
  const char *hari  = tm_day_name(t.tm_wday);
  const char *bulan = tm_month_name(t.tm_mon);
  snprintf(buf, sizeof(buf), "%.3s %d %.3s", hari, t.tm_mday, bulan);
  if (set_jika_beda(lbl_status, buf))
    lv_obj_set_style_text_color(lbl_status, lv_color_hex(C_TANGGAL), 0);
}

static void refresh_cb(lv_timer_t *tm) {
  (void)tm;
  /* Konteks loop: di sinilah RTC dibaca dan hasil NTP diterapkan. */
  tm_tick();

  /* ---- jam besar ---- */
  struct tm t;
  bool have_time = tm_now(&t);
  if (have_time) {
    static int menit_lalu = -1;
    if (t.tm_min != menit_lalu) {
      menit_lalu = t.tm_min;
      lv_label_set_text_fmt(lbl_jam_hh, "%02d", t.tm_hour);
      lv_label_set_text_fmt(lbl_jam_mm, "%02d", t.tm_min);
    }
  }

  status_baris();

  /* ---- baterai ----
   * Persen saja tidak jujur saat kabel tertancap. Selama mengisi, tegangan sel
   * dinaikkan oleh arus pengisian dan fase CV menahannya di 4,2 V, jadi kurva
   * Li-Po membaca hampir 100% jauh sebelum selnya benar-benar penuh.
   *
   * battery_charging() menilai tren tegangan berbeban 3 menit terakhir, jadi ia
   * punya dua batas: baru menjawab setelah jam menyala 3 menit, dan tidak
   * mengenali fase CV di ujung pengisian karena tegangannya sudah rata.
   * Keduanya hanya membuat petirnya TIDAK muncul -- tidak pernah klaim palsu
   * bahwa jam sedang mengisi. */
  battery_update();
  if (battery_valid()) {
    static int batt_lalu = -1;
    static int isi_lalu  = -1;
    int bp  = battery_percent();
    int chg = battery_charging() ? 1 : 0;
    if (bp != batt_lalu || chg != isi_lalu) {
      batt_lalu = bp;
      isi_lalu  = chg;
      lv_label_set_text_fmt(lbl_batt, "%d%%", bp);
      lv_label_set_text(lbl_ikon_batt, batt_ikon(bp, chg != 0));
      batt_tata(chg != 0);
    }
  }

  /* ---- empat metrik + tiga cincin ----
   * jam_snapshot() dipakai, BUKAN ppg_get(): sensor hanya menyala selama
   * pengukuran, jadi membaca langsung dari sensor berarti keempat kartu
   * menampilkan "--" hampir sepanjang waktu -- benar, tetapi tidak berguna.
   * jam_snapshot() menambalnya dengan hasil pengukuran terakhir. */
  ppg_data_t p;
  jam_snapshot(&p);

  nilai_set(lbl_hr, sat_hr, &lv_font_montserrat_30, p.bpm_valid,  "%.0f", p.bpm, -1.0f);
  nilai_set(lbl_sp, sat_sp, &lv_font_montserrat_30, p.spo2_valid, "%.0f", p.spo2, -1.0f);
  nilai_set(lbl_gl, sat_gl, &lv_font_montserrat_30, p.glu_valid,  "%.0f", p.glucose, -1.0f);
  nilai_set(lbl_bp, NULL,   &lv_font_montserrat_26, p.bp_valid,   "%.0f/%.0f", p.sbp, p.dbp);

  cincin_set(cincin_hr, p.bpm_valid,  p.bpm,     HR_MIN, HR_MAKS);
  cincin_set(cincin_sp, p.spo2_valid, p.spo2,    SP_MIN, SP_MAKS);
  cincin_set(cincin_gl, p.glu_valid,  p.glucose, GL_MIN, GL_MAKS);

  /* ================= Heartbeat serial, tiap 5 detik =================
   * Ini satu-satunya jendela ke dalam jam sekarang: wajah barunya tidak punya
   * ikon Bluetooth, pil status sesi, maupun penghitung entri tertunda, jadi
   * yang dulu bisa dibaca dari layar kini HANYA ada di sini. Menghapusnya
   * bersama layar-layar itu akan membuat jam yang diam mustahil dibedakan dari
   * jam yang macet. */
  static uint8_t n = 0;
  if (++n < 10) return;
  n = 0;

  /* Diagnostik boot dicetak di heartbeat PERTAMA, bukan di setup(): USB CDC
   * board ini re-enumerate setelah reset sehingga apa pun yang dicetak setup()
   * hampir selalu hilang sebelum host siap menerima. */
  static bool diag_done = false;
  if (!diag_done) {
    diag_done = true;
    Serial.printf("[diag] PCF85063 terdeteksi: %s\n", rtc_ok() ? "YA" : "TIDAK");
    Serial.printf("[diag] waktu saat boot: %s\n", tm_boot_info());
    rtc_scan_bus();
  }

  static const char *SRC[] = { "none", "rtc", "ntp" };
  Serial.printf("[hb] %02d:%02d:%02d src=%s wifi=%d ble=%d sesi=%d tunda=%d  "
                "touch irq=%lu err=%lu evt=%lu  heap=%lu\n",
                have_time ? t.tm_hour : 0, have_time ? t.tm_min : 0,
                have_time ? t.tm_sec : 0,
                SRC[tm_source()], net_connected() ? 1 : 0,
                jam_siap_notifikasi() ? 2 : (jam_terhubung() ? 1 : 0),
                (int)jam_status(), (int)jam_tertunda(),
                (unsigned long)touch_irq_count, (unsigned long)touch_readerr,
                (unsigned long)touch_events, (unsigned long)ESP.getFreeHeap());

  long dir, dred, dthr; uint32_t dn, dp;
  ppg_diag(&dir, &dred, &dn, &dthr, &dp);
  Serial.printf("[ppg]  %s%s  bpm=%s%.0f  spo2=%s%.1f  glukosa*=%s%.0f  "
                "td*=%s%.0f/%.0f\n",
                ppg_state_text(), p.held ? " [tahan]" : "",
                p.bpm_valid  ? "" : "(-)", p.bpm,
                p.spo2_valid ? "" : "(-)", p.spo2,
                p.glu_valid  ? "" : "(-)", p.glucose,
                p.bp_valid   ? "" : "(-)", p.sbp, p.dbp);
  /* Baris mentah: sampel yang diam di satu angka berarti FIFO tidak
   * menghasilkan apa pun -- biasanya sensor tidak dicatu daya. */
  Serial.printf("[ppg] mentah ir=%ld red=%ld (ambang %ld)  sampel=%lu  poll=%lu\n",
                dir, dred, dthr, (unsigned long)dn, (unsigned long)dp);

  Serial.printf("[batt] counts=%d/4095  raw=%d mV (sebaran %d mV)  "
                "baterai=%d mV  floor=%d mV%s  %d%%\n",
                battery_raw_counts(), battery_raw_millivolts(),
                battery_spread_mv(), battery_millivolts(),
                battery_floor_mv(), battery_charging() ? " [mengisi]" : "",
                battery_percent());
  if (battery_history_count() > 1) {
    char hb[192];
    battery_history(hb, sizeof(hb));
    Serial.printf("[batt] riwayat/menit (mV di pin, tertua dulu): %s\n", hb);
  }
}

/* ================= Tombol PWR: tekan sekali hidup, tekan lagi mati ========= */
#define PWR_DEBOUNCE_MS  50

static bool     pwr_siap = false;      /* tombol sudah pernah dilepas sejak boot */
static int      pwr_level_lalu = HIGH;
static uint32_t pwr_stabil_ms = 0;

static void pwr_matikan(void) {
  Serial.println("[pwr] tombol PWR ditekan -- mematikan");
  ledcWrite(LCD_BL, 0);
  jam_siap_mati();
  digitalWrite(BAT_EN, LOW);
  uint32_t t0 = millis();
  bool dicatat = false;
  for (;;) {
    if (!dicatat && (uint32_t)(millis() - t0) >= 2000) {
      dicatat = true;
      Serial.println("[pwr] masih hidup setelah latch dilepas -- board dicatu USB, "
                     "bukan baterai. Cabut USB atau tekan RST.");
    }
    delay(100);
  }
}

static void pwr_poll(void) {
  int level = digitalRead(PWR_KEY);      /* LOW = sedang ditekan */
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

/* ================= Tombol BOOT: satu-satunya kendali =================
 * Perannya persis peran tombol utama di versi 7 layar, dan percabangannya sama:
 *
 *   IDLE            -> cek manual. Hasilnya berhenti di layar jam: tidak ada
 *                      entri sampel, tidak ada event, tidak ada satu byte pun
 *                      yang dikirim ke aplikasi.
 *   ARMED / RUNNING -> "Selesai Makan", yaitu SUMBER TUNGGAL t0 (dokumen 3).
 *
 * Hanya tekan-lama yang dihitung, dan itu bukan gaya-gayaan: tombol ini juga
 * strapping pin yang dipakai untuk masuk mode download, jadi ia gampang
 * tersenggol saat menancapkan kabel. Salah satu perannya memulai sesi dua jam
 * yang tidak bisa dibatalkan dari jam, jadi ambang 700 ms jauh lebih murah
 * daripada sesi yang lahir karena senggolan.
 *
 * Aksinya dijalankan saat ambang terlewati, bukan saat jari dilepas: kalau
 * menunggu pelepasan, pengguna menahan sambil menebak apakah sudah cukup lama,
 * dan tebakan itu satu-satunya umpan balik yang ada. */
#define BOOT_DEBOUNCE_MS   30
#define BOOT_LAMA_MS      700

static bool      boot_siap        = false;
static int       boot_level_lalu  = HIGH;
static int       boot_stabil_lvl  = HIGH;
static uint32_t  boot_stabil_ms   = 0;
static uint32_t  boot_tekan_ms    = 0;
static bool      boot_sudah_jalan = false;

/* Umpan balik penolakan wajib spesifik. "Tidak terjadi apa-apa" adalah cara
 * tercepat membuat pengguna menyimpulkan jamnya rusak, padahal jam menolak
 * karena alasan yang benar. */
static void boot_aktifkan(void) {
  bool idle = (jam_status() == AW_SESI_IDLE);
  if (idle) jam_cek_manual();
  else      jam_tekan_tombol();

  switch (jam_umpan_balik_ditolak()) {
    case JAM_TOLAK_SEDANG_UKUR: status_pesan("SEDANG MENGUKUR");   return;
    case JAM_TOLAK_BATERAI:     status_pesan("BATERAI LEMAH");     return;
    case JAM_TOLAK_SENSOR:      status_pesan("SENSOR TIDAK ADA");  return;
    case JAM_TOLAK_SESI_AKTIF:  status_pesan("SESI SEDANG JALAN"); return;
    case JAM_TOLAK_SESI_BERJALAN: status_pesan("SESI SUDAH MULAI"); return;
    case JAM_TOLAK_BELUM_ARM:   status_pesan("BELUM DISIAPKAN");   return;
    default: break;
  }
  status_pesan(idle ? "CEK MANUAL" : "SELESAI MAKAN");
}

static void boot_poll(void) {
  int level = digitalRead(BOOT_KEY);            /* LOW = sedang ditekan */
  if (level != boot_level_lalu) {
    boot_level_lalu = level;
    boot_stabil_ms  = millis();
  } else if ((uint32_t)(millis() - boot_stabil_ms) >= BOOT_DEBOUNCE_MS &&
             level != boot_stabil_lvl) {
    boot_stabil_lvl = level;                    /* tepi yang sudah bersih */
    if (level == LOW) {
      boot_tekan_ms    = millis();
      boot_sudah_jalan = false;
    } else if (!boot_siap) {
      boot_siap = true;
      Serial.println("[boot] tombol dilepas -- siap dipakai");
    }
  }

  if (boot_siap && boot_stabil_lvl == LOW && !boot_sudah_jalan &&
      (uint32_t)(millis() - boot_tekan_ms) >= BOOT_LAMA_MS) {
    boot_sudah_jalan = true;
    boot_aktifkan();
  }
}

/* ---------------- Setup / Loop ---------------- */
void setup() {
  /* ================= PALING AWAL, sebelum apa pun =================
   * Menahan latch baterai adalah syarat board tetap hidup setelah jari lepas
   * dari tombol PWR. Setiap milidetik sebelum baris ini adalah milidetik saat
   * board masih bergantung pada jari yang menekan. */
  pinMode(BAT_EN, OUTPUT);
  digitalWrite(BAT_EN, HIGH);
  pinMode(PWR_KEY, INPUT_PULLUP);
  pinMode(BOOT_KEY, INPUT_PULLUP);

  Serial.begin(115200);
  delay(200);
  Serial.println("\n[boot] AsaWatch -- wajah satu halaman");

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
   * touch permanen: register beku di finger=1 dengan IRQ membanjir ~80/detik.
   *
   * Terukur lewat uji A/B terkontrol pada board ini:
   *   touch.begin() -> gfx->begin() : fingers=0, irq/s=0     (bersih, 48 detik)
   *   gfx->begin() -> touch.begin() : fingers=1, irq/s=80    (ghost, konsisten)
   * ======================================================= */
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);   /* 400k tidak stabil untuk CST816T di board ini */
  pinMode(TOUCH_IRQ, INPUT_PULLUP);
  bool touch_ada = touch.begin(Wire, CST816_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
  if (!touch_ada) {
    Serial.println("[err] CST816 tidak terdeteksi -- wajah ini memang tidak butuh sentuh");
  } else {
    Serial.printf("[ok] touch: %s\n", touch.getModelName());
  }
  delay(150);
  /* IRQ hanya dipasang kalau chipnya benar-benar menjawab. Pada board yang
   * sentuhannya rusak, pin IRQ yang menggantung bisa memicu interupsi liar, dan
   * setiap satu di antaranya membuat loop() melewati ppg_update(). */
  if (touch_ada) attachInterrupt(digitalPinToInterrupt(TOUCH_IRQ), touch_isr, FALLING);

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

  /* Indev penunjuk hanya didaftarkan kalau chip sentuhnya menjawab. Chip yang
   * rusak dan terkunci di "jari menempel" membuat LVGL melihat tekanan abadi,
   * dan gejalanya layar yang seolah membeku. */
  if (touch_ada) {
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touch_read;
    lv_indev_drv_register(&indev_drv);
  }

  /* RTC berbagi bus I2C dengan touch, dan Wire sudah di-begin di atas. */
  rtc_begin();
  tm_begin();
  battery_begin();
  ppg_begin();

  build_wajah();
  lv_scr_load(scr_wajah);

  lv_timer_create(refresh_cb, 500, NULL);

  lv_timer_handler();
  ledcWrite(LCD_BL, 204);   /* ~80% */

  /* Arming tombol BOOT ditentukan dari keadaan pin, bukan ditunggu sebagai
   * tepi. boot_poll() hanya bereaksi pada PERUBAHAN level, jadi tombol yang
   * sudah dilepas sejak boot tidak pernah menghasilkan tepi apa pun dan
   * armingnya tak pernah datang -- gejalanya halus: tekanan pertama pengguna
   * terbuang. Yang sebenarnya perlu dijaga cuma satu keadaan, yaitu tombol yang
   * MASIH ditahan di sini (GPIO9 strapping pin, jadi ini kejadian normal
   * sehabis flash). Membacanya sekali menjawab keduanya. */
  boot_stabil_lvl = digitalRead(BOOT_KEY);
  boot_level_lalu = boot_stabil_lvl;
  boot_siap       = (boot_stabil_lvl == HIGH);
  Serial.printf("[boot] tombol BOOT %s\n",
                boot_siap ? "siap (tekan lama = cek manual / selesai makan)"
                          : "masih ditahan -- lepaskan dulu");

  /* Paling akhir: UI sudah tampil sebelum radio mulai menyita CPU dan heap.
   *
   * AsaWatch dulu, baru Wi-Fi. Urutannya penting untuk heap: init NimBLE
   * meminta blok yang relatif besar sekaligus, dan lebih mudah didapat sebelum
   * stack Wi-Fi memfragmentasi heap dengan buffer-buffernya.
   *
   * jam_mulai() sendiri punya urutan internal yang tidak boleh dibalik
   * (NVS -> boot_id naik -> muat ring -> sesi dipaksa IDLE -> BLE -> event
   * BOOT); lihat aw_jam.cpp. */
  jam_mulai();
  net_begin();

  Serial.printf("[ok] setup selesai, free heap = %lu\n", (unsigned long)ESP.getFreeHeap());
}

void loop() {
  touch_poll();          /* tetap dibaca demi board yang sentuhannya sehat */
  pwr_poll();            /* satu digitalRead; tombol mati harus selalu responsif */
  boot_poll();

  /* PPG ditunda selama masih ada IRQ touch yang belum diproses. Keduanya berbagi
   * bus I2C, dan satu transaksi FIFO MAX30105 (~1 ms di 100 kHz) cukup untuk
   * menunda pembacaan touch yang datanya hilang hampir seketika setelah IRQ. */
  if (!touch_irq_flag) ppg_update();

  /* Logika protokol berjalan di task yang SAMA dengan lv_timer_handler()
   * (dokumen 13.2). Yang menyeberang task tinggal satu: antrean perintah BLE
   * di aw_ble. */
  jam_putar();

  lv_timer_handler();
  delay(2);
}
