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
 * SATU TOMBOL, DUA MAKNA (protokol v1.3). Artinya ditentukan keadaan jam, dan
 * LAYAR INI yang menuliskannya -- lihat status_baris() dan tombol_arti(). Ada
 * ARM_TITIK berarti "Ukur"; ARMED berarti "Selesai Makan"; IDLE tanpa keduanya
 * jatuh ke cek manual yang hasilnya berhenti di layar ini. Pemilihan antara dua
 * makna protokolnya ada di aw_jam, bukan di sini: ARM_TITIK bisa tiba lewat BLE
 * antara layar membaca keadaan dan jari diangkat.
 *
 * JAM TIDAK MENJADWALKAN APA PUN lagi. Sebabnya bukan sensor melainkan daya:
 * jam bertahan ~50 menit menyala sedangkan satu sesi lebih dari dua jam, jadi ia
 * PASTI dimatikan sebelum titik berikutnya jatuh tempo. Yang menjadwalkan
 * sekarang aplikasi, lewat UKUR dan ARM_TITIK. Satu-satunya pengukuran atas
 * inisiatif jam yang tersisa adalah index 1, dan itu akibat langsung tombol yang
 * baru ditekan, bukan tenggat.
 *
 * SELAMA MENGUKUR keempat kartu menampilkan bacaan langsung yang terus berubah,
 * termasuk yang belum lolos gerbang kirim (ditandai warna lebih redup). Yang
 * dikirim ke aplikasi tetap hanya angka yang sudah stabil; lihat field `awal` di
 * ppg.h untuk pemisahan "boleh ditampilkan" dari "boleh dikirim".
 *
 * TOMBOL PWR punya dua arti yang dibedakan dari lamanya tekanan: tahan 3 detik
 * menyalakan atau mematikan jam sungguhan, klik singkat cuma memadamkan layar
 * (jam tetap berjalan). Gerbang tiga detik itu berlaku di kedua arah -- lihat
 * pwr_gerbang_nyala() untuk sisi menyalanya dan pwr_poll() untuk sisi matinya.
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
 *   aw_proto/aw_store/  - AsaWatch: protokol BLE v1.3, ring buffer NVS, mesin
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
#include "splash_assets.h"

#include "config.h"
#include "rtc.h"
#include "time_manager.h"
#include "net.h"
#include "weather.h"
#include "battery.h"
#include "ppg.h"
#include "aw_jam.h"
#include "aw_ble.h"

/* Ditaruh SETINGGI ini, jauh dari satu-satunya pemakainya di bawah, karena
 * praprosesor Arduino menyisipkan prototipe otomatis tepat setelah blok #include
 * -- termasuk untuk tombol_arti(), yang mengembalikan tipe ini. Typedef yang
 * ditulis di dekat fungsinya akan berada DI BAWAH prototipenya sendiri, dan
 * pesan galatnya ("does not name a type") tidak menunjuk ke sebabnya sama
 * sekali.
 *
 * Apa arti tombol fisik saat ini. HANYA untuk label dan pesan -- tidak pernah
 * untuk memilih fungsi mana yang dipanggil (dokumen 12 poin 5 & 13.4). */
typedef enum { TBL_MATI = 0, TBL_UKUR, TBL_SELESAI_MAKAN, TBL_CEK_MANUAL } tombol_arti_t;

/* BATT_N_KOTAK dan batt_widget_t juga harus di sini, alasan sama persis dengan
 * tombol_arti_t di atas: batt_gambar()/batt_buat() memakai batt_widget_t*
 * sebagai parameter, dan prototipe otomatisnya disisipkan tepat di bawah blok
 * #include -- sebelum typedef yang ditulis di dekat pemakainya sendiri sempat
 * ada. Definisi ukuran/posisi ikon baterai yang sesungguhnya tetap di bagian
 * Geometri seperti biasa; ini cuma bentuk handle-nya. */
#define BATT_N_KOTAK 3
typedef struct {
  lv_obj_t *cangkang, *nub, *petir;
  lv_obj_t *kotak[BATT_N_KOTAK];
} batt_widget_t;

/* Subset digit-only dari montserrat_48 (cuma glyph '-' '.' '/' dan 0-9),
 * dipakai untuk jam besar di halaman KEDUA (wajah metrik). Font bawaan LVGL
 * di ukuran itu ikut membawa seluruh ASCII + ikon FontAwesome (~90 KB) yang
 * tak pernah dirender di sini. font_digits_46 tetap ada di direktori sketsa
 * tapi tidak lagi dipakai.
 *
 * font_jam_home: subset serupa tapi Montserrat BOLD 64px, dipakai KHUSUS jam
 * besar halaman UTAMA (lihat font_jam_home.c untuk alasan bold-nya). Dua font
 * beda, bukan satu dipakai dua kali -- lihat komentar di build_home() untuk
 * kenapa keduanya tidak (lagi) berbagi geometri. */
extern "C" {
LV_FONT_DECLARE(font_digits_48)
LV_FONT_DECLARE(font_jam_home)
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

/* ================= Layar: hidup / mati =================
 * Ini yang dilakukan KLIK SINGKAT tombol PWR, dan ia satu-satunya bentuk
 * "on/off" yang bisa dibolak-balik firmware pada board ini.
 *
 * Mematikan jam sungguhan berarti melepas latch BAT_EN, dan itu jalan satu arah:
 * begitu jalur baterai terputus, tidak ada lagi firmware yang berjalan untuk
 * menyambungnya kembali -- yang menyalakan board lagi adalah tombol PWR secara
 * fisik (firmware cuma menilai lamanya, lihat pwr_gerbang_nyala()). Karena itu
 * on/off sungguhan dipegang tekan-lama 3 detik, dan klik singkat -- yang harus
 * tetap punya arti pada jam tangan -- dipetakan ke layar.
 *
 * Dua-duanya dikerjakan: backlight DAN perintah SLPIN ke panel. Backlight saja
 * sudah membuat layar gelap, tetapi ST7789-nya tetap menyegarkan 240x280 piksel
 * sepanjang malam. Urutannya penting di kedua arah: matikan cahaya dulu baru
 * panel (kalau dibalik, panel yang masuk sleep sempat terlihat berkedip), dan
 * saat menyalakan, panel dulu -- cahayanya menunggu sampai satu frame utuh
 * tergambar, lihat s_bl_tunda di loop(). */
#define LCD_BL_TERANG  204        /* ~80%, sama dengan yang dipakai setup() */

static bool s_layar_nyala = true;
static bool s_bl_tunda    = false;

/* true setelah latch BAT_EN dilepas tetapi board ternyata masih hidup -- artinya
 * ia dicatu USB, bukan baterai. Di baterai, baris setelah digitalWrite(BAT_EN,
 * LOW) tidak pernah dieksekusi. Diletakkan di sini, bukan di dekat pwr_poll(),
 * karena layar_set() harus melihatnya: jam yang sudah diminta mati tidak boleh
 * menyalakan layarnya sendiri lagi karena ada pengukuran yang jatuh tempo. */
static bool pwr_daya_lepas = false;

/* true kalau halaman kedua (wajah metrik) yang sedang tampil, false kalau
 * halaman utama. Dideklarasikan di sini, jauh dari halaman_set()/
 * halaman_evaluasi() yang memakainya (ada di bagian "Pembaruan isi" di
 * bawah), karena splash_tutup() -- yang ada JAUH lebih awal di berkas ini --
 * juga perlu menyetelnya saat menentukan halaman awal. Arduino menyisipkan
 * prototipe otomatis untuk FUNGSI, tidak untuk variabel, jadi variabel globalnya
 * sendiri harus sudah ada sebelum pemakaian pertamanya. */
static bool     s_di_wajah = false;
static uint32_t s_pulang_home_pada = 0;   /* 0 = tidak ada tenggat berjalan */

/* Layar pembuka. Definisinya jauh di bawah (butuh scr_wajah dan helper widget),
 * tetapi layar_set() adalah satu-satunya pintu menyala/mati layar dan karena itu
 * juga satu-satunya tempat yang benar untuk memulai dan membatalkannya.
 * Dideklarasikan eksplisit alih-alih menumpang prototipe otomatis Arduino,
 * supaya urutan berkas tidak diam-diam menentukan apakah ini ter-compile. */
static void splash_mulai(void);
static void splash_batal(void);

/* 1 = logo tampil juga setiap layar dinyalakan kembali, bukan cuma saat boot.
 *
 * Ini keputusan produk, bukan detail teknis, jadi ditaruh sebagai sakelar yang
 * terlihat. Sisi baiknya: layar tidak pernah menyala dengan wajah jam basi
 * sepersekian detik sebelum digambar ulang.
 *
 * DIMATIKAN, dan yang mematikannya adalah auto-mati layar. Selama layar
 * praktis tidak pernah padam sendiri, logo saat bangun itu peristiwa langka
 * dan terasa mewah. Begitu layar mati tiap 20-30 detik, ia berubah jadi pajak
 * yang dibayar berkali-kali sehari: setiap kali ingin melihat jam, SPL_TOTAL_MS
 * berdiri lebih dulu di depan angkanya.
 *
 * Ongkos yang sama sudah lama berlaku di tempat yang lebih buruk -- sejak v1.2
 * layar menyala sendiri saat pengukuran dimulai (lihat refresh_cb), dan di situ
 * splash berdiri persis di antara pengguna dan isyarat "tempelkan jari
 * sekarang".
 *
 * Splash saat BOOT tidak terpengaruh: setup() memanggil splash_mulai() sendiri,
 * tidak lewat sakelar ini. */
#define SPL_SAAT_BANGUN 0

/* ---- Auto-mati layar ----
 * Sebelumnya jam ini TIDAK punya batas waktu sama sekali: sekali menyala ia
 * menyala sampai tombol ditekan lagi. Yang membuat itu tidak bisa dipertahankan
 * adalah satu angka -- jam bertahan sekitar 50 menit dengan layar menyala. Jadi
 * sekali klik PWR untuk melihat jam lalu lupa mengkliknya lagi, dan baterainya
 * habis total dalam kurang dari sejam. Tidak ada yang mencegahnya, dan
 * kehilangannya senyap: yang terlihat cuma jam yang mati padahal tadi penuh.
 *
 * Dua tenggat, karena dua maksud yang berbeda:
 *   TOMBOL -- pengguna MEMINTA melihat sesuatu, jadi diberi waktu membaca
 *             empat kartu metrik dengan tenang.
 *   AUTO   -- jam yang memberi tahu sesuatu tanpa diminta (kabel masuk,
 *             pengukuran mulai). Isyaratnya cuma berguna sebentar; mengisi
 *             sendiri berjam-jam dan sering semalaman.
 *
 * Tenggatnya dipasang PEMANGGIL, bukan layar_set(), supaya setiap penyalaan
 * menyatakan maksudnya sendiri -- dan supaya layar_set(false) tetap punya satu
 * arti saja. */
#define LAYAR_MATI_TOMBOL_MS  20000UL
#define LAYAR_AUTO_MATI_MS    10000UL
static uint32_t layar_mati_pada = 0;   /* 0 = tanpa batas waktu */

static void layar_set(bool nyala) {
  if (nyala && pwr_daya_lepas) return;
  if (nyala == s_layar_nyala) return;
  s_layar_nyala = nyala;

  /* Setiap perubahan keadaan membatalkan tenggat yang berjalan. Penyalaan yang
   * diminta pengguna karena itu selalu abadi, walau datang di tengah 20 detik
   * penyalaan otomatis -- yang belakangan menang, dan itu yang diharapkan. */
  layar_mati_pada = 0;

  /* Dilaporkan SEBELUM duty backlight berubah -- inilah "nilai sebelum" bagi
   * probe sag di battery.cpp, yang darinya jam tahu ia sedang dicolok atau
   * tidak. Layar mati/nyala adalah satu-satunya langkah beban besar yang
   * terjadi rutin, jadi di sinilah satu-satunya tempat yang benar. */
  battery_beban_akan_berubah(nyala);

  if (nyala) {
    gfx->displayOn();
#if SPL_SAAT_BANGUN
    /* Memuat layar pembuka sekaligus meng-invalidate seluruh piksel, jadi ini
     * menggantikan lv_obj_invalidate() di bawah, bukan menambahinya. */
    splash_mulai();
#else
    /* Isi RAM panel tidak dijamin selamat dari SLPIN, jadi seluruh layar
     * digambar ulang alih-alih mengandalkan apa yang tersisa di sana. */
    lv_obj_invalidate(lv_scr_act());
#endif
    s_bl_tunda = true;
  } else {
    /* Dibatalkan SEBELUM panel dipadamkan. Animasi yang dibiarkan berjalan di
     * balik layar mati bukan cuma membuang CPU: timer penutupnya akan memuat
     * wajah jam beberapa ratus milidetik kemudian, sehingga penyalaan
     * berikutnya menemukan splash yang sudah separuh jalan dan memulainya dari
     * tengah. */
    splash_batal();
    ledcWrite(LCD_BL, 0);
    gfx->displayOff();
    s_bl_tunda = false;
  }
  Serial.printf("[pwr] layar %s\n", nyala ? "menyala" : "dimatikan");
}

/* Nyala dengan tenggat. Urutannya penting: layar_set() mengosongkan tenggat,
 * jadi tenggat baru harus dipasang SESUDAHNYA. Kalau layar sudah menyala,
 * layar_set() langsung kembali dan tenggat tidak dipasang -- juga disengaja:
 * layar yang sedang dipakai pengguna tidak boleh mati gara-gara kabel dicolok. */
static void layar_nyala_sementara(uint32_t ms) {
  const bool sudah_nyala = s_layar_nyala;
  layar_set(true);
  if (!sudah_nyala && s_layar_nyala) layar_mati_pada = millis() + ms;
}

/* Glyph non-ASCII yang tersedia di lv_font_montserrat_* bawaan LVGL. */
#define TXT_DEG  "\xC2\xB0"      /* U+00B0 derajat */
#define TXT_DOT  "\xE2\x80\xA2"  /* U+2022 bullet  */

/* ================= Palet =================
 * Setiap nilai di bawah adalah warna yang benar-benar diambil dari piksel
 * wajah_jam_modular_240x280.png, bukan tafsiran. */
#define C_BG_ATAS   0x06070B   /* latar di tepi atas   */
#define C_BG_BAWAH  0x0D0C18   /* latar di tepi bawah  */

/* Kartu halaman kedua: dikembalikan ke warna asli wajah_jam_modular_240x280.png
 * atas permintaan langsung ("buat seperti ini lagi") -- sempat diganti ke
 * sampel dari WhatsApp Image 2026-09-02 at 00.33.37.jpeg, tapi itu dibatalkan. */
#define C_KARTU     0x171A21
#define C_KARTU_BRD 0x23262D
#define C_PUTIH     0xFFFFFF   /* angka besar & jam    */
#define C_TANGGAL   0xB9C1D0   /* baris tanggal        */
#define C_REDUP     0x8E96A6   /* judul kartu, satuan, ikon baterai   */
#define C_ISI       0xA2E32B   /* petir "sedang mengisi" */
/* Jalur di belakang cincin. Sebelumnya 0x2E3444, kontras baik terhadap latar
 * navy nyaris-hitam yang lama (C_BG_ATAS/C_BG_BAWAH). Sejak latar halaman ini
 * disamakan dengan C_HOME_BG_BODY (teal, jauh lebih terang), warna lama itu
 * nyaris melebur -- luminansinya nyaris sama, cuma rona yang beda, dan mata
 * jauh lebih peka pada beda terang-gelap daripada beda rona. Disamakan dengan
 * C_HOME_VSEP: sudah terbukti terlihat di garis pemisah header halaman utama
 * di atas latar yang sama persis. */
#define C_TRACK     0x3D747B
#define C_CINCIN_HR 0xFF3B5C
#define C_CINCIN_SP 0xA2E32B
#define C_CINCIN_GL 0x22D3EE

/* ---- Palet halaman utama (home) ----
 * Diambil dari piksel 1baru.png dengan sample langsung (bukan tafsiran),
 * sama seperti palet di atas. Skema warnanya beda sengaja -- teal + emas,
 * bukan navy + neon -- karena halaman ini "kartu ucapan" yang dilihat
 * sepanjang hari, sementara wajah jam adalah dasbor metrik yang hanya
 * tampil saat ada pengukuran. */
#define C_HOME_BG_HDR   0x04363D   /* latar bilah header       */
#define C_HOME_BG_BODY  0x034B55   /* latar badan halaman      */
#define C_HOME_DIVIDER  0xB79E2E   /* garis pemisah header     */
#define C_HOME_VSEP     0x3D747B   /* garis pemisah tegak      */
#define C_HOME_GOLD     0xECC94B   /* wajik & titik dua jam    */
#define C_HOME_MUTED    0xCFE3DE   /* nama hari, tanggal       */

/* ================= Geometri =================
 * Cincin di PITA HORIZONTAL DI TENGAH, persis di antara baris kartu atas dan
 * bawah, dan karena x pusatnya juga di tengah layar, ia otomatis berjarak
 * sama dari kartu kiri MAUPUN kanan, di kedua baris.
 *
 * Baris atas dan baris bawah HAMPIR bertemu di garis tengah (KARTU_TENGAH,
 * pertengahan GEOMETRI_ATAS/BAWAH) -- disisakan celah KARTU_GAP kecil di
 * antara keduanya (bukan 0, dan bukan pula jarak napas lebar seperti
 * sebelumnya) supaya kedua baris tidak terlihat "bertabrakan" lewat garis
 * bordernya sendiri. Cincin tetap dipusatkan tepat di KARTU_TENGAH dan
 * radiusnya jauh lebih besar daripada celah ini -- ia SENGAJA menimpa kedua
 * baris kartu, dan lencana di baliknya (lihat build_wajah()) yang membuat
 * tumpang-tindih itu terlihat sebagai fondasi, bukan tabrakan.
 *
 * Lebar dan posisi X kartu TIDAK berubah dari desain asli:
 *   kartu kiri  x=10..115   kartu kanan x=124..229   (lebar 106, jarak 8) */
#define KARTU_W        106
#define KARTU_X1        10
#define KARTU_X2       124

#define GEOMETRI_ATAS   32    /* di bawah baris status (y13) & baterai (y14+14) */
#define GEOMETRI_BAWAH 276    /* sisa 4 px ke tepi layar (280)                  */
#define KARTU_GAP        8    /* celah tegak antar baris -- sama seperti jarak
                                * mendatar antar kolom (KARTU_X2 - KARTU_X1 - KARTU_W) */

#define KARTU_TENGAH  ((GEOMETRI_ATAS + GEOMETRI_BAWAH) / 2)          /* 154 */
#define KARTU_Y1      GEOMETRI_ATAS
#define KARTU_H       (KARTU_TENGAH - KARTU_GAP / 2 - KARTU_Y1)       /* 118 */
#define KARTU_Y2      (KARTU_TENGAH + KARTU_GAP / 2)                  /* 158 */

/* Cincin dipusatkan di tengah layar (x=120) dan tepat di KARTU_TENGAH --
 * diturunkan dari tata letak kartu supaya keduanya tidak bisa diam-diam
 * saling menyimpang kalau salah satu diubah lagi. */
#define CINCIN_CX (SCREEN_W / 2)
#define CINCIN_CY KARTU_TENGAH
#define CINCIN_W  7

/* Blok isi kartu (ikon+judul di atas, nilai+satuan di bawahnya) tingginya
 * sekitar 52 px (relatif y=8 sampai dasar nilai di y=26+~34) -- angka tetap
 * dari font_nilai terbesar yang dipakai (montserrat_30), bukan dihitung
 * ulang per kartu. KARTU_ISI_DY memusatkannya vertikal dalam kartu yang kini
 * jauh lebih tinggi (118px) daripada blok isinya sendiri; tanpa ini isi kartu
 * akan menempel di atas dengan rongga kosong besar di bawahnya -- persis
 * kesan "tata letak berantakan" yang diminta dirapikan. */
#define KARTU_ISI_DY  ((KARTU_H - 52) / 2 - 8)

/* ---- Ikon baterai berkotak ----
 * Bentuknya mengikuti images (2).jpeg. Proporsinya diukur dari gambar itu, di
 * garis tengah badannya: garis 17, jarak 8, lalu kotak 60 - celah 12 - kotak 60
 * - celah 12 - kotak 60, jarak 8, garis 17 -- total badan 254 px.
 *
 * Yang penting dari angka-angka itu bukan nilainya, tapi PERBANDINGANNYA:
 * celah selebar seperlima kotak, dan kotak tidak menempel ke garis badan.
 * Percobaan pertama mengabaikan keduanya (badan 24 px, celah 1 px, kotak
 * mengisi rongga sampai mepet garis) dan hasilnya ketiga kotak melebur jadi
 * satu blok -- jumlahnya tidak bisa dihitung mata, yang menghapus seluruh
 * gunanya. Badan dilebarkan ke 28 px supaya celah 2 px dan jarak 1 px muat.
 *
 * TIGA kotak, bukan empat atau lima, dan itu keputusan soal kejujuran bukan
 * soal ruang. Persen dari tegangan Li-Po hanya bisa dipercaya sampai sekitar
 * +-5..10% (lihat battery.h). Lima kotak berarti tiap kotak bernilai 20% --
 * lebih halus daripada yang benar-benar diketahui, sehingga kotak paling bawah
 * akan berkedip-kedip mengikuti derau, bukan mengikuti daya. Tiga kotak
 * membuat satu langkah bernilai ~33%, nyaman di atas ambang kesalahan itu:
 * setiap perubahan yang terlihat di layar adalah perubahan yang nyata.
 *
 * Tepi kanan dikunci di x=232 supaya sejajar dengan tepi kanan kartu di
 * bawahnya -- sama seperti angka persen yang digantikannya. */
/* BATT_N_KOTAK didefinisikan di dekat tombol_arti_t, dekat kepala berkas --
 * lihat komentar di sana kenapa. */
#define BATT_BRD     2                       /* tebal garis badan            */
#define BATT_PAD     1                       /* jarak kotak ke garis badan   */
#define BATT_KOTAK_W 6
#define BATT_KOTAK_H 8
#define BATT_CELAH   2                       /* celah antar kotak            */
#define BATT_KOTAK_D (BATT_KOTAK_W + BATT_CELAH)             /* langkah      */
#define BATT_NUB_W   3
#define BATT_NUB_H   7

/* Badan dihitung DARI ISINYA, bukan sebaliknya. Menetapkan lebar badan lebih
 * dulu lalu membagi rongganya bertiga tidak pernah habis dibagi rata, dan sisa
 * satu piksel itu selalu jatuh di salah satu celah sehingga ketiga kotak
 * terlihat tidak sama jaraknya. Dengan arah hitung dibalik, ukuran badan
 * dijamin pas: 3x6 kotak + 2x2 celah + 2x1 jarak + 2x2 garis = 28. */
#define BATT_W  (BATT_N_KOTAK * BATT_KOTAK_W + (BATT_N_KOTAK - 1) * BATT_CELAH \
                 + 2 * BATT_PAD + 2 * BATT_BRD)                       /* 28 */
#define BATT_H  (BATT_KOTAK_H + 2 * BATT_PAD + 2 * BATT_BRD)          /* 14 */

/* Posisinya kini per-layar (halaman utama DAN halaman kedua masing-masing
 * punya ikonnya sendiri, lihat batt_widget_t), jadi tepi kanan/atas diteruskan
 * sebagai argumen ke batt_buat() alih-alih dipatok satu makro seperti dulu. */
#define BATT_KANAN_WAJAH 232                 /* tepi kanan tonjolan, halaman kedua */
#define BATT_Y_WAJAH     14                  /* sejajar teks header halaman kedua  */
#define BATT_KANAN_HOME  228                 /* diukur dari 1baru.png / 2          */
#define BATT_Y_HOME       8                  /* sejajar teks header halaman utama  */

/* ================= Geometri: halaman utama (home) =================
 * Diukur dari 1baru.png (mockup 2x = 480x560) dibagi 2, persis konvensi yang
 * sama dipakai wajah_jam_modular_240x280.png. Lihat pesan commit/PR untuk skrip
 * pengukuran pikselnya kalau tata letak ini perlu diubah lagi. */
#define HOME_HDR_H       31    /* tinggi bilah header, sampai garis pemisah  */
#define HOME_VSEP_X     120    /* garis pemisah tegak di header, tengah layar */

/* img_plane (ilustrasi pesawat kertas, sisa desain 7-layar lama) sengaja
 * TIDAK dipakai lagi di sini -- permintaan langsung supaya halaman utama
 * tidak ada pesawatnya. Ruang 122px yang dulu dipakainya (y 50-172) itulah
 * yang sekarang membiarkan jam besar dan blok hari/tanggal dipusatkan
 * vertikal di bawah, alih-alih berdesakan di bagian bawah layar seperti
 * sebelumnya. img_weather dan img_diamond TETAP dipakai. */

/* Pemisah "garis - wajik - garis" di atas nama hari. Y ini plus seluruh blok
 * hari/jam/tanggal di bawahnya dipusatkan vertikal pada badan layar (di bawah
 * header, 31-280px): jarak header->pemisah dan tanggal->tepi bawah sengaja
 * dibuat sama (55px) supaya blok ini terasa mengambang di tengah, bukan
 * menempel ke salah satu tepi. */
#define HOME_SEP_Y       91
#define HOME_SEP_GAP      3    /* jarak ujung garis ke wajik                 */
#define HOME_SEP_LINE_W  28

#define HOME_HARI_Y     108
#define HOME_TANGGAL_Y  210

/* Jam besar, font_jam_home (Montserrat Bold 64px) -- BUKAN font_digits_48
 * yang dipakai halaman kedua. Sebelum halaman ini dirombak (hapus pesawat)
 * keduanya sengaja berbagi font supaya geometrinya identik; sekarang jam di
 * sini jauh lebih besar (ink ~47px vs ~34px) jadi offset titik dua dan
 * lebar kotak HH/MM di bawah ini dihitung ULANG dari metrik font_jam_home,
 * BUKAN diturunkan dari halaman kedua lagi -- jangan disamakan lagi tanpa
 * menghitung ulang dari font_jam_home.c.
 *
 * Titik dua di sini emas, bukan putih, karena desainnya begitu (lihat
 * 1baru.png). Lebar kotak HH/MM (94px) dan GAP (8px) dipilih supaya kombinasi
 * dua-digit terlebar font_jam_home (mis. "04"/"44", ~89px) tetap muat dengan
 * sisa ruang di kedua tepi layar -- JANGAN diperkecil tanpa mengukur ulang
 * adv_w terlebar di font_jam_home.c kalau font itu diganti lagi. */
#define HOME_JAM_Y       143
#define HOME_JAM_W        94   /* cukup utk kombinasi 2 digit terlebar font_jam_home */
#define HOME_TITIK_DUA_X 114   /* 120 - lebar/2 */
#define HOME_TITIK_DUA_W  12
#define HOME_JAM_GAP       8   /* jarak digit ke titik dua, kedua sisi */

/* Offset vertikal kedua kotak titik dua, relatif ke HOME_JAM_Y -- dihitung
 * dari ink height font_jam_home (47px) dibagi rata: pra-jarak, kotak,
 * jarak-tengah, kotak masing-masing ~12/12/11/12px. Lihat font_jam_home.c
 * (glyph_dsc, line_height=47) kalau ukuran fontnya berubah lagi. */
#define HOME_JAM_DOT1_DY  12
#define HOME_JAM_DOT2_DY  35

/* ================= SUMBER DATA =================
 * Tidak ada angka yang dikarang di layar ini. Kalau sebuah nilai belum
 * tersedia -- jari tidak menempel, sensor tidak terpasang, ADC belum stabil --
 * yang tampil adalah "--", bukan angka contoh. Pada layar kesehatan, angka
 * contoh yang tampak nyata lebih berbahaya daripada tanda hubung.
 *
 *   jam / hari / tanggal   -> RTC PCF85063, disinkronkan dari jam HP setiap kali
 *                             BLE tersambung (ANCHOR_WAKTU) dan dari NTP kalau
 *                             Wi-Fi ada
 *   persen baterai         -> ADC1 + kurva Li-Po (battery.cpp)
 *   detak, SpO2, glukosa,  -> MAX30105/30102 lewat jam_snapshot()
 *   tekanan darah             glukosa & tekanan EKSPERIMENTAL, lihat ppg.h
 */

/* ================= Handle widget ================= */
static lv_obj_t *scr_wajah;
static lv_obj_t *scr_home;

static lv_obj_t *lbl_status;                /* baris tanggal / status         */

/* Ikon baterai digambar sendiri dari objek LVGL, bukan glyph LV_SYMBOL_*.
 * Sebabnya bukan selera: lambang baterai di font hanya punya lima tingkat
 * tetap dan tidak bisa diberi tahu berapa kotak yang boleh menyala, jadi
 * satu-satunya cara menampilkan tepat tiga kotak adalah menggambarnya.
 *
 * Sejak halaman utama ditambahkan, ikonnya ada DUA SALINAN -- satu objek LVGL
 * cuma boleh py satu induk, jadi halaman utama dan halaman kedua tidak bisa
 * berbagi objek yang sama. batt_buat()/batt_gambar() dipanggil dua kali,
 * sekali per struct, dan keduanya selalu disegarkan bersamaan (lihat
 * refresh_cb) supaya tidak ada halaman yang menampilkan angka basi begitu
 * pengguna berpindah. batt_widget_t sendiri ada di dekat tombol_arti_t di
 * kepala berkas -- lihat komentar di sana. */
static batt_widget_t batt_wajah, batt_home;

static lv_obj_t *cincin_hr, *cincin_sp, *cincin_gl;

static lv_obj_t *lbl_hr, *lbl_sp, *lbl_gl, *lbl_bp;          /* angka besar   */
static lv_obj_t *sat_hr, *sat_sp, *sat_gl;                   /* satuan        */

/* ---- Halaman utama ---- */
static lv_obj_t *lbl_home_suhu, *lbl_home_kondisi;
static lv_obj_t *lbl_home_hari, *lbl_home_tanggal;
static lv_obj_t *lbl_home_hh, *lbl_home_mm;

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

/* Isi satu kuadran kartu metrik pada `parent`: ikon, judul, angka besar,
 * satuan -- TANPA kotak/border sendiri. Dulu tiap kartu punya kotaknya
 * sendiri-sendiri (mk_kartu() lama), tapi atas permintaan langsung keempat
 * kuadran digabung jadi SATU kartu (lihat build_wajah()) supaya terlihat
 * lebih rapi -- kotaknya kini digambar sekali oleh pemanggil, dan fungsi ini
 * cuma menempelkan isi di titik (x,y) yang diberikan, relatif terhadap
 * `parent` (di sini selalu scr_wajah, jadi x/y-nya tetap koordinat layar
 * penuh persis seperti KARTU_X1/X2/Y1/Y2).
 *
 * font_nilai diparameterkan karena kuadran tekanan memuat enam karakter
 * ("118/76") sedangkan yang lain paling banyak tiga -- memaksakan satu ukuran
 * untuk keempatnya akan membuat tekanan meluber. Gambar desainnya sendiri
 * sudah merender kuadran itu lebih kecil (tinggi ink 21 px lawan 23 px), jadi
 * ini mengikuti desain, bukan menyimpang darinya. */
static void mk_isi_kartu(lv_obj_t *parent, int x, int y, const lv_img_dsc_t *ikon,
                         const char *judul, const char *satuan,
                         const lv_font_t *font_nilai,
                         lv_obj_t **out_nilai, lv_obj_t **out_satuan) {
  /* +KARTU_ISI_DY di ketiga y: memusatkan blok ikon/judul/nilai vertikal
   * dalam kartu yang sekarang jauh lebih tinggi daripada blok isinya sendiri
   * -- lihat definisi makronya. */
  mk_img(parent, ikon, x + 10, y + 9 + KARTU_ISI_DY);
  /* Kembali ke C_REDUP untuk judul & satuan, seperti desain asli -- sempat
   * diputihkan mengikuti referensi WhatsApp, tapi itu dibatalkan bersama
   * seluruh perombakan kartu ke gaya itu. */
  mk_label(parent, judul, &lv_font_montserrat_12, C_REDUP, x + 33, y + 8 + KARTU_ISI_DY);

  *out_nilai = mk_label(parent, "--", font_nilai, C_PUTIH, x + 10, y + 26 + KARTU_ISI_DY);
  if (satuan) {
    *out_satuan = mk_label(parent, satuan, &lv_font_montserrat_12, C_REDUP, 0, 0);
    satuan_sejajar(*out_satuan, *out_nilai, font_nilai);
  } else if (out_satuan) {
    *out_satuan = NULL;
  }
}

/* ================= Garis kemajuan di TEPI LAYAR =================
 * Garis hijau bercahaya yang merayap mengelilingi tepi layar selama pengukuran.
 * Perannya sama dengan lingkaran loading -- memperlihatkan bahwa sesuatu sedang
 * berjalan dan seberapa jauh -- hanya saja jalurnya persegi bersudut tumpul
 * mengikuti bentuk layar, sehingga tidak memakan satu piksel pun dari empat
 * kartu di tengah. Itu syarat yang tidak bisa ditawar di sini: wajah ini sudah
 * penuh, dan justru selama mengukur keempat kartu itu yang paling ingin dilihat.
 *
 * JALURNYA dimulai di TENGAH TEPI ATAS lalu searah jarum jam, sama seperti
 * lingkaran loading yang mulai di angka 12. Karena itu tepi atas terbagi dua
 * potong: setengah kanan tumbuh paling awal, setengah kiri menutup paling akhir.
 * Garis hanya bertemu titik awalnya saat kemajuan 100 -- dan tidak pernah
 * mundur; lihat jam_ukur_persen() di aw_jam.cpp untuk jaminan monotonnya.
 *
 * BENTUKNYA sembilan potong: lima ruas lurus dan empat sudut melengkung
 * berjari-jari TEPI_R. Ruas lurus adalah kotak tipis; sudutnya lv_arc seperempat
 * lingkaran yang nilainya bisa diisi sebagian, jadi kepala garis bergerak mulus
 * melewati tikungan alih-alih melompat dari satu sisi ke sisi berikutnya.
 *
 * Kenapa objek, bukan canvas: canvas seukuran layar butuh 240x280x2 = 134 KB,
 * sementara seluruh heap LVGL di board ini 48 KB. Sembilan potong yang cuma
 * diubah posisi, ukuran, dan nilainya tidak meminta memori tambahan sama sekali.
 *
 * CAHAYANYA dari lapisan kedua, bukan dari shadow. Shadow LVGL hanya mengikuti
 * kotak latar sebuah objek, jadi ia tidak bisa melengkung mengikuti sudut --
 * pada tikungan cahayanya akan terlihat sebagai kotak. Dua lapisan garis yang
 * berbagi satu sumbu (yang bawah lebih tebal dan tembus pandang) melengkung
 * dengan benar di seluruh jalur, dan digambar dengan cara yang sama persis. */
#define TEPI_INSET     4     /* jarak sumbu garis dari tepi layar */
#define TEPI_R        24     /* jari-jari sudut                   */
#define TEPI_LEBAR     3     /* tebal garis                       */
#define TEPI_HALO      9     /* tebal lapisan cahaya di bawahnya  */

/* Jenis potong. Sudut butuh lv_arc karena hanya arc yang bisa diisi sebagian
 * sambil tetap melengkung; ruas lurus cukup kotak. */
typedef enum { TP_DATAR, TP_TEGAK, TP_SUDUT } tepi_jenis_t;

typedef struct {
  tepi_jenis_t jenis;
  int panjang;     /* piksel sepanjang jalur                                  */
  int p1, p2;      /* DATAR: y sumbu, x mulai; TEGAK: x sumbu, y mulai;
                      SUDUT: cx, cy                                           */
  int p3;          /* DATAR/TEGAK: arah tumbuh +1/-1; SUDUT: sudut mulai (deg) */
} tepi_potong_t;

/* Panjang busur seperempat lingkaran: (pi/2) * 24 = 37,7 -> 38. Dibulatkan
 * sekali di sini supaya total jalur dan pembagian per potong memakai angka yang
 * sama; kalau tidak, sisa pembulatan akan menumpuk di potong terakhir dan garis
 * "menutup" beberapa piksel sebelum 100%. */
#define TEPI_BUSUR  38

/* Titik-titik sumbu jalur. Ditulis apa adanya, bukan lewat rumus, supaya mudah
 * dicocokkan dengan gambar desain. */
#define TEPI_X0  TEPI_INSET                         /*   4 */
#define TEPI_X1  (SCREEN_W - TEPI_INSET)            /* 236 */
#define TEPI_Y0  TEPI_INSET                         /*   4 */
#define TEPI_Y1  (SCREEN_H - TEPI_INSET)            /* 276 */
#define TEPI_XK  (TEPI_X0 + TEPI_R)                 /*  28, akhir tikungan kiri  */
#define TEPI_XN  (TEPI_X1 - TEPI_R)                 /* 212, awal tikungan kanan  */
#define TEPI_YA  (TEPI_Y0 + TEPI_R)                 /*  28 */
#define TEPI_YB  (TEPI_Y1 - TEPI_R)                 /* 252 */
#define TEPI_XT  (SCREEN_W / 2)                     /* 120, titik awal & akhir   */

static const tepi_potong_t TEPI[9] = {
  { TP_DATAR, TEPI_XN - TEPI_XT, TEPI_Y0, TEPI_XT, +1 },  /* atas, ke kanan   */
  { TP_SUDUT, TEPI_BUSUR,        TEPI_XN, TEPI_YA, 270 }, /* sudut kanan atas */
  { TP_TEGAK, TEPI_YB - TEPI_YA, TEPI_X1, TEPI_YA, +1 },  /* kanan, turun     */
  { TP_SUDUT, TEPI_BUSUR,        TEPI_XN, TEPI_YB,   0 }, /* sudut kanan bawah*/
  { TP_DATAR, TEPI_XN - TEPI_XK, TEPI_Y1, TEPI_XN, -1 },  /* bawah, ke kiri   */
  { TP_SUDUT, TEPI_BUSUR,        TEPI_XK, TEPI_YB,  90 }, /* sudut kiri bawah */
  { TP_TEGAK, TEPI_YB - TEPI_YA, TEPI_X0, TEPI_YB, -1 },  /* kiri, naik       */
  { TP_SUDUT, TEPI_BUSUR,        TEPI_XK, TEPI_YA, 180 }, /* sudut kiri atas  */
  { TP_DATAR, TEPI_XT - TEPI_XK, TEPI_Y0, TEPI_XK, +1 },  /* atas, menutup    */
};

/* Dua lapisan: [0] halo yang lebar dan tembus pandang, [1] garis yang tegas. */
static lv_obj_t *tepi_obj[2][9];

static lv_obj_t *mk_tepi_lurus(lv_obj_t *scr, uint8_t lebar, lv_opa_t opa) {
  lv_obj_t *o = mk_box(scr, 0, 0, 1, 1, C_ISI, lebar / 2);
  lv_obj_set_style_bg_opa(o, opa, 0);
  lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
  return o;
}

static lv_obj_t *mk_tepi_sudut(lv_obj_t *scr, uint8_t lebar, lv_opa_t opa,
                               int sudut_mulai) {
  int d = TEPI_R * 2 + lebar;
  lv_obj_t *a = lv_arc_create(scr);
  lv_obj_remove_style(a, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(a, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(a, d, d);
  lv_arc_set_rotation(a, 0);
  lv_arc_set_bg_angles(a, sudut_mulai, sudut_mulai + 90);
  lv_arc_set_range(a, 0, 1000);
  lv_arc_set_value(a, 0);
  /* Busur latar dimatikan: yang menggambar "belum tercapai" bukan tugas potong
   * ini -- di jalur lurus tidak ada padanannya, dan setengah jalur yang punya
   * jejak sementara setengahnya tidak akan terlihat seperti cacat gambar. */
  lv_obj_set_style_arc_opa(a, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_arc_width(a, lebar, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(a, lv_color_hex(C_ISI), LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(a, opa, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(a, true, LV_PART_INDICATOR);
  lv_obj_add_flag(a, LV_OBJ_FLAG_HIDDEN);
  return a;
}

static void build_tepi(lv_obj_t *scr) {
  static const uint8_t LEBAR[2] = { TEPI_HALO, TEPI_LEBAR };
  static const lv_opa_t OPA[2]  = { LV_OPA_30,  LV_OPA_COVER };

  for (int lap = 0; lap < 2; lap++) {
    for (int i = 0; i < 9; i++) {
      tepi_obj[lap][i] = (TEPI[i].jenis == TP_SUDUT)
        ? mk_tepi_sudut(scr, LEBAR[lap], OPA[lap], TEPI[i].p3)
        : mk_tepi_lurus(scr, LEBAR[lap], OPA[lap]);
    }
  }
}

/* Gambar potong ke-i sepanjang `panjang` piksel pada lapisan dengan tebal
 * `lebar`. Sumbu jalurnya sama untuk kedua lapisan -- yang berbeda cuma tebal,
 * jadi halo selalu berpusat tepat di bawah garisnya.
 *
 * Potongnya ditunjuk INDEKS, bukan pointer ke tepi_potong_t, dan itu bukan
 * selera: berkas .ino disisipi prototipe otomatis di bagian atas berkas, di atas
 * typedef ini, sehingga parameter bertipe tepi_potong_t gagal dikompilasi dengan
 * "does not name a type". Indeks int menghindari seluruh persoalan itu. */
static void tepi_potong_gambar(lv_obj_t *o, int i, int panjang, int lebar) {
  const tepi_potong_t *t = &TEPI[i];
  if (panjang <= 0) {
    lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
    return;
  }
  lv_obj_clear_flag(o, LV_OBJ_FLAG_HIDDEN);

  switch (t->jenis) {
    case TP_DATAR: {
      int x = (t->p3 > 0) ? t->p2 : (t->p2 - panjang);
      lv_obj_set_pos(o, x, t->p1 - lebar / 2);
      lv_obj_set_size(o, panjang, lebar);
      break;
    }
    case TP_TEGAK: {
      int y = (t->p3 > 0) ? t->p2 : (t->p2 - panjang);
      lv_obj_set_pos(o, t->p1 - lebar / 2, y);
      lv_obj_set_size(o, lebar, panjang);
      break;
    }
    default: {   /* TP_SUDUT */
      int d = TEPI_R * 2 + lebar;
      lv_obj_set_pos(o, t->p1 - d / 2, t->p2 - d / 2);
      lv_arc_set_value(o, (int32_t)((long)panjang * 1000 / t->panjang));
      break;
    }
  }
}

/* Panjang seluruh jalur. Dijumlah dari tabel, bukan dari keliling layar:
 * sudut yang melengkung memotong jalurnya, jadi keliling persegi akan terlalu
 * panjang dan garis tidak akan pernah benar-benar menutup di 100%. */
static int tepi_total(void) {
  static int total = 0;
  if (!total) for (int i = 0; i < 9; i++) total += TEPI[i].panjang;
  return total;
}

/* persen 0 menyembunyikan seluruh garis. Keluar lebih awal kalau angkanya tidak
 * berubah: mengubah posisi/ukuran objek LVGL meng-invalidate areanya, dan di
 * sini area itu melingkari seluruh tepi layar. */
static void tepi_set(int persen) {
  static int persen_lalu = -1;
  if (persen == persen_lalu) return;
  persen_lalu = persen;

  static const int LEBAR[2] = { TEPI_HALO, TEPI_LEBAR };

  for (int lap = 0; lap < 2; lap++) {
    long sisa = (long)tepi_total() * persen / 100;
    for (int i = 0; i < 9; i++) {
      int panjang = (sisa >= TEPI[i].panjang) ? TEPI[i].panjang
                                              : (sisa > 0 ? (int)sisa : 0);
      sisa -= panjang;
      tepi_potong_gambar(tepi_obj[lap][i], i, panjang, LEBAR[lap]);
    }
  }
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
  /* Latar rata, SAMA PERSIS dengan C_HOME_BG_BODY halaman utama -- disamakan
   * atas permintaan langsung, supaya berpindah halaman tidak terasa seperti
   * berpindah aplikasi. Sebelumnya di sini gradien navy tipis (C_BG_ATAS ->
   * C_BG_BAWAH) yang diambil dari wajah_jam_modular_240x280.png; makronya
   * sudah tidak dipakai di mana pun, sengaja tidak dihapus dari palet supaya
   * gampang dikembalikan kalau dua halaman ini nanti perlu dibedakan lagi. */
  lv_obj_set_style_bg_color(scr_wajah, lv_color_hex(C_HOME_BG_BODY), 0);
  lv_obj_set_style_bg_opa(scr_wajah, LV_OPA_COVER, 0);

  /* --- baris tanggal / status ---
   * x=82, bukan 86, untuk memberi ruang tambahan ke kelompok baterai di kanan --
   * cincin berakhir di x=79 jadi 82 masih aman.
   *
   * letter_space 0, berbeda dari versi sebelumnya. Jarak 1 px dulu dipakai
   * supaya "SAB 15 AGU" -- sepuluh huruf kapital rapat -- tidak terbaca sebagai
   * satu blok. Tanggal sekarang "Wed, 15 Aug 26": empat karakter lebih panjang,
   * sudah punya koma sebagai jeda, dan huruf kecilnya sendiri yang membentuk
   * kata. Menahan jarak 1 px di sini berarti 13 piksel tambahan yang mendorong
   * tanggal terpanjang sampai menyentuh ikon baterai di kanan. */
  lbl_status = mk_label(scr_wajah, "--", &lv_font_montserrat_12, C_TANGGAL, 82, 13);
  lv_obj_set_style_text_letter_space(lbl_status, 0, 0);

  /* --- ikon baterai berkotak, tepi kanan dikunci di x=232 ---
   * Seluruhnya statis: badan, tonjolan, dan tiga kotak dibuat sekali di sini,
   * dan yang berubah saat runtime cuma warna serta bendera HIDDEN tiap kotak.
   * Tidak ada lagi hitung-ulang posisi seperti pada versi berangka, karena
   * tidak ada lagi teks yang lebarnya berubah -- "9%" dan "100%" dulu berbeda
   * belasan piksel dan itulah yang memaksa penataan ulang setiap kali.
   *
   * Badan digambar sebagai objek bergaris tanpa isi, bukan lewat mk_box():
   * mk_box() selalu memasang latar rapat, sedangkan yang dibutuhkan di sini
   * justru rongga tembus pandang supaya kotak-kotak di dalamnya -- yang dibuat
   * SESUDAH badan, jadi tergambar di atasnya -- benar-benar terlihat. */
  batt_buat(scr_wajah, &batt_wajah, BATT_KANAN_WAJAH, BATT_Y_WAJAH);

  /* Jam besar (dulu di sini, font_digits_48 rata kanan ke x=153 + titik dua +
   * "35" dari x=172) DIHAPUS atas permintaan -- waktu sudah tampil di halaman
   * utama, dan halaman ini murni dasbor metrik. Ruangnya sengaja dibiarkan
   * kosong, bukan diisi widget lain, karena itu bukan bagian dari permintaan
   * ini. */

  /* --- empat kotak kartu, posisi & ukuran asli desain, dengan border ---
   * mk_isi_kartu() di bawah cuma menempel ISI-nya (ikon/judul/nilai/satuan);
   * kotaknya sendiri digambar di sini, sekali per kuadran, persis seperti
   * mk_kartu() versi asli (border 1px C_KARTU_BRD, sudut 14px). */
  for (int qy = 0; qy < 2; qy++) {
    for (int qx = 0; qx < 2; qx++) {
      lv_obj_t *k = mk_box(scr_wajah, qx ? KARTU_X2 : KARTU_X1,
                           qy ? KARTU_Y2 : KARTU_Y1, KARTU_W, KARTU_H,
                           C_KARTU, 14);
      lv_obj_set_style_border_width(k, 1, 0);
      lv_obj_set_style_border_color(k, lv_color_hex(C_KARTU_BRD), 0);
      lv_obj_set_style_border_opa(k, LV_OPA_COVER, 0);
    }
  }

  /* --- lencana bulat: fondasi cincin di atas kartu ---
   * Kartu sekarang mengisi penuh kiri-kanan cincin (lihat definisi KARTU_H),
   * jadi cincin SELALU menimpa keempat kotak kartu di titik pertemuannya --
   * itu disengaja, bukan cacat, tapi arc telanjang di atas kartu+border begitu
   * saja terlihat seperti tabrakan, bukan elemen yang sengaja diletakkan di
   * sana. Piringan solid ini memberinya fondasi yang jelas, persis seperti
   * lencana pada rujukan yang diberikan (WhatsApp Image 2026-09-02).
   *
   * Digambar SEBELUM ketiga arc supaya arc (dan isi kartu) tergambar DI ATAS
   * piringan ini, dan SESUDAH keempat kotak kartu supaya piringan ini yang
   * menimpa kartu -- bukan sebaliknya. Radius 999 sengaja jauh melebihi
   * setengah sisi; LVGL memotongnya sendiri jadi lingkaran penuh. */
  {
    int r_lencana = 34 + CINCIN_W / 2;
    mk_box(scr_wajah, CINCIN_CX - r_lencana, CINCIN_CY - r_lencana,
          2 * r_lencana, 2 * r_lencana, C_PUTIH, 999);
  }

  /* --- tiga cincin, dari luar ke dalam --- */
  cincin_hr = mk_cincin(scr_wajah, 34, C_CINCIN_HR);
  cincin_sp = mk_cincin(scr_wajah, 25, C_CINCIN_SP);
  cincin_gl = mk_cincin(scr_wajah, 16, C_CINCIN_GL);

  /* --- empat kuadran metrik, isi saja -- kotaknya sudah digambar di atas --- */
  mk_isi_kartu(scr_wajah, KARTU_X1, KARTU_Y1, &ic_detak,   "DETAK",   "BPM",
              &lv_font_montserrat_30, &lbl_hr, &sat_hr);
  mk_isi_kartu(scr_wajah, KARTU_X2, KARTU_Y1, &ic_spo2,    "SpO2",    "%",
              &lv_font_montserrat_30, &lbl_sp, &sat_sp);
  mk_isi_kartu(scr_wajah, KARTU_X1, KARTU_Y2, &ic_glukosa, "GLUKOSA", "mg/dL",
              &lv_font_montserrat_30, &lbl_gl, &sat_gl);
  mk_isi_kartu(scr_wajah, KARTU_X2, KARTU_Y2, &ic_tekanan, "TEKANAN", NULL,
              &lv_font_montserrat_26, &lbl_bp, NULL);

  /* Paling akhir supaya pita tepi berada DI ATAS segalanya: cahayanya memang
   * dimaksudkan jatuh menimpa apa pun yang kebetulan ada di dekat tepi. */
  build_tepi(scr_wajah);
}

/* ================= Halaman utama (home) =================
 * Dari 1.png/1baru.png -- "kartu ucapan" yang tampil begitu jam dinyalakan
 * dan setiap kali tidak ada apa pun yang perlu diperhatikan pengguna. Jam
 * tidak punya layar sentuh, jadi satu-satunya jalan berpindah ke halaman
 * kedua (wajah metrik) adalah aplikasi memicu sesuatu yang mengubah keadaan
 * jam -- ARM_SESI/ARM_TITIK/UKUR/MULAI_SESI -- dan tombol fisik yang menyusul
 * di belakangnya. Lihat halaman_evaluasi() untuk aturannya.
 *
 * img_weather/img_diamond sudah lama ada di ui_assets.h (sisa desain 7-layar
 * lama) tapi baru dipakai di sini. img_plane (ilustrasi pesawat) ada di berkas
 * yang sama tapi SENGAJA tidak dipanggil lagi -- lihat catatan di dekat
 * HOME_SEP_Y. Warna latar (C_HOME_BG_HDR/C_HOME_BG_BODY) HARUS sama persis
 * dengan yang dipanggang ke bitmap img_weather/img_diamond di 1baru.png,
 * kalau tidak kotak potongannya akan terlihat -- sama seperti syarat SPL_BG
 * untuk splash_assets.h. */
static void build_home(void) {
  scr_home = lv_obj_create(NULL);
  lv_obj_remove_style_all(scr_home);
  lv_obj_clear_flag(scr_home, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(scr_home, lv_color_hex(C_HOME_BG_BODY), 0);
  lv_obj_set_style_bg_opa(scr_home, LV_OPA_COVER, 0);

  /* --- bilah header: cuaca (kiri), pemisah tegak, baterai (kanan) --- */
  mk_box(scr_home, 0, 0, SCREEN_W, HOME_HDR_H, C_HOME_BG_HDR, 0);
  mk_box(scr_home, 0, HOME_HDR_H - 1, SCREEN_W, 1, C_HOME_DIVIDER, 0);

  mk_img(scr_home, &img_weather, 12, 5);
  lbl_home_suhu = mk_label(scr_home, "--" TXT_DEG "C", &lv_font_montserrat_14,
                           C_PUTIH, 46, 7);
  lbl_home_kondisi = mk_label(scr_home, "CUACA " TXT_DOT " --",
                              &lv_font_montserrat_10, C_HOME_MUTED, 46, 21);
  lv_obj_set_style_text_letter_space(lbl_home_kondisi, 1, 0);

  mk_box(scr_home, HOME_VSEP_X, 8, 1, 16, C_HOME_VSEP, 0);

  batt_buat(scr_home, &batt_home, BATT_KANAN_HOME, BATT_Y_HOME);

  /* --- pemisah "garis - wajik - garis" --- */
  mk_box(scr_home, HOME_VSEP_X - 5 - HOME_SEP_GAP - HOME_SEP_LINE_W, HOME_SEP_Y,
        HOME_SEP_LINE_W, 1, C_HOME_GOLD, 0);
  mk_img(scr_home, &img_diamond, HOME_VSEP_X - 5, HOME_SEP_Y - 5);
  mk_box(scr_home, HOME_VSEP_X + 5 + HOME_SEP_GAP, HOME_SEP_Y,
        HOME_SEP_LINE_W, 1, C_HOME_GOLD, 0);

  /* --- nama hari --- */
  lbl_home_hari = mk_label(scr_home, "--", &lv_font_montserrat_12, C_HOME_MUTED,
                           0, HOME_HARI_Y);
  lv_obj_set_width(lbl_home_hari, SCREEN_W);
  lv_obj_set_style_text_align(lbl_home_hari, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_letter_space(lbl_home_hari, 2, 0);

  /* --- jam besar, dipusatkan di x=120 (bukan rata kanan ke kartu seperti
   * halaman kedua) -- font_jam_home, bukan font_digits_48; lihat definisi
   * HOME_JAM_Y dkk. untuk pengukurannya. Lebar kotak HOME_JAM_W dipatok
   * (bukan cuma HOME_TITIK_DUA_X - HOME_JAM_GAP seperti dulu) supaya rata-
   * kanan/kirinya tidak diam-diam berubah kalau HOME_JAM_GAP disetel ulang. */
  lbl_home_hh = mk_label(scr_home, "--", &font_jam_home, C_PUTIH, 0, HOME_JAM_Y);
  lv_obj_set_width(lbl_home_hh, HOME_JAM_W);
  lv_obj_set_style_text_align(lbl_home_hh, LV_TEXT_ALIGN_RIGHT, 0);

  /* Offset HOME_JAM_DOT1_DY/DOT2_DY dihitung dari metrik font_jam_home
   * sendiri (lihat definisinya) -- BUKAN lagi disamakan dengan titik dua
   * halaman kedua, karena font dan ukurannya sudah tidak sama lagi. */
  mk_box(scr_home, HOME_TITIK_DUA_X, HOME_JAM_Y + HOME_JAM_DOT1_DY,
        HOME_TITIK_DUA_W, HOME_TITIK_DUA_W, C_HOME_GOLD, 2);
  mk_box(scr_home, HOME_TITIK_DUA_X, HOME_JAM_Y + HOME_JAM_DOT2_DY,
        HOME_TITIK_DUA_W, HOME_TITIK_DUA_W, C_HOME_GOLD, 2);

  lbl_home_mm = mk_label(scr_home, "--", &font_jam_home, C_PUTIH,
                         HOME_TITIK_DUA_X + HOME_TITIK_DUA_W + HOME_JAM_GAP,
                         HOME_JAM_Y);

  /* --- tanggal --- */
  lbl_home_tanggal = mk_label(scr_home, "--", &lv_font_montserrat_12,
                              C_HOME_MUTED, 0, HOME_TANGGAL_Y);
  lv_obj_set_width(lbl_home_tanggal, SCREEN_W);
  lv_obj_set_style_text_align(lbl_home_tanggal, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_letter_space(lbl_home_tanggal, 2, 0);
}

/* ================= Layar pembuka =================
 *
 * Tampil saat boot dan -- kalau SPL_SAAT_BANGUN -- setiap kali layar dinyalakan
 * lagi. Itulah yang menentukan seluruh anggarannya. Splash yang cuma dilihat
 * sekali sehari boleh megah; yang dilihat belasan kali sehari harus SELESAI
 * sebelum kesabaran habis.
 *
 * Batas itu dulu ditetapkan di bawah satu detik dan seluruh jadwal diturunkan
 * darinya. Sekarang 1,4 detik, dan itu keputusan sadar: pada 850 ms ketiga
 * gerakannya lewat terlalu cepat untuk terbaca sebagai gerakan -- yang tersisa
 * di mata cuma logo yang tiba-tiba sudah ada. Harganya spesifik dan ada dua:
 * boot mundur setengah detik, dan -- ini yang lebih mahal -- setiap kali layar
 * menyala sendiri untuk sebuah pengukuran, SPL_TOTAL_MS berdiri lebih lama di
 * antara pengguna dan isyarat "tempelkan jari sekarang". Kalau isyarat itu
 * ternyata lebih berharga, yang diubah SPL_SAAT_BANGUN menjadi 0, BUKAN jadwal
 * di bawah: splash saat boot tidak punya siapa pun yang sedang menunggu.
 *
 * KENAPA GAMBAR, BUKAN GAMBAR VEKTOR
 * Tanda jam di logo punya gradien badan, garis hati, dan garis EKG yang saling
 * menimpa. Menirunya dengan primitif LVGL berarti belasan objek yang harus
 * digambar ulang setiap frame -- lebih lambat DAN tidak pernah benar-benar mirip.
 * Satu bitmap RGB565 adalah blit tunggal: tercepat sekaligus paling setia.
 * Kata "ASAWatch" ikut jadi gambar karena bobot dan huruf 'W'-nya tidak ada di
 * Montserrat bawaan LVGL. Anak judulnya justru KEBALIKANNYA: huruf kapital tipis
 * berjarak lebar yang di 8 px tinggi (satu-satunya ukuran yang muat) akan jadi
 * bubur kalau dijadikan bitmap, sedangkan font hinted tetap tajam -- jadi ia
 * satu-satunya yang dirender sebagai teks.
 *
 * KENAPA TIDAK ADA ZOOM ATAU PUTAR
 * lv_img_set_zoom menskalakan ulang seluruh piksel di CPU setiap frame. Pada
 * C6 tanpa PSRAM itu justru sumber patah-patah yang paling dihindari di sini.
 * Yang dipakai cuma tiga gerakan yang MURAH: sapuan busur (cuma cincin tipisnya
 * yang digambar ulang), pudar (blit yang sama, sekali campur), dan geser tegak
 * sejauh 8 px. Ketiganya tidak menyentuh penskalaan sama sekali.
 *
 * URUTANNYA PUNYA ARTI
 * Busur menyapu lebih dulu dan sendirian -- ia menggambar lingkaran bezel jam,
 * jadi tanda jamnya seolah muncul DI DALAM sesuatu yang baru saja terbentuk.
 * Kata dan anak judul menyusul dari bawah setelah tandanya utuh. Kalau ketiganya
 * memudar bersamaan, tidak ada yang menuntun mata dan hasilnya cuma kedipan. */

/* Harus sama persis dengan BG di gensplash.py: latar sudah dipanggang ke dalam
 * kedua bitmap, jadi warna yang berbeda akan memperlihatkan persegi gambarnya
 * sebagai kotak yang lebih terang. */
#define SPL_BG      0x090A10
#define SPL_CX      120
#define SPL_CY      109
#define SPL_R       72          /* jari-jari busur, menyisakan 14 px dari tali jam */
#define SPL_ARC_W   3
#define SPL_C_ARC   0x35C8A0
#define SPL_C_SUB   0x7FBFA8
#define SPL_KATA_Y  197
#define SPL_KATA_DY 8           /* tinggi geseran masuk kata "ASAWatch" */
#define SPL_SUB_Y   228

/* Jadwal, dalam milidetik sejak splash_mulai(). Tumpang tindihnya disengaja:
 * setiap unsur mulai sebelum unsur sebelumnya betul-betul selesai, sehingga
 * tidak pernah ada saat layar diam menunggu giliran berikutnya.
 *
 * Ketujuh angka pertama tadinya jadwal 850 ms lama DIKALI ~1,6, dan sekarang
 * DIKALI ~1,3 LAGI di atas itu (atas permintaan langsung: animasinya diminta
 * sedikit lebih lama supaya terasa lebih halus) -- dikali, bukan ditulis
 * ulang, supaya tumpang tindih yang sudah ditimbang itu tetap utuh dan yang
 * berubah cuma kecepatannya. Busur menyapu lebih tenang, tanda jam memudar
 * cukup lama untuk benar-benar terlihat memudar, dan kata "ASAWatch"
 * menempuh 8 px-nya sebagai gerakan alih-alih sebagai lompatan.
 *
 * Jeda bacanya (180 ms) TIDAK ikut dikalikan lagi -- ia menjawab pertanyaan
 * yang berbeda dari keenam lainnya: bukan seberapa cepat sebuah unsur masuk,
 * melainkan berapa lama logo yang SUDAH utuh boleh berdiri diam sebelum layar
 * berganti, dan itu tidak berubah hanya karena animasi masuknya lebih halus. */
#define SPL_ARC_MS     880
#define SPL_MARK_TUNDA 270
#define SPL_MARK_MS    550
#define SPL_KATA_TUNDA 790
#define SPL_KATA_MS    520
#define SPL_SUB_TUNDA  1130
#define SPL_SUB_MS     450
#define SPL_TOTAL_MS   1760     /* 1580 ms animasi + 180 ms jeda baca */

/* LV_DISP_DEF_REFR_PERIOD 30 ms berarti 33 frame/detik, dan pudaran 260 ms
 * hanya kebagian 9 langkah opasitas -- terlihat sebagai tangga, bukan pudaran.
 * Periodenya dipercepat SELAMA splash saja lalu dikembalikan. Ini target, bukan
 * tenggat: kalau satu frame ternyata butuh lebih dari 16 ms, LVGL sekadar
 * menggambar lebih jarang -- sama dengan yang akan terjadi tanpa baris ini.
 * Wajah jam tidak ikut dipercepat karena ia cuma berubah dua kali per detik.
 *
 * DUA timer, bukan satu, dan ini terukur: mempercepat timer gambar saja tidak
 * mengubah apa pun sama sekali -- frame tetap datang tiap 30 ms. Timer ANIMASI
 * LVGL punya periodenya sendiri yang juga LV_DISP_DEF_REFR_PERIOD, dan selama
 * ia belum jalan tidak ada nilai yang berubah, jadi tidak ada apa pun untuk
 * digambar ulang. Timer gambar yang lebih cepat hanya menemukan layar bersih. */
#define SPL_REFR_MS    16

static void spl_periode(uint32_t ms) {
  lv_timer_set_period(lv_disp_get_default()->refr_timer, ms);
  lv_timer_set_period(lv_anim_get_timer(), ms);
}

static lv_obj_t *scr_splash;
static lv_obj_t *spl_busur, *spl_o_mark, *spl_o_kata, *spl_o_sub;
static lv_timer_t *spl_timer = NULL;
static bool     s_splash_tampil = false;
static uint32_t spl_frame = 0, spl_mulai_ms = 0, spl_render_maks = 0;

/* Pengukur kehalusan. Dipasang permanen di disp_drv dan hanya mencacah selama
 * splash: satu percabangan per frame, dan sebagai gantinya pertanyaan "apakah
 * animasinya patah-patah" bisa dijawab dari serial, bukan dari kesan mata. */
static void spl_monitor(lv_disp_drv_t *drv, uint32_t waktu, uint32_t px) {
  LV_UNUSED(drv); LV_UNUSED(px);
  if (!s_splash_tampil) return;
  spl_frame++;
  if (waktu > spl_render_maks) spl_render_maks = waktu;
}

static void spl_exec_busur(void *o, int32_t v) { lv_arc_set_value((lv_obj_t *)o, v); }
static void spl_exec_gbr(void *o, int32_t v)   { lv_obj_set_style_img_opa((lv_obj_t *)o, (lv_opa_t)v, 0); }
static void spl_exec_teks(void *o, int32_t v)  { lv_obj_set_style_text_opa((lv_obj_t *)o, (lv_opa_t)v, 0); }
static void spl_exec_y(void *o, int32_t v)     { lv_obj_set_y((lv_obj_t *)o, v); }

static void spl_anim(lv_obj_t *o, lv_anim_exec_xcb_t cb, int32_t dari, int32_t ke,
                     uint32_t tunda, uint32_t lama, lv_anim_path_cb_t jalur) {
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, o);
  lv_anim_set_exec_cb(&a, cb);
  lv_anim_set_values(&a, dari, ke);
  lv_anim_set_delay(&a, tunda);
  lv_anim_set_time(&a, lama);
  lv_anim_set_path_cb(&a, jalur);
  lv_anim_start(&a);
}

static void build_splash(void) {
  scr_splash = lv_obj_create(NULL);
  lv_obj_remove_style_all(scr_splash);
  lv_obj_clear_flag(scr_splash, LV_OBJ_FLAG_SCROLLABLE);
  /* Rata, bukan bergradien seperti wajah jam. Bukan selera: latar kedua bitmap
   * sudah dipanggang dengan SATU warna, dan gradien akan membuat persegi
   * gambarnya terlihat di mana pun warnanya tidak lagi berimpit. */
  lv_obj_set_style_bg_color(scr_splash, lv_color_hex(SPL_BG), 0);
  lv_obj_set_style_bg_opa(scr_splash, LV_OPA_COVER, 0);

  /* Busur dibangun seperti mk_cincin (knob dilepas, klik dimatikan) tetapi tanpa
   * jalur latar: yang belum tersapu harus benar-benar kosong, karena jejak
   * gelap sepanjang lingkaran akan membocorkan ke mana sapuannya menuju. */
  int d = SPL_R * 2 + 1;
  spl_busur = lv_arc_create(scr_splash);
  lv_obj_remove_style(spl_busur, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(spl_busur, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(spl_busur, d, d);
  lv_obj_set_pos(spl_busur, SPL_CX - SPL_R, SPL_CY - SPL_R);
  lv_arc_set_rotation(spl_busur, 270);        /* mulai di jam 12 */
  lv_arc_set_bg_angles(spl_busur, 0, 360);
  lv_arc_set_range(spl_busur, 0, 1000);
  lv_arc_set_value(spl_busur, 0);
  lv_obj_set_style_arc_opa(spl_busur, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_arc_width(spl_busur, SPL_ARC_W, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(spl_busur, lv_color_hex(SPL_C_ARC), LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(spl_busur, true, LV_PART_INDICATOR);

  spl_o_mark = mk_img(scr_splash, &spl_mark,
                      SPL_CX - spl_mark.header.w / 2,
                      SPL_CY - spl_mark.header.h / 2);
  spl_o_kata = mk_img(scr_splash, &spl_kata,
                      (SCREEN_W - (int)spl_kata.header.w) / 2, SPL_KATA_Y);

  spl_o_sub = mk_label(scr_splash, "VITAL HEALTH MONITORING",
                       &lv_font_montserrat_12, SPL_C_SUB, 0, 0);
  /* Jarak huruf 1 px meniru tracking lebar pada logo. Lebih dari itu membuat
   * barisnya melewati 240 px dan terpotong di kedua tepi. */
  lv_obj_set_style_text_letter_space(spl_o_sub, 1, 0);
  lv_obj_align(spl_o_sub, LV_ALIGN_TOP_MID, 0, SPL_SUB_Y);
}

static void splash_tutup(bool tuntas) {
  lv_anim_del(spl_busur,  NULL);
  lv_anim_del(spl_o_mark, NULL);
  lv_anim_del(spl_o_kata, NULL);
  lv_anim_del(spl_o_sub,  NULL);

  /* Selalu ke halaman utama dulu, BUKAN scr_wajah seperti versi sebelum ada
   * halaman ini. Bukan pilihan sewenang-wenang: dokumen 13.4 mengunci urutan
   * init "... -> splash -> jam_mulai() -> net_begin()", dan splash ini
   * berakhir SEBELUM jam_mulai() pernah dipanggil -- jam_status()/
   * jam_titik_armed() belum berarti apa-apa di titik ini (nilainya kebetulan
   * "IDLE, tidak ada titik" dari inisialisasi statis, bukan dari NVS yang
   * sungguhan dimuat). Kalau ARM_TITIK selamat lintas boot, setup() akan
   * meluruskan ke halaman kedua sendiri tepat setelah jam_mulai() kembali --
   * lihat baris itu untuk alasan kenapa koreksinya di sana, bukan di sini. */
  s_di_wajah = false;
  lv_scr_load(scr_home);

  uint32_t lama = millis() - spl_mulai_ms;
  s_splash_tampil = false;
  if (!tuntas) {
    Serial.printf("[splash] dibatalkan di %lu ms\n", (unsigned long)lama);
    return;
  }
  Serial.printf("[splash] %lu frame / %lu ms = %lu fps, render terlama %lu ms\n",
                (unsigned long)spl_frame, (unsigned long)lama,
                (unsigned long)(lama ? spl_frame * 1000UL / lama : 0),
                (unsigned long)spl_render_maks);
}

static void splash_selesai_cb(lv_timer_t *t) {
  LV_UNUSED(t);
  /* repeat_count 1: LVGL menghapus timernya sendiri tepat setelah callback ini
   * kembali, jadi yang dibersihkan di sini cuma pointernya. Melepasnya dengan
   * lv_timer_del() di sini berarti LVGL menghapus blok yang sudah bebas. */
  spl_timer = NULL;
  splash_tutup(true);
}

static void splash_mulai(void) {
  if (s_splash_tampil) return;
  s_splash_tampil = true;
  spl_frame       = 0;
  spl_render_maks = 0;
  spl_mulai_ms    = millis();

  /* Keadaan t=0 ditulis eksplisit sebelum animasi mana pun dimulai. LVGL 8.3
   * memang menerapkan nilai awal seketika (early_apply), tetapi splash ini
   * dijalankan berulang kali pada objek yang SAMA -- kalau satu saja anim gagal
   * dimulai, sisa keadaan dari putaran sebelumnya akan tampil sebagai unsur yang
   * sudah utuh sejak frame pertama. */
  lv_arc_set_value(spl_busur, 0);
  lv_obj_set_style_img_opa(spl_o_mark, LV_OPA_TRANSP, 0);
  lv_obj_set_style_img_opa(spl_o_kata, LV_OPA_TRANSP, 0);
  lv_obj_set_style_text_opa(spl_o_sub, LV_OPA_TRANSP, 0);
  lv_obj_set_y(spl_o_kata, SPL_KATA_Y + SPL_KATA_DY);

  lv_scr_load(scr_splash);

  spl_anim(spl_busur,  spl_exec_busur, 0, 1000,
           0, SPL_ARC_MS, lv_anim_path_ease_out);
  spl_anim(spl_o_mark, spl_exec_gbr, LV_OPA_TRANSP, LV_OPA_COVER,
           SPL_MARK_TUNDA, SPL_MARK_MS, lv_anim_path_ease_out);
  /* Dua anim pada objek yang sama tetapi exec_cb berbeda, jadi keduanya hidup:
   * lv_anim_start() hanya menggusur anim dengan pasangan (var, exec_cb) identik. */
  spl_anim(spl_o_kata, spl_exec_gbr, LV_OPA_TRANSP, LV_OPA_COVER,
           SPL_KATA_TUNDA, SPL_KATA_MS, lv_anim_path_linear);
  spl_anim(spl_o_kata, spl_exec_y, SPL_KATA_Y + SPL_KATA_DY, SPL_KATA_Y,
           SPL_KATA_TUNDA, SPL_KATA_MS, lv_anim_path_ease_out);
  spl_anim(spl_o_sub,  spl_exec_teks, LV_OPA_TRANSP, LV_OPA_COVER,
           SPL_SUB_TUNDA, SPL_SUB_MS, lv_anim_path_linear);

  spl_timer = lv_timer_create(splash_selesai_cb, SPL_TOTAL_MS, NULL);
  lv_timer_set_repeat_count(spl_timer, 1);

  spl_periode(SPL_REFR_MS);
}

static void splash_batal(void) {
  if (!s_splash_tampil) return;
  if (spl_timer) { lv_timer_del(spl_timer); spl_timer = NULL; }
  splash_tutup(false);
}

/* Dipakai HANYA saat boot: memutar splash sampai habis dengan CPU penuh, sebelum
 * jam_mulai() dan net_begin() menyalakan radio. Membiarkannya berjalan asinkron
 * di loop() bersama init NimBLE dan Wi-Fi berarti animasinya tersendat persis di
 * satu-satunya kesempatan ia dilihat utuh. Saat layar bangun kembali splash TIDAK
 * boleh diputar begini -- FIFO MAX30105 penuh dalam ratusan milidetik, dan
 * menghentikan loop() selama itu akan membuang sampel PPG.
 *
 * Jaring pengaman waktunya bukan basa-basi: kalau timer penutup entah bagaimana
 * tidak pernah jalan, tanpa batas ini jam berhenti di logo selamanya. */
static void splash_tunggu(void) {
  uint32_t batas = millis() + SPL_TOTAL_MS + 500;
  while (s_splash_tampil && (int32_t)(millis() - batas) < 0) {
    lv_timer_handler();
    delay(1);
  }
  if (s_splash_tampil) {
    Serial.println("[splash] BUG: timer penutup tidak pernah jalan -- dipaksa tutup");
    splash_batal();
  }
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

/* Satu kartu metrik.
 *
 * `sementara` menandai angka yang sudah nyata tetapi belum lolos gerbang kirim
 * (ppg.h, field `awal`). Ia ditampilkan lebih redup, bukan disembunyikan dan
 * bukan pula ditulis sama seperti hasil akhir. Menyembunyikannya berarti layar
 * kosong justru saat sensor paling sibuk; menulisnya sama persis berarti angka
 * yang masih bergoyang beberapa mg/dL tidak bisa dibedakan dari angka yang
 * akhirnya tercatat di aplikasi. Redup menjawab keduanya sekaligus, dan tidak
 * memakan satu piksel pun ruang tambahan -- yang di layar ini memang tidak ada. */
static void nilai_set(lv_obj_t *lbl, lv_obj_t *satuan, const lv_font_t *fn,
                      bool sah, bool sementara, const char *fmt, float a, float b) {
  char buf[16];
  if (!sah && !sementara)   snprintf(buf, sizeof(buf), "--");
  else if (b < 0.0f)        snprintf(buf, sizeof(buf), fmt, a);
  else                      snprintf(buf, sizeof(buf), fmt, a, b);
  if (set_jika_beda(lbl, buf)) satuan_sejajar(satuan, lbl, fn);

  uint32_t warna = sementara ? C_TANGGAL : C_PUTIH;
  if (lv_obj_get_style_text_color(lbl, 0).full != lv_color_hex(warna).full)
    lv_obj_set_style_text_color(lbl, lv_color_hex(warna), 0);
}

/* ---- Ikon baterai: tiga kotak, tanpa angka persen ----
 *
 * Angka persen sengaja DIHILANGKAN, bukan sekadar disembunyikan karena kurang
 * muat. Persen dari tegangan Li-Po hanya bisa dipercaya sampai sekitar
 * +-5..10% (alasannya panjang, ada di battery.h dan battery.cpp): kurvanya
 * datar sekali di tengah, hambatan dalam sel berubah menurut suhu dan umur,
 * dan beban board sendiri melompat ratusan mA saat radio atau LED PPG menyala.
 * Menulis "73%" di layar menjanjikan ketelitian satu persen yang alatnya tidak
 * punya. Tiga kotak menjanjikan sepertiga -- dan janji itu bisa ditepati.
 *
 * Ambang naik dan ambang turun DIPISAH. Tanpa itu, persen yang menggantung
 * tepat di satu ambang akan membuat kotak terakhir berkedip tiap kali nilainya
 * bergeser satu digit -- gejala yang paling merusak kepercayaan, karena
 * kedipannya terlihat seperti baterai yang bermasalah padahal itu cuma derau
 * ADC. Jaraknya 6%, sedikit di atas riak yang tersisa setelah median + EMA +
 * minimum jendela 3 menit di battery.cpp.
 *
 * Ambangnya juga tidak rata jaraknya. Yang benar-benar perlu dibedakan adalah
 * ujung bawah -- "masih bisa dipakai" versus "cari charger sekarang" -- bukan
 * ujung atas, di mana beda 90% dan 100% tidak mengubah apa pun yang dilakukan
 * pemakai. */
static const int BATT_TURUN[BATT_N_KOTAK] = { 12, 42, 67 };  /* kotak ke-n padam di bawah ini */
static const int BATT_NAIK [BATT_N_KOTAK] = { 18, 48, 73 };  /* kotak ke-n menyala di atas ini */

static int batt_hitung_kotak(int persen, int lalu) {
  /* Tampilan pertama belum punya riwayat, jadi histeresis tidak bisa dipakai:
   * memulai dari 0 kotak akan membuat baterai penuh tampil sebagai 2 kotak
   * sampai persen menyentuh 73%. Titik tengah kedua ambang adalah tebakan
   * netral yang tidak condong ke atas maupun ke bawah. */
  if (lalu < 0) {
    int n = 0;
    while (n < BATT_N_KOTAK && persen >= (BATT_TURUN[n] + BATT_NAIK[n]) / 2) n++;
    return n;
  }
  int n = lalu;
  while (n < BATT_N_KOTAK && persen >= BATT_NAIK[n]) n++;
  while (n > 0            && persen <  BATT_TURUN[n - 1]) n--;
  return n;
}

/* Warna memikul satu-satunya peringatan yang tersisa setelah angka dibuang.
 * Nol kotak berarti badan baterai kosong melompong -- tanpa warna merah, layar
 * yang kosong itu tidak bisa dibedakan dari ikon yang belum sempat digambar. */
static void batt_gambar(batt_widget_t *w, int kotak, bool mengisi) {
  uint32_t warna = mengisi ? C_ISI : (kotak == 0 ? C_CINCIN_HR : C_REDUP);

  lv_obj_set_style_border_color(w->cangkang, lv_color_hex(warna), 0);
  lv_obj_set_style_bg_color(w->nub, lv_color_hex(warna), 0);

  for (int i = 0; i < BATT_N_KOTAK; i++) {
    if (i < kotak) {
      lv_obj_set_style_bg_color(w->kotak[i], lv_color_hex(warna), 0);
      lv_obj_clear_flag(w->kotak[i], LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(w->kotak[i], LV_OBJ_FLAG_HIDDEN);
    }
  }

  /* Petir muncul di kiri badan, bukan menggantikan isinya. Versi sebelumnya
   * harus menukar lambang karena baris ini cuma menyisakan ~50 px di kanan
   * tanggal; sekarang angka persen sudah pergi dan ruangnya cukup, jadi
   * "sedang mengisi" bisa ditandai dua kali -- warna DAN lambang -- tanpa
   * mengorbankan jumlah kotak yang terbaca. */
  if (mengisi) lv_obj_clear_flag(w->petir, LV_OBJ_FLAG_HIDDEN);
  else         lv_obj_add_flag(w->petir, LV_OBJ_FLAG_HIDDEN);
}

/* Bangun satu ikon baterai berkotak di `parent` (tepi kanan tonjolan di
 * `kanan`, tepi atas badan di `y`) dan simpan handle-nya di `w`. Dipanggil
 * sekali per layar -- lihat kenapa di komentar batt_widget_t. */
static void batt_buat(lv_obj_t *parent, batt_widget_t *w, int kanan, int y) {
  int bx = kanan - BATT_NUB_W - BATT_W;

  /* Badan digambar sebagai objek bergaris tanpa isi, bukan lewat mk_box():
   * mk_box() selalu memasang latar rapat, sedangkan yang dibutuhkan di sini
   * justru rongga tembus pandang supaya kotak-kotak di dalamnya -- yang dibuat
   * SESUDAH badan, jadi tergambar di atasnya -- benar-benar terlihat. */
  w->cangkang = lv_obj_create(parent);
  lv_obj_remove_style_all(w->cangkang);
  lv_obj_set_pos(w->cangkang, bx, y);
  lv_obj_set_size(w->cangkang, BATT_W, BATT_H);
  lv_obj_set_style_bg_opa(w->cangkang, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(w->cangkang, BATT_BRD, 0);
  lv_obj_set_style_border_color(w->cangkang, lv_color_hex(C_REDUP), 0);
  lv_obj_set_style_border_opa(w->cangkang, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(w->cangkang, 3, 0);
  lv_obj_clear_flag(w->cangkang, LV_OBJ_FLAG_SCROLLABLE);

  w->nub = mk_box(parent, bx + BATT_W, y + (BATT_H - BATT_NUB_H) / 2,
                  BATT_NUB_W, BATT_NUB_H, C_REDUP, 1);

  for (int i = 0; i < BATT_N_KOTAK; i++)
    w->kotak[i] = mk_box(parent, bx + BATT_BRD + BATT_PAD + i * BATT_KOTAK_D,
                         y + BATT_BRD + BATT_PAD, BATT_KOTAK_W, BATT_KOTAK_H,
                         C_REDUP, 1);

  w->petir = mk_label(parent, LV_SYMBOL_CHARGE, &lv_font_montserrat_12, C_ISI, 0, 0);
  lv_obj_update_layout(parent);
  lv_obj_align_to(w->petir, w->cangkang, LV_ALIGN_OUT_LEFT_MID, -3, 0);

  /* Mulai dari keadaan "belum ada bacaan": badan kosong, warna peringatan.
   * Bukan tiga kotak penuh -- ikon penuh sebelum ADC sempat mengukur adalah
   * angka karangan dalam bentuk gambar. */
  batt_gambar(w, 0, false);
}

/* ---- Baris tanggal yang merangkap baris status ----
 * Ini satu-satunya area teks bebas di wajah ini, jadi ia memikul tiga hal
 * sekaligus. Urutan prioritasnya penting dan disengaja:
 *
 *   1. Pesan sesaat (2 detik) -- jawaban atas tombol yang baru saja ditekan.
 *      Paling atas karena ini satu-satunya umpan balik yang dimiliki tombol
 *      BOOT; tanpa itu menekan tombol terasa seperti tidak terjadi apa-apa.
 *   2. Kemajuan pengukuran -- pengguna harus diam selama belasan detik, dan
 *      dokumen 14 mewajibkan ia tahu bahwa sesuatu sedang berjalan serta
 *      seberapa jauh. Yang ditampilkan persen, angka yang sama dengan yang
 *      dikirim ke aplikasi di byte 8 paket Status; alasan memilihnya di atas
 *      sisa detik ada di status_baris().
 *   3. Panggilan pengukuran terjadwal yang menunggu tombol.
 *   4. Tanggal -- keadaan tenang.
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
    /* Teks "UKUR n%" DIHAPUS dari baris ini atas permintaan -- sejak cincin
     * pindah ke tengah layar (di antara keempat kartu), cincin ITU SENDIRI
     * sudah menggambarkan kemajuan yang sama persis (jam_ukur_persen(), lihat
     * cincin_set() di refresh_cb), jadi angka di baris status jadi keterangan
     * ganda. Cabang ini TETAP ADA dan tetap return lebih awal -- bukan sekadar
     * dihapus -- supaya selama mengukur baris ini tidak jatuh ke cabang
     * TEKAN: UKUR n di bawah (yang keliru: tombolnya sedang tidak menunggu
     * ditekan, sensornya sedang bekerja). */
    set_jika_beda(lbl_status, "");
    return;
  }

  /* Arti tombol ditulis DI SINI, di baris yang sama dengan tanggal, dan ia
   * mendahului tanggal saat ada yang menunggu ditekan (dokumen 14).
   *
   * Ini bukan hiasan: jam punya satu tombol dengan dua makna, dan penggunanya
   * lansia. Dua tombol fisik lebih mahal daripada satu tombol yang layarnya
   * menjelaskan diri -- tetapi hanya kalau layarnya benar-benar menjelaskan.
   *
   * Perhatikan tombolnya PADAM begitu titiknya terukur, termasuk saat yang
   * mengukur adalah aplikasi lewat UKUR dan bukan tombolnya. Baris ini ikut
   * padam pada putaran refresh berikutnya karena ia membaca jam_titik_armed()
   * apa adanya. Tombol yang masih menyala untuk titik yang sudah terisi adalah
   * kebohongan yang akan membuat pengguna menekannya lagi. */
  if (jam_titik_armed()) {
    snprintf(buf, sizeof(buf), "TEKAN: UKUR %u", (unsigned)jam_titik_index());
    if (set_jika_beda(lbl_status, buf))
      lv_obj_set_style_text_color(lbl_status, lv_color_hex(C_ISI), 0);
    return;
  }
  /* Petunjuk "TEKAN: SELESAI MAKAN" untuk status ARMED tetap tidak ada (lihat
   * riwayat di atas -- sesi ARMED jatuh ke baris kosong di bawah, sama seperti
   * IDLE). Ini TIDAK mengubah aturan tombol fisik sama sekali: jam_tekan_tombol()
   * masih memvalidasi status sesi sendiri (dokumen 12), cuma layar yang tidak
   * mengumumkannya lebih dulu.
   *
   * Jam/hari/tanggal/bulan DIHAPUS dari baris ini atas permintaan -- halaman
   * kedua murni dasbor metrik, kalender sepenuhnya jadi urusan halaman utama.
   * Baris ini karena itu kosong di luar cabang TEKAN: UKUR n di atas, yang
   * tetap dipertahankan karena itu umpan balik pengukuran, bukan kalender. */
  set_jika_beda(lbl_status, "");
}

/* ================= Halaman utama: isi ================= */
static const char *HARI_ID[7] = {
  "MINGGU", "SENIN", "SELASA", "RABU", "KAMIS", "JUMAT", "SABTU"
};
static const char *BULAN_ID[12] = {
  "JANUARI", "FEBRUARI", "MARET",     "APRIL",   "MEI",      "JUNI",
  "JULI",    "AGUSTUS",  "SEPTEMBER", "OKTOBER", "NOVEMBER", "DESEMBER"
};

/* Disegarkan tiap putaran TERLEPAS dari halaman mana yang sedang aktif -- sama
 * seperti isi halaman kedua di refresh_cb(). Alasannya sama juga: begitu
 * halaman_evaluasi() memutuskan pindah, isinya harus sudah benar di frame
 * pertama, bukan menyusul beberapa ratus milidetik kemudian. */
static void home_refresh(void) {
  struct tm t;
  if (tm_now(&t)) {
    static int menit_lalu = -1;
    if (t.tm_min != menit_lalu) {
      menit_lalu = t.tm_min;
      lv_label_set_text_fmt(lbl_home_hh, "%02d", t.tm_hour);
      lv_label_set_text_fmt(lbl_home_mm, "%02d", t.tm_min);
    }
    set_jika_beda(lbl_home_hari,
                  HARI_ID[(t.tm_wday >= 0 && t.tm_wday < 7) ? t.tm_wday : 0]);

    char buf[24];
    snprintf(buf, sizeof(buf), "%d %s %04d", t.tm_mday,
             BULAN_ID[(t.tm_mon >= 0 && t.tm_mon < 12) ? t.tm_mon : 0],
             t.tm_year + 1900);
    set_jika_beda(lbl_home_tanggal, buf);
  }

  /* Cuaca MATI secara bawaan (AW_PAKAI_WIFI 0, config.h): weather_get()
   * menjawab valid=false apa adanya, dan yang tampil "--" -- bukan angka
   * karangan -- persis filosofi yang sama dengan seluruh layar kesehatan. */
  weather_t w;
  weather_get(&w);

  char suhu[12];
  if (w.valid) snprintf(suhu, sizeof(suhu), "%d" TXT_DEG "C", w.temp_c);
  else         snprintf(suhu, sizeof(suhu), "--" TXT_DEG "C");
  set_jika_beda(lbl_home_suhu, suhu);

  char kondisi[24];
  snprintf(kondisi, sizeof(kondisi), "CUACA " TXT_DOT " %s", w.valid ? w.cond : "--");
  set_jika_beda(lbl_home_kondisi, kondisi);
}

/* ================= Halaman utama <-> halaman kedua =================
 * Jam ini tidak berlayar sentuh (touch_poll() dipertahankan hanya demi board
 * yang sentuhannya sehat, lihat catatan di kepala berkas), jadi pengguna tidak
 * bisa mengetuk apa pun untuk berpindah halaman. Perpindahannya karena itu
 * murni REAKSI atas keadaan sesi yang memang sudah datang dari aplikasi:
 * ARM_SESI/ARM_TITIK/UKUR/MULAI_SESI masing-masing mengubah salah satu dari
 * tiga hal yang diperiksa di bawah. TIDAK ADA opcode baru untuk ini, dan tidak
 * boleh ada -- dokumen protokol tidak menyebut halaman sama sekali, navigasi
 * adalah urusan tampilan lokal, bukan kontrak kawat (dokumen 1).
 *
 * Ke halaman kedua: SEKETIKA, begitu ada sesuatu yang perlu dilihat pengguna --
 * sesi ARMED/RUNNING, titik ukur menyala, atau sedang mengukur (termasuk cek
 * manual dari tombol fisik, lihat boot_aktifkan() yang memaksa halaman ini
 * juga supaya umpan balik penolakan tetap terlihat).
 * Ke halaman utama: hanya setelah HALAMAN_PULANG_MS tenang PENUH, supaya hasil
 * terakhir sempat terbaca dulu -- persis alasan yang sama dengan tenggat auto-
 * mati layar di layar_nyala_sementara(). */
#define HALAMAN_PULANG_MS 5000UL

static void halaman_set(bool wajah) {
  if (wajah == s_di_wajah) return;
  s_di_wajah = wajah;
  s_pulang_home_pada = 0;
  lv_scr_load(wajah ? scr_wajah : scr_home);
}

static void halaman_evaluasi(void) {
  bool perlu_wajah = jam_status() != AW_SESI_IDLE || jam_titik_armed() ||
                     jam_sedang_mengukur();

  if (perlu_wajah) {
    halaman_set(true);
    return;
  }
  if (!s_di_wajah) return;      /* sudah di home, tidak ada yang perlu ditunda */

  if (!s_pulang_home_pada) s_pulang_home_pada = millis() + HALAMAN_PULANG_MS;
  else if ((int32_t)(millis() - s_pulang_home_pada) >= 0) halaman_set(false);
}

static void refresh_cb(lv_timer_t *tm) {
  (void)tm;
  /* Konteks loop: di sinilah RTC dibaca dan hasil NTP diterapkan. */
  tm_tick();

  /* struct tm/have_time TIDAK dibuang walau jam besar sudah dihapus dari
   * halaman ini (lihat build_wajah()): heartbeat serial di bawah masih
   * memakai keduanya. */
  struct tm t;
  bool have_time = tm_now(&t);

  status_baris();
  home_refresh();

  /* ---- pita kemajuan di tepi layar ----
   * Hanya hidup selama pengukuran; di luar itu tepi layar harus bersih supaya
   * wajah jam kembali seperti desainnya. */
  tepi_set(jam_sedang_mengukur() ? (int)jam_ukur_persen() : 0);

  /* ---- halaman utama <-> halaman kedua ----
   * SEBELUM blok bangun-layar di bawah: kalau keduanya terjadi di putaran yang
   * sama (mis. UKUR datang lewat BLE selagi layar mati), lv_obj_invalidate()
   * yang dipanggil layar_set() harus sudah menunjuk ke layar yang BARU dipilih
   * di sini, bukan layar lama yang sebentar lagi diganti. */
  halaman_evaluasi();

  /* ---- pengukuran yang tidak dimulai tombol membangunkan layar ----
   * Alasannya berubah di v1.3 tetapi barisnya tetap perlu, dan justru lebih
   * perlu. Yang membangunkan layar sekarang bukan lagi jadwal jam sendiri --
   * jadwal itu dicabut -- melainkan UKUR dari aplikasi, yang bisa tiba kapan
   * saja termasuk saat layar sedang gelap. Tanpa baris ini, LED menyala pada
   * jam yang layarnya gelap dan pengguna tidak punya cara apa pun mengetahui
   * bahwa SEKARANG saatnya menempelkan jari -- pengukurannya lalu menyerah
   * setelah 90 detik tanpa kontak dan titik itu tetap kosong.
   *
   * Hanya pada TRANSISI mulai-mengukur, bukan selama pengukurannya berjalan:
   * kalau tidak, layar yang baru saja dimatikan pengguna menyala lagi setengah
   * detik kemudian dan tidak bisa dimatikan sampai pengukurannya selesai. Satu
   * kali menyala adalah pemberitahuan; menyala terus adalah tombol yang rusak.
   *
   * Cek manual dikecualikan: ia dimulai oleh tekanan tombol, jadi layarnya sudah
   * pasti menyala dan pemakainya sudah pasti sedang melihat. Hal yang sama
   * berlaku untuk tombol ukur ARM_TITIK, dan itu tidak perlu dikecualikan
   * terpisah -- layar yang sudah menyala membuat layar_set(true) tidak
   * melakukan apa pun. */
  static bool ukur_lalu = false;
  bool ukur_kini = jam_sedang_mengukur();
  if (ukur_kini && !ukur_lalu && !jam_ukur_lokal())
    layar_nyala_sementara(LAYAR_AUTO_MATI_MS);
  ukur_lalu = ukur_kini;

  /* ---- baterai ----
   * Kotak saja tidak jujur saat kabel tertancap. Selama mengisi, tegangan sel
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
    static int kotak_lalu = -1;
    static int isi_lalu   = -1;
    int kotak = batt_hitung_kotak(battery_percent(), kotak_lalu);
    int chg   = battery_charging() ? 1 : 0;
    if (kotak != kotak_lalu || chg != isi_lalu) {
      /* Layar dibangunkan saat MULAI mengisi, sebagai satu-satunya umpan balik
       * bahwa kabelnya benar-benar masuk -- jam ini tidak punya LED charger.
       * isi_lalu == -1 dikecualikan supaya boot dalam keadaan tercolok tidak
       * ikut memicunya; layarnya toh sudah menyala di situ. */
      if (chg && isi_lalu == 0) layar_nyala_sementara(LAYAR_AUTO_MATI_MS);
      kotak_lalu = kotak;
      isi_lalu   = chg;
      /* Kedua salinan disegarkan bersamaan -- lihat komentar batt_widget_t --
       * supaya halaman mana pun yang aktif saat halaman_evaluasi() berikutnya
       * berpindah tidak pernah menampilkan ikon yang basi. */
      batt_gambar(&batt_wajah, kotak, chg != 0);
      batt_gambar(&batt_home, kotak, chg != 0);
    }
  }

  /* Tenggat auto-mati. Pengukuran menundanya alih-alih membatalkannya: pengguna
   * harus diam belasan detik dan kemajuannya cuma ada di layar (dokumen 14),
   * jadi mematikannya di tengah situ akan membuang pengukuran yang sedang
   * berjalan. Kalau ditunda, layarnya tetap mati beberapa detik setelah selesai
   * dan tidak menyala semalaman. */
  if (layar_mati_pada && (int32_t)(millis() - layar_mati_pada) >= 0) {
    if (jam_sedang_mengukur()) layar_mati_pada = millis() + 5000UL;
    else                       layar_set(false);   /* ini mengosongkan tenggatnya */
  }

  /* ---- empat metrik + tiga cincin ----
   * jam_snapshot() dipakai, BUKAN ppg_get(): sensor hanya menyala selama
   * pengukuran, jadi membaca langsung dari sensor berarti keempat kartu
   * menampilkan "--" hampir sepanjang waktu -- benar, tetapi tidak berguna.
   * jam_snapshot() menambalnya dengan hasil pengukuran terakhir. */
  ppg_data_t p;
  jam_snapshot(&p);

  /* p.awal: angka sudah nyata, gerbang kirim belum terlewati. Kartu menampilkan
   * keduanya (yang sementara lebih redup) supaya layar bergerak selama sensor
   * bekerja. Yang dikirim ke aplikasi tidak ikut berubah sedikit pun -- itu
   * diputuskan aw_jam.cpp dari p.*_valid, bukan dari apa yang tampil di sini. */
  bool sementara = p.awal;

  nilai_set(lbl_hr, sat_hr, &lv_font_montserrat_30, p.bpm_valid,  sementara,
            "%.0f", p.bpm, -1.0f);
  nilai_set(lbl_sp, sat_sp, &lv_font_montserrat_30, p.spo2_valid, sementara,
            "%.0f", p.spo2, -1.0f);
  nilai_set(lbl_gl, sat_gl, &lv_font_montserrat_30, p.glu_valid,  sementara,
            "%.0f", p.glucose, -1.0f);
  nilai_set(lbl_bp, NULL,   &lv_font_montserrat_26, p.bp_valid,   sementara,
            "%.0f/%.0f", p.sbp, p.dbp);

  cincin_set(cincin_hr, p.bpm_valid || sementara,  p.bpm,     HR_MIN, HR_MAKS);
  cincin_set(cincin_sp, p.spo2_valid || sementara, p.spo2,    SP_MIN, SP_MAKS);
  cincin_set(cincin_gl, p.glu_valid || sementara,  p.glucose, GL_MIN, GL_MAKS);

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

  static const char *SRC[] = { "none", "rtc", "ntp", "ble" };
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
  Serial.printf("[ppg]  %s%s%s  bpm=%s%.0f  spo2=%s%.1f  glukosa*=%s%.0f  "
                "td*=%s%.0f/%.0f\n",
                ppg_state_text(), p.held ? " [tahan]" : "",
                p.awal ? " [awal]" : "",
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

/* ================= Tombol PWR: tahan 3 detik = on/off =====================
 * Satu gerbang yang sama untuk kedua arah, persis seperti tombol daya HP:
 *
 *   tahan >= 3 dtk saat mati    -> menyala   (lihat pwr_gerbang_nyala())
 *   tahan >= 3 dtk saat menyala -> mati      (latch BAT_EN dilepas)
 *   klik singkat                -> layar mati / hidup. Jam tetap berjalan:
 *                                  sensor, sesi, BLE, dan ring buffer tidak
 *                                  tersentuh sama sekali.
 *
 * Kenapa harus digerbang di KEDUA arah. Tanpa gerbang menyala, jam yang
 * tersenggol di dalam tas akan menyala sendiri -- board ini menyambungkan
 * baterainya secara perangkat keras selama tombol ditekan, jadi senggolan
 * sepersekian detik sudah cukup untuk menjalankan firmware, dan firmware itu
 * yang lalu menahan latch-nya sendiri sampai baterainya habis. Tanpa gerbang
 * mati, satu senggolan yang sama mengakhiri sesi dua jam yang sedang berjalan
 * dan sesi itu tidak bisa dimulai ulang dari jam.
 *
 * Tiga detik dipilih pengguna perangkat ini. Ia juga angka yang wajar: cukup
 * lama untuk tidak pernah terjadi di dalam tas, cukup pendek untuk tidak terasa
 * seperti jam yang menolak perintah.
 *
 * Aksi tekan-lama dijalankan saat ambangnya terlewati, bukan saat jari dilepas,
 * supaya layar yang padam menjadi jawaban atas tekanan itu sendiri -- dengan
 * begitu pengguna tidak menahan sambil menebak apakah sudah cukup lama. */
#define PWR_DEBOUNCE_MS   50
#define PWR_LAMA_MS     3000

static bool     pwr_siap = false;      /* tombol sudah pernah dilepas sejak boot */
static int      pwr_level_lalu  = HIGH;
static int      pwr_stabil_lvl  = HIGH;
static uint32_t pwr_stabil_ms   = 0;
static uint32_t pwr_tekan_ms    = 0;
static bool     pwr_lama_jalan  = false;

/* Gerbang MENYALA, dipanggil dari setup() tepat setelah latch dipasang.
 *
 * Board ini menyambungkan baterainya secara perangkat keras selama PWR ditekan,
 * jadi firmware tidak punya cara menolak dijalankan -- yang bisa dilakukannya
 * cuma memutuskan apakah ia MELANJUTKAN. Di sinilah keputusan itu: selama tombol
 * masih ditahan, tunggu sampai tiga detik penuh; kalau jarinya lepas lebih awal,
 * latch dilepas lagi dan board mati persis seperti sebelum tersenggol.
 *
 * Ambangnya diukur dari millis(), bukan dari saat fungsi ini mulai. millis()
 * menghitung sejak CPU menyala, jadi ~300 ms yang dihabiskan bootloader ikut
 * terhitung -- dan yang dirasakan pengguna memang lama JARINYA menekan, bukan
 * lama firmware berjalan.
 *
 * Tombol yang sudah terlepas saat fungsi ini dijalankan bukan urusannya: itu
 * berarti board menyala karena sebab lain (kabel USB ditancapkan, tombol RST,
 * atau flash baru selesai), dan tidak ada tekanan tombol yang perlu dinilai.
 *
 * Mengembalikan false hanya pada satu keadaan: tombol dilepas terlalu cepat
 * TETAPI board tetap hidup -- artinya ia dicatu USB, dan pelepasan latch tidak
 * berefek apa-apa. Pemanggil memakainya untuk mencetak keterangan itu; boot
 * tetap dilanjutkan, karena kalau tidak, setiap sesi pengembangan lewat USB akan
 * berakhir di board yang diam tanpa penjelasan. */
static bool pwr_gerbang_nyala(void) {
  if (digitalRead(PWR_KEY) != LOW) return true;   /* bukan dinyalakan dari tombol */

  while (millis() < PWR_LAMA_MS) {
    if (digitalRead(PWR_KEY) != LOW) {
      digitalWrite(BAT_EN, LOW);    /* di baterai: board berhenti tepat di sini */
      delay(50);
      digitalWrite(BAT_EN, HIGH);   /* masih hidup -> USB; pasang lagi dan lanjut */
      return false;
    }
    delay(10);
  }
  return true;
}

static void pwr_matikan(void) {
  Serial.println("[pwr] tombol PWR ditahan -- mematikan");
  layar_set(false);
  jam_siap_mati();               /* sensor padam, ring buffer dipaksa ke NVS */
  digitalWrite(BAT_EN, LOW);     /* di baterai: board berhenti tepat di sini */

  /* Sampai di sini berarti masih ada daya dari USB. Versi sebelumnya menjawab
   * keadaan itu dengan for(;;) delay(100) -- board yang layarnya hidup tapi
   * tidak menjawab tombol apa pun, tidak bisa dibedakan dari firmware yang
   * hang, dan hanya bisa dipulihkan lewat tombol RST. Sekarang ia cuma diam
   * dengan layar mati, dan klik berikutnya menghidupkannya lagi. */
  pwr_daya_lepas = true;
  Serial.println("[pwr] latch dilepas. Kalau board masih hidup, ia dicatu USB: "
                 "tahan PWR 3 dtk lagi untuk menyalakan kembali.");
}

static void pwr_hidupkan_lagi(void) {
  digitalWrite(BAT_EN, HIGH);
  pwr_daya_lepas = false;
  layar_nyala_sementara(LAYAR_MATI_TOMBOL_MS);
  Serial.println("[pwr] latch dipasang lagi -- jam menyala kembali");
}

/* Klik singkat. Saat jam sudah diminta mati (dan cuma bertahan karena USB),
 * klik sengaja tidak berbuat apa pun: menyalakan harus lewat gerbang tiga detik
 * yang sama seperti mematikan, kalau tidak "off" jadi keadaan yang bisa
 * dibatalkan sentuhan tak sengaja. */
static void pwr_klik(void) {
  if (pwr_daya_lepas) return;
  /* Klik saat sudah menyala tetap MEMATIKAN, bukan memperpanjang tenggat.
   * Tombol ini satu-satunya cara mematikan layar dengan sengaja, dan menukarnya
   * jadi "perpanjang" akan menghilangkan kemampuan itu sama sekali. Kalau butuh
   * lebih lama: klik mati lalu klik nyala lagi. */
  if (s_layar_nyala) layar_set(false);
  else               layar_nyala_sementara(LAYAR_MATI_TOMBOL_MS);
}

static void pwr_poll(void) {
  int level = digitalRead(PWR_KEY);      /* LOW = sedang ditekan */
  if (level != pwr_level_lalu) {
    pwr_level_lalu = level;
    pwr_stabil_ms  = millis();
  } else if ((uint32_t)(millis() - pwr_stabil_ms) >= PWR_DEBOUNCE_MS &&
             level != pwr_stabil_lvl) {
    pwr_stabil_lvl = level;              /* tepi yang sudah bersih */
    if (level == LOW) {
      pwr_tekan_ms   = millis();
      pwr_lama_jalan = false;
    } else if (!pwr_siap) {
      /* Tombol PWR memang MASIH ditahan saat board menyala -- begitulah cara
       * board ini dinyalakan. Tekanan itu bukan perintah dan tidak dihitung. */
      pwr_siap = true;
      Serial.println("[pwr] tombol dilepas -- klik = layar, tahan 3 dtk = mati");
    } else if (!pwr_lama_jalan) {
      pwr_klik();                        /* dilepas sebelum ambang tekan-lama */
    }
  }

  if (pwr_siap && pwr_stabil_lvl == LOW && !pwr_lama_jalan &&
      (uint32_t)(millis() - pwr_tekan_ms) >= PWR_LAMA_MS) {
    pwr_lama_jalan = true;
    /* Gerbang yang sama, dua arah. Cabang "menyala" hanya terpakai pada board
     * yang dicatu USB: di baterai, pwr_matikan() memang tidak pernah kembali. */
    if (pwr_daya_lepas) pwr_hidupkan_lagi();
    else                pwr_matikan();
  }
}

/* ================= Tombol BOOT: tombol pengukuran =================
 * SATU tombol, SATU arti: "ukur sekarang". Yang diukur ditentukan keadaan sesi,
 * bukan cara menekannya:
 *
 *   IDLE    -> cek manual. Hasilnya berhenti di layar jam: tidak ada entri
 *              sampel, tidak ada event, tidak ada satu byte pun yang dikirim ke
 *              aplikasi. Inilah "ngukur di luar sesi".
 *   ARMED   -> "Selesai Makan": t0 lahir (SUMBER TUNGGAL, dokumen 3) dan
 *              pengukuran index 1 langsung berjalan.
 *   RUNNING -> pengukuran terjadwal yang sudah jatuh tempo: satu jam setelah
 *              makan (index 2) dan dua jam setelah makan (index 3). Ketiganya --
 *              index 1, 2, 3 -- karena itu lahir dari tombol yang sama, dan
 *              ketiganya terkirim ke aplikasi lewat BLE.
 *
 * Klik singkat dan tekan-lama melakukan hal yang sama, dan itu disengaja. Versi
 * sebelumnya menuntut tahan 700 ms karena pin ini juga strapping pin mode
 * download sehingga gampang tersenggol saat kabel ditancapkan. Alasan itu
 * dibayar terlalu mahal di pemakaian sehari-hari: menahan tombol sambil menahan
 * jari yang lain diam di sensor selama belasan detik adalah dua hal yang tidak
 * bisa dilakukan sekaligus dengan nyaman, sementara senggolan kabel cuma terjadi
 * di meja kerja. Tekan-lama dipertahankan supaya kebiasaan lama tetap bekerja,
 * bukan sebagai syarat.
 *
 * Aksi tekan-lama dijalankan saat ambang terlewati, bukan saat jari dilepas:
 * kalau menunggu pelepasan, pengguna menahan sambil menebak apakah sudah cukup
 * lama, dan tebakan itu satu-satunya umpan balik yang ada. Klik singkat, karena
 * definisinya "dilepas sebelum ambang", memang baru bisa dinilai saat dilepas --
 * penjaga boot_sudah_jalan yang memastikan satu tekanan tidak dihitung dua
 * kali. */
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
 * karena alasan yang benar. true kalau tekanan itu ditolak -- pemanggil memakai
 * itu untuk memilih pesan keberhasilannya sendiri. */
static bool boot_umpan_balik_tolak(void) {
  switch (jam_umpan_balik_ditolak()) {
    case JAM_TOLAK_SEDANG_UKUR:   status_pesan("SEDANG MENGUKUR");   return true;
    case JAM_TOLAK_BATERAI:       status_pesan("BATERAI LEMAH");     return true;
    case JAM_TOLAK_SENSOR:        status_pesan("SENSOR TIDAK ADA");  return true;
    case JAM_TOLAK_SESI_AKTIF:    status_pesan("SESI SEDANG JALAN"); return true;
    case JAM_TOLAK_SESI_BERJALAN: status_pesan("SESI SUDAH MULAI");  return true;
    case JAM_TOLAK_BELUM_ARM:     status_pesan("BELUM DISIAPKAN");   return true;
    default: return false;
  }
}

static tombol_arti_t tombol_arti(void) {
  if (jam_titik_armed())              return TBL_UKUR;
  if (jam_status() == AW_SESI_ARMED)  return TBL_SELESAI_MAKAN;
  if (jam_status() == AW_SESI_IDLE)   return TBL_CEK_MANUAL;
  return TBL_MATI;                    /* RUNNING tanpa titik ter-ARM */
}

static void boot_aktifkan(void) {
  /* TIGA arti sekarang, dan hanya SATU di antaranya dipilih di sini.
   *
   * Pemilihan antara "Ukur" dan "Selesai Makan" ada di dalam jam_tekan_tombol(),
   * bukan di sini, dan itu bukan gaya melainkan syarat: sebuah ARM_TITIK bisa
   * tiba lewat BLE dalam jeda antara baris ini membaca keadaan dan pengguna
   * mengangkat jarinya, dan UI yang memilih sendiri akan mengukur titik yang
   * salah (dokumen 13.4).
   *
   * Yang MASIH dipilih di sini cuma cek manual, dan itu memang bukan bagian
   * protokol: ia fitur lokal yang hasilnya tidak pernah meninggalkan jam. Ia
   * dipetakan ke satu-satunya keadaan yang tersisa -- IDLE tanpa titik ter-ARM --
   * jadi ia tidak pernah bisa mencuri giliran dari jalur protokol mana pun. */
  tombol_arti_t arti = tombol_arti();

  /* Tombol fisik SELALU membawa ke halaman kedua, diterima atau ditolak.
   * Tanpa ini, menekan tombol saat berdiri di halaman utama tidak
   * menghasilkan apa pun yang terlihat: status_pesan()/status_baris() cuma
   * menulis ke lbl_status, dan label itu cuma ada di scr_wajah. Ini jalur
   * yang sama dengan halaman_evaluasi(), sengaja dipaksa lebih awal supaya
   * umpan baliknya tampil sejak frame pertama, bukan menyusul di putaran
   * refresh berikutnya. */
  halaman_set(true);

  if (arti == TBL_CEK_MANUAL) jam_cek_manual();
  else                        jam_tekan_tombol();

  if (boot_umpan_balik_tolak()) return;

  switch (arti) {
    case TBL_UKUR:           status_pesan("MENGUKUR TITIK"); break;
    case TBL_SELESAI_MAKAN:  status_pesan("SELESAI MAKAN");  break;
    case TBL_CEK_MANUAL:     status_pesan("CEK MANUAL");     break;
    default:                 status_pesan("SESI BERJALAN");  break;
  }
}


/* ================= Konsol uji serial (dokumen 15) =================
 * Menyuntik opcode Kontrol persis seperti aplikasi menulisnya, LEWAT ANTREAN
 * YANG SAMA dengan tulisan BLE. Jalur yang sama itu syarat, bukan kenyamanan:
 * memanggil rutin aw_jam langsung akan melewati handler-nya, sehingga cabang
 * NAK dan idempotensi -- yang justru paling sering salah -- tidak pernah teruji.
 *
 * Ia ada karena satu sesi v1.3 tidak bisa diuji tanpa HP sama sekali: sejak jam
 * berhenti menjadwalkan, SETIAP titik sesudah index 1 datang lewat UKUR atau
 * ARM_TITIK. Tanpa konsol ini, satu-satunya cara memverifikasi ARM_TITIK
 * bertahan melewati pemutusan daya adalah menjalankan aplikasi Flutter.
 *
 * Perintah (satu baris, diakhiri Enter):
 *   arm [1|2]    ARM_SESI dengan sesiId uji ke-1 atau ke-2
 *   batal        BATAL_SESI
 *   mulai        MULAI_SESI  (jalankan dua kali untuk menguji idempotensinya)
 *   ukur <idx>   UKUR index idx
 *   titik <idx>  ARM_TITIK index idx
 *   now          UKUR_SEKARANG
 *   tombol       tekan tombol fisik (bukan BLE -- menguji jalur tombol)
 *   status       cetak keadaan jam
 *
 * Satu sesi utuh tanpa HP:
 *   arm -> ukur 0 -> tombol -> titik 2 -> tombol -> titik 3 -> tombol
 */
static const uint8_t SESI_UJI[2][16] = {
  { 0xA1,0xA1,0xA1,0xA1, 0xA1,0xA1,0xA1,0xA1, 0xA1,0xA1,0xA1,0xA1, 0xA1,0xA1,0xA1,0xA1 },
  { 0xB2,0xB2,0xB2,0xB2, 0xB2,0xB2,0xB2,0xB2, 0xB2,0xB2,0xB2,0xB2, 0xB2,0xB2,0xB2,0xB2 },
};
static uint8_t konsol_sesi = 0;      /* sesiId uji yang sedang dipakai */

static void konsol_kirim(uint8_t op, bool bawa_sesi, bool bawa_index, uint8_t index) {
  uint8_t buf[18];
  uint8_t n = 0;
  buf[n++] = op;
  if (bawa_sesi)  { memcpy(&buf[n], SESI_UJI[konsol_sesi], 16); n += 16; }
  if (bawa_index) buf[n++] = index;
  if (!aw_ble_suntik_perintah(buf, n))
    Serial.println("[konsol] antrean penuh");
  else
    Serial.printf("[konsol] -> opcode 0x%02X (%u byte)\n", op, (unsigned)n);
}

/* ================= Probe sag: "apakah jam sedang dicolok?" =================
 *
 * Charger CC/CV adalah sumber TEREGULASI, baterai tidak. Jadi kalau beban
 * diubah mendadak:
 *   - di baterai   -> tegangan ambles sebanding beban x resistansi dalam sel
 *   - saat dicolok -> charger menahannya, amblesnya nyaris nol
 * Bedanya berlaku di SEMUA tingkat isi, termasuk saat sel sudah penuh dan
 * trennya rata -- persis lubang yang tidak bisa ditutup battery_charging() yang
 * menilai tren tegangan 3 menit (dan yang membuat petir hilang justru saat
 * mengisi dari kondisi hampir penuh, keluhan yang memulai semua ini).
 *
 * Bebannya backlight: satu-satunya konsumen besar yang bisa dinyalakan dan
 * dimatikan seketika tanpa efek samping. Hanya duty PWM-nya yang disentuh,
 * BUKAN layar_set() -- menghidupkan panel ikut menjalankan splash, dan animasi
 * itu sendiri adalah beban yang berubah-ubah, yaitu derau yang sedang diukur.
 *
 * Ini masih tahap KALIBRASI: ambangnya belum ada, dan keluaran inilah yang
 * dipakai untuk menetapkannya. Begitu ambangnya ketahuan, probe aktif ini
 * diganti versi pasif yang menumpang pada layar mati/nyala yang toh sudah
 * terjadi sendiri -- tanpa kedipan dan tanpa delay().
 */
typedef struct {
  uint32_t t_s;        /* uptime detik, untuk mengurutkan kejadian     */
  int16_t  berat;      /* mV baterai, backlight menyala                */
  int16_t  ringan;     /* mV baterai, backlight padam                  */
  int16_t  sag;        /* ringan - berat; >0 = beban lepas -> naik     */
  int16_t  hanyut;     /* tren selama probe; memisahkan naik CC dari sag */
  int16_t  derau;      /* sebaran di pin, lantai kebermaknaan          */
} sag_hasil_t;

#define SAG_LOG_N 14
static sag_hasil_t sag_log[SAG_LOG_N];
static int         sag_log_n = 0, sag_log_i = 0;

/* Memblokir ~1,3 detik: LVGL, touch, dan jam_putar() berhenti selama itu.
 * Boleh HANYA karena ini build kalibrasi sementara. Versi pasifnya nanti tidak
 * memblokir sama sekali. */
static void sag_ukur(bool cetak) {
  const int bl_awal = s_layar_nyala ? LCD_BL_TERANG : 0;
  int sb1 = 0, sb2 = 0, sb3 = 0;

  /* Berat diukur DUA KALI mengapit yang ringan. Bukan demi presisi melainkan
   * demi membatalkan hanyutan linear: kalau jam sedang mengisi di fase CC
   * tegangannya naik terus, dan probe dua titik biasa akan melaporkan kenaikan
   * itu sebagai "sag" bertanda salah. */
  ledcWrite(LCD_BL, LCD_BL_TERANG);
  delay(400);
  const int berat1 = battery_baca_langsung_mv(&sb1);

  ledcWrite(LCD_BL, 0);
  delay(400);
  const int ringan = battery_baca_langsung_mv(&sb2);

  ledcWrite(LCD_BL, LCD_BL_TERANG);
  delay(400);
  const int berat2 = battery_baca_langsung_mv(&sb3);

  ledcWrite(LCD_BL, bl_awal);                  /* kembalikan apa adanya */

  const int berat  = (berat1 + berat2 + 1) / 2;
  const int derau  = sb1 > sb2 ? (sb1 > sb3 ? sb1 : sb3) : (sb2 > sb3 ? sb2 : sb3);

  sag_hasil_t *h = &sag_log[sag_log_i];
  h->t_s    = millis() / 1000UL;
  h->berat  = (int16_t)berat;
  h->ringan = (int16_t)ringan;
  h->sag    = (int16_t)(ringan - berat);
  h->hanyut = (int16_t)(berat2 - berat1);
  h->derau  = (int16_t)derau;

  sag_log_i = (sag_log_i + 1) % SAG_LOG_N;
  if (sag_log_n < SAG_LOG_N) sag_log_n++;

  if (cetak)
    Serial.printf("[sag] t=%lus berat=%d/%d ringan=%d  SAG=%+d mV  "
                  "hanyut=%+d mV  derau_pin=%d mV\n",
                  (unsigned long)h->t_s, berat1, berat2, ringan,
                  (int)h->sag, (int)h->hanyut, (int)h->derau);
}

/* Serial mati begitu USB dicabut -- padahal "di baterai" justru salah satu dari
 * dua kondisi yang harus dibandingkan. Jadi hasilnya disimpan di RAM dan dibaca
 * belakangan. Ini bekerja karena mencolok USB lagi TIDAK me-reset board (ia cuma
 * mulai mengisi), pola yang sama persis dengan battery_history(). */
static void sag_cetak_log(void) {
  Serial.printf("[saglog] %d hasil (tertua dulu), uptime sekarang %lus\n",
                sag_log_n, (unsigned long)(millis() / 1000UL));
  for (int k = 0; k < sag_log_n; k++) {
    const sag_hasil_t *h = &sag_log[(sag_log_i - sag_log_n + k + 2 * SAG_LOG_N) % SAG_LOG_N];
    Serial.printf("  t=%5lus  berat=%4d  ringan=%4d  SAG=%+4d  hanyut=%+4d  derau=%d\n",
                  (unsigned long)h->t_s, (int)h->berat, (int)h->ringan,
                  (int)h->sag, (int)h->hanyut, (int)h->derau);
  }
  Serial.println("[saglog] SAG saat dicolok harus jauh lebih kecil daripada "
                 "SAG di baterai; selisihnya = ambang");
}

/* 1 = probe berjalan sendiri tiap SAG_AUTO_MS. HANYA untuk kalibrasi: ia
 * mengedipkan backlight dan memblokir 1,3 detik. Set 0 setelah ambang didapat. */
#define AW_KALIBRASI_SAG 0
#define SAG_AUTO_MS      20000UL

/* Instrumen lag. Murah (dua micros() per iterasi) dan sengaja dibiarkan
 * terpasang: "jamnya terasa lambat" adalah keluhan yang mustahil dikejar tanpa
 * angka, dan menebak-nebak penyebabnya memakan satu siklus flash tiap tebakan. */
static uint32_t lag_maks_us = 0, lag_total_us = 0, lag_n = 0, lag_lambat = 0;

static void konsol_jalankan(char *baris) {
  char *sp = strchr(baris, ' ');
  int arg = 0;
  if (sp) { *sp = 0; arg = atoi(sp + 1); }

  if      (!strcmp(baris, "arm")) {
    if (sp && (arg == 1 || arg == 2)) konsol_sesi = (uint8_t)(arg - 1);
    Serial.printf("[konsol] sesi uji #%u\n", (unsigned)(konsol_sesi + 1));
    konsol_kirim(AW_OP_ARM_SESI, true, false, 0);
  }
  else if (!strcmp(baris, "batal")) konsol_kirim(AW_OP_BATAL_SESI, true, false, 0);
  else if (!strcmp(baris, "mulai")) konsol_kirim(AW_OP_MULAI_SESI, true, false, 0);
  else if (!strcmp(baris, "ukur"))  konsol_kirim(AW_OP_UKUR, true, true, (uint8_t)arg);
  else if (!strcmp(baris, "titik")) konsol_kirim(AW_OP_ARM_TITIK, true, true, (uint8_t)arg);
  else if (!strcmp(baris, "now"))   konsol_kirim(AW_OP_UKUR_SEKARANG, false, false, 0);
  else if (!strcmp(baris, "tombol")) {
    /* Jalur tombol fisik, sengaja BUKAN lewat antrean: yang diuji di sini justru
     * dispatcher satu-tombol-dua-makna, yang tidak punya opcode. */
    Serial.println("[konsol] tombol fisik ditekan");
    boot_aktifkan();
  }
  else if (!strcmp(baris, "status")) {
    static const char *NAMA[] = { "IDLE", "ARMED", "RUNNING" };
    Serial.printf("[konsol] sesi=%s  mengukur=%d  titik_armed=%d index=%u  "
                  "t0=%lu  uptime=%lu  tertunda=%u\n",
                  NAMA[jam_status() <= 2 ? jam_status() : 0],
                  jam_sedang_mengukur() ? 1 : 0,
                  jam_titik_armed() ? 1 : 0, (unsigned)jam_titik_index(),
                  (unsigned long)jam_t0_uptime(), (unsigned long)jam_uptime(),
                  (unsigned)jam_tertunda());
  }
  else if (!strcmp(baris, "lag")) {
    Serial.printf("[lag] loop: maks=%lu us  rata=%lu us  n=%lu  >20ms=%lu  "
                  "(probe ADC terakhir=%lu us)\n",
                  (unsigned long)lag_maks_us,
                  lag_n ? (unsigned long)(lag_total_us / lag_n) : 0UL,
                  (unsigned long)lag_n, (unsigned long)lag_lambat,
                  (unsigned long)battery_probe_us());
    lag_maks_us = 0; lag_total_us = 0; lag_n = 0; lag_lambat = 0;
  }
  else if (!strcmp(baris, "sag"))    sag_ukur(true);
  else if (!strcmp(baris, "saglog")) sag_cetak_log();
  else if (baris[0]) Serial.printf("[konsol] tidak dikenal: \"%s\"\n", baris);
}

static void konsol_poll(void) {
  static char buf[48];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { buf[n] = 0; konsol_jalankan(buf); n = 0; continue; }
    if (n < sizeof(buf) - 1) buf[n++] = c;
  }
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
      /* Layar dibangunkan pada TEKANAN, bukan pada aksinya: pengukuran yang
       * dimulai di layar gelap tidak punya cara memberi tahu kemajuannya, dan
       * kemajuan itulah satu-satunya alasan pengguna mau menahan jari diam. */
      if (!s_layar_nyala) layar_nyala_sementara(LAYAR_MATI_TOMBOL_MS);
    } else if (!boot_siap) {
      boot_siap = true;
      Serial.println("[boot] tombol dilepas -- siap dipakai");
    } else if (!boot_sudah_jalan) {
      boot_aktifkan();                          /* dilepas sebelum ambang lama */
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

  /* Gerbang menyala: tombol harus ditahan sampai 3 detik, kalau tidak jam mati
   * lagi sebelum sempat menampilkan apa pun. Dijalankan tepat setelah latch
   * dipasang, karena latch itulah yang membuat kita punya daya untuk menghitung
   * tiga detiknya sama sekali. */
  bool nyala_disengaja = pwr_gerbang_nyala();

  Serial.begin(115200);

  /* Menulis ke serial TIDAK BOLEH memblokir loop(). Bawaan core menyakitkan:
   * HWCDC.cpp memakai tx_timeout_ms=100 dan max_consec_timeouts=20, jadi SATU
   * Serial.printf() bisa menahan pemanggilnya sampai ~2 DETIK kalau host tidak
   * menguras buffernya.
   *
   * Yang membuatnya jarang terlihat: saat USB dicabut tidak ada host sama
   * sekali, tulisan langsung dibuang, dan loop() melaju normal. Begitu kabel
   * ditancapkan TANPA ada terminal yang membaca -- mengecas dari adaptor,
   * power bank, atau PC dengan Serial Monitor tertutup -- backpressure-nya
   * muncul dan heartbeat 5 detik yang mencetak lima baris itu menjadi lima
   * kesempatan menggantung. Gejalanya persis "jamnya nge-lag kalau ditancap".
   *
   * 0 berarti antre kalau muat, buang kalau penuh. Yang hilang cuma log
   * diagnostik, dan hanya pada detik-detik saat memang tidak ada yang membaca.
   * Itu harga yang jelas lebih murah daripada UI yang membeku. */
  Serial.setTxTimeoutMs(0);
  delay(200);
  Serial.println("\n[boot] AsaWatch -- wajah satu halaman");
  if (!nyala_disengaja)
    Serial.println("[pwr] tombol dilepas sebelum 3 dtk -- board tetap jalan "
                   "karena dicatu USB, bukan baterai");

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
  disp_drv.monitor_cb = spl_monitor;   /* hanya mencacah selama layar pembuka */
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
  build_home();
  build_splash();
  splash_mulai();

  /* Frame pertama digambar SEBELUM cahaya dinyalakan. Yang tampak di frame itu
   * adalah latar rata tanpa satu pun unsur logo (semuanya masih opasitas nol),
   * jadi layar terbuka dari gelap ke gelap -- tanpa kilatan isi RAM panel yang
   * tersisa dari sebelum reset. */
  lv_timer_handler();
  /* Penyalaan backlight pertama juga langkah beban, dan yang paling berharga:
   * ia memberi jawaban "dicolok atau tidak" beberapa ratus milidetik setelah
   * boot, tanpa menunggu pengguna mematikan layar sekali pun. */
  battery_beban_akan_berubah(true);
  ledcWrite(LCD_BL, LCD_BL_TERANG);

  /* Splash diputar sampai habis di sini, bukan dibiarkan berjalan sendiri di
   * loop(). Alasannya ada di splash_tunggu(). Konsekuensinya boot mundur
   * ~SPL_TOTAL_MS, dan itu memang harga yang dibayar dengan sadar.
   *
   * refresh_cb sengaja dibuat SESUDAHNYA. Kalau ia sudah ada selama putaran
   * splash, ia akan menyala satu kali di detik pertama -- yaitu sebelum
   * jam_mulai() -- dan memanggil jam_status()/jam_snapshot() pada mesin sesi
   * yang belum diinisialisasi. Kode lama tidak pernah menemui ini karena
   * lv_timer_handler() cuma dipanggil sekali di sini. */
  splash_tunggu();

  lv_timer_create(refresh_cb, 500, NULL);
  lv_timer_handler();

  /* Arming tombol PWR persis sealasan dengan arming tombol BOOT di bawah, dan
   * di sini keliru diamnya lebih mahal: tanpa baris ini, board yang menyala
   * dengan tombol PWR sudah terlepas -- yaitu setiap kali ia dicatu USB -- tidak
   * pernah melihat tepi apa pun, pwr_siap tidak pernah menjadi true, dan tombol
   * PWR mati total sampai reset. */
  pwr_stabil_lvl = digitalRead(PWR_KEY);
  pwr_level_lalu = pwr_stabil_lvl;
  pwr_siap       = (pwr_stabil_lvl == HIGH);
  Serial.printf("[pwr] tombol PWR %s\n",
                pwr_siap ? "siap (klik = layar on/off, tahan 3 dtk = mati)"
                         : "masih ditahan -- lepaskan dulu");

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
                boot_siap ? "siap (satu tombol, artinya mengikuti keadaan: "
                            "ukur titik / selesai makan / cek manual)"
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

  /* Koreksi halaman: splash_tutup() sudah memuat scr_home lebih dulu karena
   * urutan init ini WAJIB splash sebelum jam_mulai() (dokumen 13.4), jadi ia
   * tidak bisa tahu ada ARM_TITIK yang selamat lintas boot (dokumen 5 & 11).
   * Sekarang jam_mulai() sudah memuat NVS dan mencerminkannya ke RAM, jadi
   * jam_titik_armed() dkk. baru berarti sungguhan mulai dari sini -- kalau
   * tombol ukur memang harus menyala, halaman kedua yang tampil sejak frame
   * pertama loop(), bukan menyusul satu putaran refresh_cb() kemudian. */
  halaman_evaluasi();

  net_begin();

  Serial.printf("[ok] setup selesai, free heap = %lu\n", (unsigned long)ESP.getFreeHeap());
}

void loop() {
  const uint32_t lag_t0 = micros();
  touch_poll();          /* tetap dibaca demi board yang sentuhannya sehat */
  konsol_poll();         /* penyuntik opcode untuk uji tanpa HP (dokumen 15) */
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

#if AW_KALIBRASI_SAG
  /* Probe berkala supaya kondisi "di baterai" ikut terekam tanpa perlu serial.
   * Ditaruh setelah jam_putar() dan bukan sebelumnya supaya perintah BLE yang
   * sudah antre tidak menunggu 1,3 detik ekstra. */
  {
    static uint32_t sag_auto_ms = 0;
    if ((uint32_t)(millis() - sag_auto_ms) >= SAG_AUTO_MS) {
      sag_auto_ms = millis();
      sag_ukur(true);        /* dicetak juga: kalau USB tertancap, langsung terbaca */
    }
  }
#endif

  lv_timer_handler();

  /* Backlight menyala setelah frame pertama selesai digambar, bukan di dalam
   * layar_set(): membangunkan panel di tengah lv_timer callback lalu memanggil
   * lv_timer_handler() dari sana berarti LVGL masuk ke dirinya sendiri. Menunda
   * satu iterasi loop juga yang membuat layar tidak pernah memperlihatkan sisa
   * frame lama sepersekian detik sebelum digambar ulang. */
  if (s_bl_tunda) {
    s_bl_tunda = false;
    ledcWrite(LCD_BL, LCD_BL_TERANG);
  }

  const uint32_t lag_dt = micros() - lag_t0;
  if (lag_dt > lag_maks_us) lag_maks_us = lag_dt;
  if (lag_dt > 20000UL) lag_lambat++;
  lag_total_us += lag_dt; lag_n++;

  delay(2);
}
