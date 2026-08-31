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
 * sepersekian detik sebelum digambar ulang. Sisi mahalnya nyata dan spesifik --
 * sejak v1.2 layar menyala sendiri saat pengukuran sesi dimulai (lihat
 * refresh_cb), dan di situ SPL_TOTAL_MS berdiri persis di antara pengguna dan
 * isyarat "tempelkan jari sekarang". Diset 0 kalau isyarat itu ternyata lebih
 * berharga daripada logonya. */
#define SPL_SAAT_BANGUN 1

static void layar_set(bool nyala) {
  if (nyala && pwr_daya_lepas) return;
  if (nyala == s_layar_nyala) return;
  s_layar_nyala = nyala;

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
#define C_REDUP     0x8E96A6   /* judul kartu, satuan, ikon baterai   */
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
#define BATT_N_KOTAK 3
#define BATT_KANAN   232                     /* tepi kanan tonjolan          */
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
#define BATT_X  (BATT_KANAN - BATT_NUB_W - BATT_W)                    /* 201 */
#define BATT_Y  14                           /* pusatnya sejajar teks header */
#define BATT_KOTAK_X (BATT_X + BATT_BRD + BATT_PAD)                   /* 204 */
#define BATT_KOTAK_Y (BATT_Y + BATT_BRD + BATT_PAD)                   /* 17  */

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

static lv_obj_t *lbl_jam_hh, *lbl_jam_mm;   /* "14" dan "35"                  */
static lv_obj_t *lbl_status;                /* baris tanggal / status         */
/* Ikon baterai digambar sendiri dari objek LVGL, bukan glyph LV_SYMBOL_*.
 * Sebabnya bukan selera: lambang baterai di font hanya punya lima tingkat
 * tetap dan tidak bisa diberi tahu berapa kotak yang boleh menyala, jadi
 * satu-satunya cara menampilkan tepat tiga kotak adalah menggambarnya. */
static lv_obj_t *batt_cangkang, *batt_nub, *batt_petir;
static lv_obj_t *batt_kotak[BATT_N_KOTAK];

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
  batt_cangkang = lv_obj_create(scr_wajah);
  lv_obj_remove_style_all(batt_cangkang);
  lv_obj_set_pos(batt_cangkang, BATT_X, BATT_Y);
  lv_obj_set_size(batt_cangkang, BATT_W, BATT_H);
  lv_obj_set_style_bg_opa(batt_cangkang, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(batt_cangkang, BATT_BRD, 0);
  lv_obj_set_style_border_color(batt_cangkang, lv_color_hex(C_REDUP), 0);
  lv_obj_set_style_border_opa(batt_cangkang, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(batt_cangkang, 3, 0);
  lv_obj_clear_flag(batt_cangkang, LV_OBJ_FLAG_SCROLLABLE);

  batt_nub = mk_box(scr_wajah, BATT_X + BATT_W, BATT_Y + (BATT_H - BATT_NUB_H) / 2,
                    BATT_NUB_W, BATT_NUB_H, C_REDUP, 1);

  for (int i = 0; i < BATT_N_KOTAK; i++)
    batt_kotak[i] = mk_box(scr_wajah, BATT_KOTAK_X + i * BATT_KOTAK_D, BATT_KOTAK_Y,
                           BATT_KOTAK_W, BATT_KOTAK_H, C_REDUP, 1);

  batt_petir = mk_label(scr_wajah, LV_SYMBOL_CHARGE, &lv_font_montserrat_12,
                        C_ISI, 0, 0);
  lv_obj_update_layout(scr_wajah);
  lv_obj_align_to(batt_petir, batt_cangkang, LV_ALIGN_OUT_LEFT_MID, -3, 0);

  /* Mulai dari keadaan "belum ada bacaan": badan kosong, warna peringatan.
   * Bukan tiga kotak penuh -- ikon penuh sebelum ADC sempat mengukur adalah
   * angka karangan dalam bentuk gambar. */
  batt_gambar(0, false);

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

  /* Paling akhir supaya pita tepi berada DI ATAS segalanya: cahayanya memang
   * dimaksudkan jatuh menimpa apa pun yang kebetulan ada di dekat tepi. */
  build_tepi(scr_wajah);
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
 * Ketujuh angka pertama adalah jadwal 850 ms yang lama DIKALI ~1,6 -- dikali,
 * bukan ditulis ulang, supaya tumpang tindih yang sudah ditimbang itu tetap
 * utuh dan yang berubah cuma kecepatannya. Busur menyapu lebih tenang, tanda
 * jam memudar cukup lama untuk benar-benar terlihat memudar, dan kata
 * "ASAWatch" menempuh 8 px-nya sebagai gerakan alih-alih sebagai lompatan.
 *
 * Jeda bacanya naik lebih dari faktor itu (90 -> 180 ms) karena ia menjawab
 * pertanyaan yang berbeda dari keenam lainnya: bukan seberapa cepat sebuah
 * unsur masuk, melainkan berapa lama logo yang SUDAH utuh boleh berdiri diam
 * sebelum layar berganti. Pada 90 ms anak judul praktis belum sempat terbaca
 * penuh saat wajah jam sudah menggantikannya. */
#define SPL_ARC_MS     680
#define SPL_MARK_TUNDA 210
#define SPL_MARK_MS    420
#define SPL_KATA_TUNDA 610
#define SPL_KATA_MS    400
#define SPL_SUB_TUNDA  870
#define SPL_SUB_MS     350
#define SPL_TOTAL_MS   1400     /* 1220 ms animasi + 180 ms jeda baca */

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

  spl_periode(LV_DISP_DEF_REFR_PERIOD);
  lv_scr_load(scr_wajah);

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
static void batt_gambar(int kotak, bool mengisi) {
  uint32_t warna = mengisi ? C_ISI : (kotak == 0 ? C_CINCIN_HR : C_REDUP);

  lv_obj_set_style_border_color(batt_cangkang, lv_color_hex(warna), 0);
  lv_obj_set_style_bg_color(batt_nub, lv_color_hex(warna), 0);

  for (int i = 0; i < BATT_N_KOTAK; i++) {
    if (i < kotak) {
      lv_obj_set_style_bg_color(batt_kotak[i], lv_color_hex(warna), 0);
      lv_obj_clear_flag(batt_kotak[i], LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(batt_kotak[i], LV_OBJ_FLAG_HIDDEN);
    }
  }

  /* Petir muncul di kiri badan, bukan menggantikan isinya. Versi sebelumnya
   * harus menukar lambang karena baris ini cuma menyisakan ~50 px di kanan
   * tanggal; sekarang angka persen sudah pergi dan ruangnya cukup, jadi
   * "sedang mengisi" bisa ditandai dua kali -- warna DAN lambang -- tanpa
   * mengorbankan jumlah kotak yang terbaca. */
  if (mengisi) lv_obj_clear_flag(batt_petir, LV_OBJ_FLAG_HIDDEN);
  else         lv_obj_add_flag(batt_petir, LV_OBJ_FLAG_HIDDEN);
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
    /* Kemajuan persen, bukan sisa detik. Keduanya jujur -- keduanya dihitung
     * ulang dari laju detak yang sungguhan terjadi -- tetapi keduanya berbeda
     * pada satu hal yang menentukan: SISA BOLEH BERTAMBAH.
     *
     * Angka sisa memanjang setiap kali nadinya susah ditemukan, dan itu persis
     * keadaan yang paling sering dialami pengguna lansia (pergelangan dingin,
     * tali agak longgar). Hitung mundur yang naik dari 12 ke 25 terbaca sebagai
     * alat yang rusak atau bertambah lama karena penggunanya salah, dan orang
     * lalu MENGGERAKKAN TANGAN untuk "membetulkan" -- persis hal yang merusak
     * pembacaannya. Persen dijamin tidak pernah mundur (lihat jam_ukur_persen()
     * di aw_jam.cpp): pada nadi yang susah ia berhenti, dan berhenti jauh lebih
     * mudah dibaca sebagai "sedang berusaha" daripada angka yang naik.
     *
     * Ia juga menyamakan bahasa dengan aplikasi, yang menampilkan persen yang
     * SAMA dari byte 8 paket Status (dokumen 8) -- dua layar yang menyebut satu
     * pengukuran dengan dua satuan berbeda mengundang pertanyaan mana yang
     * benar.
     *
     * Kata "UKUR" tetap dipakai walau cincin tepi sudah menggambarkan persen
     * yang sama: baris ini bersebelahan dengan angka baterai, dan "45%" telanjang
     * di sebelah ikon baterai punya satu bacaan yang salah dan sangat wajar.
     * Panjangnya masih aman -- "UKUR 100%" lebih pendek daripada "SISA 120 dtk"
     * yang selama ini muat di ~100 px baris ini (lihat build_wajah()). */
    snprintf(buf, sizeof(buf), "UKUR %u%%", (unsigned)jam_ukur_persen());
    if (set_jika_beda(lbl_status, buf))
      lv_obj_set_style_text_color(lbl_status, lv_color_hex(C_ISI), 0);
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
  if (jam_status() == AW_SESI_ARMED) {
    if (set_jika_beda(lbl_status, "TEKAN: SELESAI MAKAN"))
      lv_obj_set_style_text_color(lbl_status, lv_color_hex(C_ISI), 0);
    return;
  }

  struct tm t;
  if (!tm_now(&t)) {
    if (set_jika_beda(lbl_status, "--"))
      lv_obj_set_style_text_color(lbl_status, lv_color_hex(C_TANGGAL), 0);
    return;
  }
  /* "Wed, 1 Jan 26" -- singkatan bahasa Inggris, tanggal tanpa nol di depan,
   * tahun dua digit. Tabel singkatannya ada di time_manager.cpp, jadi di sini
   * tidak ada pemotongan "%.3s" lagi.
   *
   * Tahunnya muat karena letter_space baris ini dinolkan (lihat build_wajah()):
   * "Wed, 15 Aug 26" berakhir sekitar x=168, sementara kelompok baterai paling
   * lebar ("100%" + ikon) baru mulai di sekitar x=183. Kalau format ini
   * diperpanjang lagi, itulah 15 piksel yang tersisa. */
  const char *hari  = tm_day_name(t.tm_wday);
  const char *bulan = tm_month_name(t.tm_mon);
  snprintf(buf, sizeof(buf), "%s, %d %s %02d", hari, t.tm_mday, bulan,
           (t.tm_year + 1900) % 100);
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

  /* ---- pita kemajuan di tepi layar ----
   * Hanya hidup selama pengukuran; di luar itu tepi layar harus bersih supaya
   * wajah jam kembali seperti desainnya. */
  tepi_set(jam_sedang_mengukur() ? (int)jam_ukur_persen() : 0);

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
  if (ukur_kini && !ukur_lalu && !jam_ukur_lokal()) layar_set(true);
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
      kotak_lalu = kotak;
      isi_lalu   = chg;
      batt_gambar(kotak, chg != 0);
    }
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
  layar_set(true);
  Serial.println("[pwr] latch dipasang lagi -- jam menyala kembali");
}

/* Klik singkat. Saat jam sudah diminta mati (dan cuma bertahan karena USB),
 * klik sengaja tidak berbuat apa pun: menyalakan harus lewat gerbang tiga detik
 * yang sama seperti mematikan, kalau tidak "off" jadi keadaan yang bisa
 * dibatalkan sentuhan tak sengaja. */
static void pwr_klik(void) {
  if (pwr_daya_lepas) return;
  layar_set(!s_layar_nyala);
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
      if (!s_layar_nyala) layar_set(true);
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
  build_splash();
  splash_mulai();

  /* Frame pertama digambar SEBELUM cahaya dinyalakan. Yang tampak di frame itu
   * adalah latar rata tanpa satu pun unsur logo (semuanya masih opasitas nol),
   * jadi layar terbuka dari gelap ke gelap -- tanpa kilatan isi RAM panel yang
   * tersisa dari sebelum reset. */
  lv_timer_handler();
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
  net_begin();

  Serial.printf("[ok] setup selesai, free heap = %lu\n", (unsigned long)ESP.getFreeHeap());
}

void loop() {
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

  delay(2);
}
