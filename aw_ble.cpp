/*
 * Catatan pustaka -- baca sebelum membandingkan berkas ini dengan dokumen.
 *
 * Contoh kode di dokumen ditulis untuk NimBLE-Arduino >= 2.0 (NimBLEDevice,
 * NIMBLE_PROPERTY, NimBLEAdvertisementData). Berkas ini memakai pustaka BLE
 * BAWAAN core Arduino-ESP32 (BLEDevice, BLECharacteristic::PROPERTY_*,
 * BLEAdvertisementData) -- dan itu bukan penyimpangan protokol: di ESP32-C6
 * pustaka bawaan core ITU SENDIRI adalah NimBLE (CONFIG_BT_NIMBLE_ENABLED=y di
 * sdkconfig chip ini), jadi yang berbeda hanya nama kelas pembungkusnya. Byte
 * di kawat identik.
 *
 * Alasannya bukan selera: NimBLE-Arduino 2.5.0 membawa salinan stack NimBLE-nya
 * sendiri, dan di C6 salinan itu tidak bisa di-link -- porting layer-nya
 * mengharapkan simbol NPL dari ROM (r_os_memblock_get, r_os_mempool_init, ...)
 * yang memang tidak ada di ROM maupun libbt.a C6, karena di chip ini
 * CONFIG_BT_LE_CONTROLLER_NPL_OS_PORTING_SUPPORT=y dan implementasinya milik
 * controller. Gejalanya "undefined reference" saat linking, bukan saat
 * kompilasi, jadi jangan tergoda memasang NimBLE-Arduino lagi kalau kode ini
 * suatu saat dipindah: kegagalannya akan terlihat seperti masalah lain.
 *
 * Seluruh isi dokumen bagian 3, 4, 10, 11, dan 13.1 tetap diikuti apa adanya.
 */
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLESecurity.h>
#include <BLEAdvertising.h>
#include <host/ble_store.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esp_mac.h>
#include <string.h>

#include "aw_ble.h"
#include "aw_store.h"

/* ---------------- Keadaan modul ---------------- */
static BLEServer         *s_server = nullptr;
static BLECharacteristic *s_c_info = nullptr;
static BLECharacteristic *s_c_kontrol = nullptr;
static BLECharacteristic *s_c_peristiwa = nullptr;
static BLECharacteristic *s_c_sampel = nullptr;
static BLECharacteristic *s_c_status = nullptr;
static BLECharacteristic *s_c_batt = nullptr;

static QueueHandle_t s_antrean = nullptr;

static uint8_t s_serial[6];
static char    s_nama[24];

static volatile bool     s_terhubung = false;
static volatile uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile bool     s_langganan_sampel = false;
static volatile bool     s_langganan_peristiwa = false;
static volatile bool     s_siap_baru = false;

/* ---- Interval iklan: dibaca dari BOND, bukan dari timer boot (dokumen 3.3) ----
 *
 *   tidak punya bond                       -> 100 ms, TERUS-MENERUS tanpa batas
 *   punya bond, 30 dtk sesudah boot/putus  -> 100 ms
 *   punya bond, sesudah itu                -> 1000 ms
 *
 * Baris pertama yang paling penting dan yang paling mudah salah. Versi
 * sebelumnya memberi 100 ms selama 60 detik sesudah boot lalu turun ke 1000 ms
 * TANPA melihat apakah jam sudah pernah dipasangkan. Akibatnya jam yang belum
 * pernah tersandingkan ikut melambat setelah semenit -- dan karena Android
 * hanya bisa menyambung pada jendela iklan, setiap percobaan connect jadi
 * memakan detik demi detik dan sering kehabisan waktu. Di layar itu terbaca
 * sebagai "pemasangan pertama selalu sulit", bukan sebagai iklan yang lambat.
 * Jam yang belum dipasangkan toh tidak sedang mengerjakan apa-apa, jadi tidak
 * ada baterai yang perlu dihemat di sana.
 *
 * Jendela 30 detik di baris kedua melayani hal yang berbeda: jam yang baru saja
 * terputus dan masih membawa sampel di buffer-nya. Ponsel yang kembali mendekat
 * menemukannya dalam hitungan detik, dengan biaya 30 detik iklan cepat per
 * peristiwa putus.
 *
 * Keputusannya DIBACA ULANG dari jumlah bond, tidak disimpan. Itu yang membuat
 * jam yang bond-nya dihapus dari Pengaturan Bluetooth ponsel otomatis kembali
 * mengiklan cepat, tanpa perlu rutin khusus dan tanpa menunggu boot berikutnya.
 *
 * Satuan BLE 0,625 ms. */
#define ADV_CEPAT       160     /* 100 ms  */
#define ADV_LAMBAT     1600     /* 1000 ms */
#define ADV_JENDELA_MS 30000UL

/* Jendela iklan cepat selama masih ada entri yang menunggu diambil.
 *
 * Ini MENGHIDUPKAN KEMBALI mekanisme yang dicabut di v1.2, dengan bentuk yang
 * berbeda dan dengan alasan yang berubah. Versi lama memakai 10 menit dan
 * dinyalakan ulang oleh entri mana pun yang masih ada, sehingga jam yang
 * ditinggal seharian dengan hasil yang tidak pernah diambil mengiklan 10x lebih
 * sering sepanjang hari demi HP yang memang tidak datang.
 *
 * Yang berubah adalah pola pemakaiannya. Di v1.3 jam TIDAK ditinggal menyala:
 * ia dinyalakan sebentar, diukur, lalu dimatikan lagi -- jadi "baru menyala dan
 * masih memegang hasil" hampir selalu berarti seseorang sedang berdiri di depan
 * HP-nya menunggu sinkronisasi. Justru di situ iklan 1000 ms paling merugikan:
 * Android connect() memindai dengan duty cycle rendah, dan pada pengiklan satu
 * detik, timeout 15 detik sudah marjinal bahkan tanpa gangguan apa pun. Gejala
 * yang muncul bukan "lambat" melainkan "selalu timeout" -- perangkat ditemukan
 * saat memindai, tetapi tidak pernah bisa disambungi.
 *
 * Dua hal yang membuat ini tidak mengulang kesalahan yang sama:
 *
 *   - Jendelanya 5 menit, bukan 10 menit.
 *   - Yang menyalakan ulang adalah entri BARU, bukan entri yang sekadar masih
 *     ada (lihat aw_ble_tertunda()). Jam dengan tujuh hasil basi dan tidak ada
 *     yang menambah karena itu gesit selama lima menit sesudah dinyalakan, lalu
 *     diam -- bukan gesit selamanya. */
#define ADV_GESIT_MS   (5UL * 60UL * 1000UL)

/* Evaluasi ulang dijadwalkan, bukan tiap iterasi loop: loop() berputar tiap
 * ~2 ms dan menanyakan jumlah bond ke stack sesering itu adalah pekerjaan yang
 * tidak menghasilkan apa pun. Satu detik sudah jauh lebih cepat daripada
 * kejadian yang dipantaunya (orang menghapus bond dari menu Bluetooth). */
#define ADV_EVAL_MS     1000UL

/* Daya pancar. Bawaan controller +3 dBm; dinaikkan ke +9 dBm.
 *
 * Ini pengungkit tunggal terbesar untuk "BLE-nya kuat": +6 dB adalah empat kali
 * lipat daya, kira-kira dua kali jarak di ruang bebas, dan yang lebih penting --
 * margin terhadap tubuh pemakainya sendiri. Jam tangan dipakai di pergelangan,
 * dan lengan serta badan menyerap 2,4 GHz dengan sangat efektif; sambungan yang
 * baik saat jam di atas meja bisa putus-putus begitu tangannya diayunkan.
 *
 * TIDAK dinaikkan sampai +20 dBm yang sebenarnya didukung C6: arus pancarnya
 * naik tajam sementara perbaikan jaraknya tinggal 2x lagi, dan pada baterai jam
 * sekecil ini itu pertukaran yang buruk. Kalau jangkauannya masih kurang,
 * ESP_PWR_LVL_P12 langkah berikutnya yang masuk akal. */
#define AW_DAYA_PANCAR  ESP_PWR_LVL_P9

static uint32_t s_adv_jendela_sampai = 0;   /* akhir jendela cepat 30 detik  */
static uint32_t s_gesit_sampai      = 0;   /* akhir jendela "ada entri menunggu" */
static uint16_t s_adv_itvl          = 0;   /* 0 = iklan belum pernah dipasang */
static uint32_t s_adv_eval_ms       = 0;
static volatile bool s_perlu_iklan_ulang = false;

/* Jumlah entri yang belum di-ACK aplikasi, dititipkan aw_jam tiap putaran.
 *
 * volatile karena net_task ikut membacanya lewat aw_ble_jumlah_tertunda(), dan
 * satu byte adalah satu-satunya bentuk yang boleh menyeberang task di sini --
 * aw_ring_tertunda() yang sebenarnya menyusuri 64 slot yang sedang diubah task
 * loop, dan memanggilnya dari luar melanggar dokumen 13.2. */
static volatile uint8_t s_tertunda = 0;

/* Kapan terakhir keaktifan iklan diperiksa -- lihat penjaga di aw_ble_putar(). */
static uint32_t s_periksa_iklan_ms = 0;

/* Diminta dari onConnect (task host NimBLE), dikerjakan di konteks loop. */
static volatile bool s_perlu_param_cepat = false;

/* ---------------- Paket Info (dokumen 4) ----------------
 * Disusun TEPAT saat dibaca supaya uptime_s-nya segar, dan WAJIB murni RAM:
 * fungsi ini berjalan di task host NimBLE, tempat NVS dan ring buffer dilarang
 * disentuh. Semua yang dipakainya -- serial, boot_id, uptime, flag anchor --
 * memang sudah ada sebagai salinan RAM sejak boot. */
static void susun_info(uint8_t *b) {
  memset(b, 0, AW_LEN_INFO);
  b[0] = AW_VERSI_MAYOR;
  b[1] = AW_VERSI_MINOR;
  memcpy(&b[2], s_serial, 6);
  aw_tulis_u16(&b[8], AW_FIRMWARE_BUILD);
  b[10] = AW_KAPASITAS_BUFFER;
  b[11] = AW_KEMAMPUAN;
  aw_tulis_u16(&b[12], aw_boot_id());
  aw_tulis_u32(&b[14], aw_uptime_s());
  b[18] = aw_anchor_boot_ini() ? 0x01 : 0x00;
  b[19] = 0;
}

/* ---------------- Callback ---------------- */
class SrvCB : public BLEServerCallbacks {
  /* Varian NimBLE dipakai karena hanya ia yang membawa ble_gap_conn_desc, dan
   * conn_handle di dalamnya diperlukan updateConnParams(). */
  void onConnect(BLEServer *srv, ble_gap_conn_desc *desc) override {
    (void)srv;
    s_terhubung = true;
    s_conn_handle = desc->conn_handle;
    /* Belum boleh mengirim apa-apa: CCCD ditulis ulang tiap koneksi, jadi
     * langganan koneksi sebelumnya tidak berlaku (dokumen 11). */
    s_langganan_sampel = s_langganan_peristiwa = false;
    /* Parameter tautan dicetak apa adanya. Tanpa angka ini, koneksi yang terasa
     * lambat tidak bisa dibedakan penyebabnya: interval yang longgar, radio yang
     * direbut Wi-Fi, atau enkripsi yang mengulang. Satuan interval 1,25 ms,
     * timeout 10 ms. */
    Serial.printf("[ble] tersambung, handle=%u  itvl=%u (%.1f ms) latency=%u "
                  "timeout=%u (%u ms)\n",
                  (unsigned)s_conn_handle, (unsigned)desc->conn_itvl,
                  desc->conn_itvl * 1.25f, (unsigned)desc->conn_latency,
                  (unsigned)desc->supervision_timeout,
                  (unsigned)desc->supervision_timeout * 10);
    /* Minta tautan yang rapat SEKARANG, sebelum aplikasi mulai menelusuri GATT.
     * Penemuan layanan, penulisan CCCD, dan pengurasan buffer semuanya berupa
     * banyak perjalanan pulang-pergi kecil, dan lamanya ditentukan interval
     * koneksi -- pada interval 500 ms, tiga puluh perjalanan sudah 15 detik.
     * Dilonggarkan lagi setelah langganan lengkap; lihat aw_ble_atur_interval(). */
    s_perlu_param_cepat = true;
  }

  void onDisconnect(BLEServer *srv, ble_gap_conn_desc *desc) override {
    (void)srv; (void)desc;
    s_terhubung = false;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_langganan_sampel = s_langganan_peristiwa = false;
    /* Jam SELALU kembali beriklan setelah putus dan tidak pernah memutus
     * koneksi sendiri saat idle: setiap koneksi yang terbentuk adalah
     * kesempatan memasang anchor (dokumen 3.2). Penyetelan intervalnya ditunda
     * ke konteks loop lewat flag ini. */
    s_perlu_iklan_ulang = true;
    Serial.println("[ble] terputus");
  }
};

class InfoCB : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *c) override {
    uint8_t b[AW_LEN_INFO];
    susun_info(b);
    c->setValue(b, sizeof(b));
  }
  void onWrite(BLECharacteristic *c) override {
    /* Karakteristik Info memang berproperti Write di tabel dokumen 3.1, tetapi
     * dokumen tidak memberinya arti apa pun. Tulisan diterima dan diabaikan --
     * bukan di-NAK, karena NAK dicadangkan untuk opcode di karakteristik
     * Kontrol dan menjawab di sini justru membingungkan aplikasi. */
    (void)c;
    Serial.println("[ble] tulisan ke Info diabaikan (tidak punya arti di v1.1)");
  }
};

class KontrolCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    /* Yang boleh dilakukan di sini hanyalah memindahkan byte mentah ke antrean.
     * Tidak ada penguraian opcode, tidak ada sentuhan ring buffer atau NVS --
     * dokumen 13.1. Callback ini juga harus cepat: respons write baru dikirim
     * setelah ia kembali. */
    String v = c->getValue();
    aw_perintah_t p;
    size_t n = v.length();
    if (n > AW_MAKS_PERINTAH) n = AW_MAKS_PERINTAH;
    p.panjang = (uint8_t)n;
    memcpy(p.data, v.c_str(), n);
    if (xQueueSend(s_antrean, &p, 0) != pdTRUE)
      Serial.println("[ble] antrean perintah penuh, satu perintah hilang");
  }
};

/* onSubscribe untuk Sampel dan Peristiwa. Pengurasan buffer dipicu di sini,
 * BUKAN saat koneksi terbentuk: notifikasi yang dikirim sebelum CCCD ditulis
 * hilang tanpa jejak, sementara entrinya terlanjur ditandai "sudah dikirim" dan
 * tidak datang lagi sampai ada SINKRON berikutnya (dokumen 11). */
class LanggananCB : public BLECharacteristicCallbacks {
public:
  explicit LanggananCB(volatile bool *bendera) : m_bendera(bendera) {}
  void onSubscribe(BLECharacteristic *c, ble_gap_conn_desc *desc,
                   uint16_t nilai) override {
    (void)c; (void)desc;
    *m_bendera = (nilai != 0);
    if (s_langganan_sampel && s_langganan_peristiwa) s_siap_baru = true;
  }
private:
  volatile bool *m_bendera;
};

/* Status disegarkan saat dibaca (dokumen 8). Yang benar-benar basi hanyalah
 * uptime_s -- empat byte pertama sudah diperbarui aw_jam setiap kali berubah --
 * jadi cukup keempat byte itu yang ditambal, dan penambalannya murni RAM. */
class StatusCB : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *c) override {
    if (c->getLength() != AW_LEN_STATUS) return;
    uint8_t b[AW_LEN_STATUS];
    memcpy(b, c->getData(), AW_LEN_STATUS);
    aw_tulis_u32(&b[4], aw_uptime_s());
    c->setValue(b, sizeof(b));
  }
};

/* ---------------- Iklan ---------------- */
static void setel_iklan(uint16_t itvl) {
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->stop();

  /* Paket iklan: flags + Complete List of 128-bit Service UUIDs.
   *
   * UUID-nya HARUS di paket iklan, bukan di scan response. Aplikasi memfilter
   * service UUID di level OS, dan filter itu bekerja pada paket iklan; jam yang
   * menaruh UUID-nya di scan response tidak akan pernah terlihat sama sekali --
   * bukan muncul lalu gagal, melainkan tidak muncul, dengan gejala yang di
   * layar tidak bisa dibedakan dari jam yang mati (dokumen 3.2 & 15).
   *
   * Anggarannya juga tidak menyisakan pilihan: batas 31 byte, UUID 128-bit
   * memakan 18 byte, nama 15, manufacturer data 5 -- total 38. Nama dan
   * manufacturer data yang harus pindah. */
  BLEAdvertisementData data_iklan;
  data_iklan.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  data_iklan.setCompleteServices(BLEUUID(AW_UUID_SERVICE));

  BLEAdvertisementData data_scan;
  data_scan.setName(s_nama);
  /* 2 byte company id little-endian + 1 byte versi mayor, supaya aplikasi bisa
   * menandai firmware terlalu tua SEBELUM menyambung. Disusun byte demi byte
   * lewat concat(char): String(const char*) akan berhenti di NUL pertama, dan
   * payload biner tidak boleh bergantung pada kebetulan bahwa ia tidak memuat
   * nol. */
  String md;
  md.concat((char)(AW_COMPANY_ID & 0xFF));
  md.concat((char)((AW_COMPANY_ID >> 8) & 0xFF));
  md.concat((char)AW_VERSI_MAYOR);
  data_scan.setManufacturerData(md);

  adv->setAdvertisementData(data_iklan);
  adv->setScanResponseData(data_scan);
  adv->setScanResponse(true);
  adv->setMinInterval(itvl);
  adv->setMaxInterval(itvl);
  adv->start();

  s_adv_itvl = itvl;
}

/* Jumlah bond, ditanyakan langsung ke penyimpan NimBLE. Pustaka BLE bawaan
 * Arduino tidak mengekspos getNumBonds(), tetapi API host di bawahnya ada. */
static bool adv_ada_bond(void) {
  int n = 0;
  return ble_store_util_count(BLE_STORE_OBJ_TYPE_OUR_SEC, &n) == 0 && n > 0;
}

/* Alasannya ikut dicetak, bukan cuma angkanya. Tanpa itu, "1000 ms" saat
 * menguji pemasangan pertama tidak bisa dibedakan dari bug -- keduanya terlihat
 * sama persis di serial. */
static void perbarui_interval_iklan(bool paksa) {
  bool     ada_bond = adv_ada_bond();
  bool     jendela  = (int32_t)(millis() - s_adv_jendela_sampai) < 0;
  bool     gesit    = s_tertunda > 0 &&
                      (int32_t)(millis() - s_gesit_sampai) < 0;
  uint16_t mau      = (!ada_bond || jendela || gesit) ? ADV_CEPAT : ADV_LAMBAT;

  if (!paksa && mau == s_adv_itvl) return;
  setel_iklan(mau);

  if (!ada_bond)     Serial.println("[ble] iklan 100 ms (tanpa bond, tanpa batas waktu)");
  else if (jendela)  Serial.printf("[ble] iklan 100 ms (jendela %lu dtk lagi)\n",
                                   (unsigned long)((s_adv_jendela_sampai - millis()) / 1000UL));
  else if (gesit)    Serial.printf("[ble] iklan 100 ms (%u entri menunggu, %lu dtk lagi)\n",
                                   (unsigned)s_tertunda,
                                   (unsigned long)((s_gesit_sampai - millis()) / 1000UL));
  else               Serial.printf("[ble] iklan 1000 ms (ber-bond, hemat)%s\n",
                                   s_tertunda ? " -- entri tertunda, jendela gesit habis" : "");
}

/* ---------------- Init ---------------- */
void aw_ble_begin(void) {
  /* Serial 6 byte dari MAC efuse: stabil lintas boot DAN lintas platform, tidak
   * seperti id perangkat dari OS (MAC di Android, UUID di iOS). */
  esp_efuse_mac_get_default(s_serial);

  /* Label uji (aw_store, "id N" di konsol serial) MENANG atas suffix hex
   * MAC kalau sudah diatur -- lihat komentar aw_label_get(). Diperlukan
   * karena suffix MAC terbukti TIDAK cukup unik pada batch 10 unit
   * pengujian: bukan cuma 2 byte terakhir (3 dari 10 kebetulan sama,
   * "FE16"), 3 byte terakhir (6 hex) pun masih kebetulan bertabrakan pada
   * unit lain -- variasi byte rendah MAC batch ini ternyata sempit. Label
   * manual adalah satu-satunya cara yang PASTI, tidak bergantung MAC sama
   * sekali. 0 = belum diatur, jatuh ke suffix MAC 6-hex seperti biasa. */
  uint8_t label = aw_label_get();
  if (label)
    snprintf(s_nama, sizeof(s_nama), "AsaWatch %02u", (unsigned)label);
  else
    snprintf(s_nama, sizeof(s_nama), "AsaWatch%02X%02X%02X",
             s_serial[3], s_serial[4], s_serial[5]);

  s_antrean = xQueueCreate(8, sizeof(aw_perintah_t));

  BLEDevice::init(String(s_nama));
  /* Minta 185; sampel 31 byte harus utuh dalam SATU notifikasi, tidak boleh
   * dipecah (dokumen 10 & 15). */
  BLEDevice::setMTU(185);

  /* Daya pancar dinaikkan untuk ketiga peran sekaligus. Menyetel yang DEFAULT
   * saja tidak cukup: pada controller ESP, iklan punya setelan dayanya sendiri,
   * dan iklan justru bagian yang paling menentukan apakah HP bisa MENEMUKAN jam
   * dari seberang ruangan. */
  BLEDevice::setPower(AW_DAYA_PANCAR, ESP_BLE_PWR_TYPE_ADV);
  BLEDevice::setPower(AW_DAYA_PANCAR, ESP_BLE_PWR_TYPE_SCAN);
  BLEDevice::setPower(AW_DAYA_PANCAR, ESP_BLE_PWR_TYPE_DEFAULT);

  /* Bonding + LE Secure Connections + Just Works, persis dokumen 10.
   *
   * MITM sengaja false, dan itu BUKAN pelonggaran keamanan. Dengan IO
   * capability NO_INPUT_OUTPUT, MITM tidak akan pernah tercapai apa pun yang
   * diminta -- jam tidak punya layar pairing, jadi satu-satunya passkey yang
   * bisa dipakai adalah angka yang dipatok di firmware, dan angka seperti itu
   * tertulis di firmware, di dokumen, dan di setiap salinan keduanya. Ia
   * tampilan, bukan perlindungan.
   *
   * Memintanya true berarti menuntut jaminan yang tidak bisa dipenuhi, dan
   * yang didapat bukan keamanan tambahan melainkan risiko pairing DITOLAK --
   * gejala "pairing kadang gagal" yang mahal dilacak karena tidak deterministik.
   *
   * Yang benar-benar melindungi data kesehatan di sini adalah enkripsi tautan
   * dan bonding, dan keduanya tetap menyala di baris yang sama. Jangan
   * mengembalikannya ke true karena "kelihatannya lebih aman". Peninjauan ulang
   * hanya masuk akal kalau kelak jam menampilkan passkey ACAK di layarnya;
   * passkey tetap justru lebih buruk daripada Just Works, bukan lebih baik. */
  BLESecurity::setAuthenticationMode(true, false, true);
  BLESecurity::setCapability(BLE_HS_IO_NO_INPUT_OUTPUT);

  s_server = BLEDevice::createServer();
  s_server->setCallbacks(new SrvCB());

  /* ---- Layanan AsaWatch: enkripsi wajib di kelima karakteristik ---- */
  BLEService *svc = s_server->createService(BLEUUID(AW_UUID_SERVICE), 40);

  s_c_info = svc->createCharacteristic(
    BLEUUID(AW_UUID_INFO),
    BLECharacteristic::PROPERTY_READ  | BLECharacteristic::PROPERTY_READ_ENC |
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_ENC);
  s_c_info->setCallbacks(new InfoCB());
  { uint8_t b[AW_LEN_INFO]; susun_info(b); s_c_info->setValue(b, sizeof(b)); }

  s_c_kontrol = svc->createCharacteristic(
    BLEUUID(AW_UUID_KONTROL),
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_ENC);
  s_c_kontrol->setCallbacks(new KontrolCB());

  s_c_peristiwa = svc->createCharacteristic(
    BLEUUID(AW_UUID_PERISTIWA),
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ_ENC);
  s_c_peristiwa->setCallbacks(new LanggananCB(&s_langganan_peristiwa));

  s_c_sampel = svc->createCharacteristic(
    BLEUUID(AW_UUID_SAMPEL),
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ_ENC);
  s_c_sampel->setCallbacks(new LanggananCB(&s_langganan_sampel));

  s_c_status = svc->createCharacteristic(
    BLEUUID(AW_UUID_STATUS),
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_READ_ENC |
    BLECharacteristic::PROPERTY_NOTIFY);
  s_c_status->setCallbacks(new StatusCB());
  { uint8_t b[AW_LEN_STATUS] = { 0 }; s_c_status->setValue(b, sizeof(b)); }

  svc->start();

  /* ---- Baterai: layanan standar, bukan karakteristik kustom (dokumen 3.1).
   * Sengaja TANPA syarat enkripsi -- ia standar SIG, dipakai apa adanya oleh OS
   * dan aplikasi mana pun, dan persentase baterai bukan data kesehatan. */
  BLEService *svc_batt = s_server->createService(BLEUUID(AW_UUID_SVC_BATT));
  s_c_batt = svc_batt->createCharacteristic(
    BLEUUID(AW_UUID_CHR_BATT),
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  { uint8_t nol = 0; s_c_batt->setValue(&nol, 1); }
  svc_batt->start();

  /* Descriptor 0x2902 (CCCD) TIDAK ditambahkan manual: NimBLE membuatnya
   * sendiri begitu sebuah karakteristik punya properti notify, dan menambahkan
   * yang kedua justru ditolak. */

  s_adv_jendela_sampai = millis() + ADV_JENDELA_MS;
  perbarui_interval_iklan(true);
  Serial.printf("[ble] \"%s\" mengiklan, serial %02X%02X%02X%02X%02X%02X\n",
                s_nama, s_serial[0], s_serial[1], s_serial[2],
                s_serial[3], s_serial[4], s_serial[5]);
}

/* ---------------- Putaran ---------------- */
void aw_ble_tertunda(uint8_t n) {
  /* NAIKNYA angka ini yang membuka jendela gesit, bukan nilainya yang bukan nol.
   * Bedanya itu yang menjaga jam tetap hemat: hasil yang sama, yang sudah lama
   * menunggu, tidak memperpanjang apa pun -- ia hanya sempat gesit sekali,
   * selama ADV_GESIT_MS, lalu ikut turun ke 1000 ms bersama yang lain.
   *
   * Boot ikut tercakup tanpa baris khusus: s_tertunda mulai dari 0, jadi
   * panggilan pertama sesudah aw_store_begin() memuat ring buffer yang tidak
   * kosong selalu terbaca sebagai kenaikan. Itu justru kasus yang paling sering
   * di v1.3 -- jam dinyalakan membawa hasil, dan detik-detik pertamanya adalah
   * saat seseorang mencoba menyambung. */
  if (n > s_tertunda) s_gesit_sampai = millis() + ADV_GESIT_MS;
  s_tertunda = n;
}

uint8_t aw_ble_jumlah_tertunda(void) { return s_tertunda; }

void aw_ble_putar(void) {
  /* Parameter cepat dipasang di konteks loop, bukan di dalam onConnect: aturan
   * dokumen 13.1 melarang pekerjaan apa pun di task host NimBLE selain
   * memindahkan byte, dan permintaan pembaruan parameter memicu prosedur GAP
   * yang lebih baik tidak dijalankan dari dalam callback-nya sendiri. */
  if (s_perlu_param_cepat) {
    s_perlu_param_cepat = false;
    if (s_terhubung && s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
      s_server->updateConnParams(s_conn_handle, 12, 24, 0, 600);
      Serial.println("[ble] minta interval rapat (15-30 ms) untuk penelusuran GATT");
    }
  }

  if (s_perlu_iklan_ulang) {
    s_perlu_iklan_ulang = false;
    /* Jendela 30 detik dibuka lagi: jam yang baru putus adalah jam yang paling
     * mungkin masih memegang sampel dan paling perlu cepat ditemukan lagi. */
    s_adv_jendela_sampai = millis() + ADV_JENDELA_MS;
    perbarui_interval_iklan(true);
    return;
  }
  if (s_terhubung) return;

  /* ---- Penjaga: pastikan iklannya BENAR-BENAR menyala ----
   * setel_iklan() memanggil adv->start(), tetapi tidak ada yang menjamin
   * panggilan itu berhasil -- dan kalau ia gagal (mis. bentrok dengan operasi
   * GAP lain tepat saat koneksi putus), jam menjadi tidak terlihat SELAMANYA
   * tanpa satu pun gejala di layar. Itu kegagalan yang paling mahal di sini:
   * hasil pengukuran menumpuk di buffer sementara HP tidak pernah bisa
   * menemukan jamnya lagi, dan satu-satunya pemulihannya reboot.
   *
   * ble_gap_adv_active() menanyakan langsung ke stack, bukan ke bendera kita
   * sendiri, jadi ia juga menangkap kasus iklan yang dimatikan dari dalam. */
  if ((uint32_t)(millis() - s_periksa_iklan_ms) >= 5000) {
    s_periksa_iklan_ms = millis();
    if (!ble_gap_adv_active()) {
      Serial.println("[ble] iklan tidak aktif padahal tidak tersambung -- dinyalakan ulang");
      setel_iklan(s_adv_itvl ? s_adv_itvl : ADV_CEPAT);
      return;
    }
  }

  /* Keputusan interval dibaca ulang dari jumlah bond, dijadwalkan sedetik
   * sekali. Inilah yang membuat penghapusan bond dari menu Bluetooth ponsel
   * langsung mengembalikan jam ke iklan cepat tanpa rutin khusus apa pun. */
  if ((uint32_t)(millis() - s_adv_eval_ms) >= ADV_EVAL_MS) {
    s_adv_eval_ms = millis();
    perbarui_interval_iklan(false);
  }
}

/* ---------------- Perintah masuk ---------------- */
bool aw_ble_suntik_perintah(const uint8_t *data, uint8_t panjang) {
  if (!s_antrean) return false;
  aw_perintah_t p;
  if (panjang > AW_MAKS_PERINTAH) panjang = AW_MAKS_PERINTAH;
  p.panjang = panjang;
  memcpy(p.data, data, panjang);
  return xQueueSend(s_antrean, &p, 0) == pdTRUE;
}

bool aw_ble_ambil_perintah(aw_perintah_t *out) {
  if (!s_antrean) return false;
  return xQueueReceive(s_antrean, out, 0) == pdTRUE;
}

/* ---------------- Pengiriman ---------------- */
bool aw_ble_kirim_sampel(const uint8_t *paket) {
  if (!s_terhubung || !s_langganan_sampel) return false;
  s_c_sampel->setValue(paket, AW_LEN_SAMPEL);
  s_c_sampel->notify();
  return true;
}

bool aw_ble_kirim_peristiwa(const uint8_t *paket) {
  if (!s_terhubung || !s_langganan_peristiwa) return false;
  s_c_peristiwa->setValue(paket, AW_LEN_PERISTIWA);
  s_c_peristiwa->notify();
  return true;
}

void aw_ble_set_status(const uint8_t *paket) {
  if (!s_c_status) return;
  s_c_status->setValue(paket, AW_LEN_STATUS);
  /* Notify hanya kalau ada yang tersambung; nilainya tetap tersimpan sehingga
   * pembacaan berikutnya sudah benar. */
  if (s_terhubung) s_c_status->notify();
}

void aw_ble_set_baterai(uint8_t persen) {
  if (!s_c_batt) return;
  s_c_batt->setValue(&persen, 1);
  if (s_terhubung) s_c_batt->notify();
}

/* ---------------- Keadaan ---------------- */
bool aw_ble_terhubung(void) { return s_terhubung; }

bool aw_ble_siap_notifikasi(void) {
  return s_terhubung && s_langganan_sampel && s_langganan_peristiwa;
}

bool aw_ble_ambil_flag_siap_baru(void) {
  bool f = s_siap_baru;
  s_siap_baru = false;
  return f;
}

void aw_ble_atur_interval(bool sibuk) {
  if (!s_terhubung || s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;
  /* Satuan interval 1,25 ms; satuan supervision timeout 10 ms. Rapat saat sesi
   * berjalan supaya sampel sampai tanpa tertahan; longgar saat idle supaya radio
   * tidak menguras baterai.
   *
   * Timeout dinaikkan dari 4 detik ke 6 detik di kedua keadaan. Timeout adalah
   * berapa lama tautan boleh sunyi sebelum dianggap putus, dan 4 detik terlalu
   * pendek untuk barang yang dipakai di pergelangan tangan: satu ayunan lengan
   * yang menaruh badan pemakainya tepat di antara jam dan HP sudah bisa menelan
   * beberapa detik paket. Putus di situ bukan cuma "menyambung lagi" -- CCCD
   * ikut hilang, aplikasi harus berlangganan ulang, dan seluruh buffer diantre
   * ulang. Dua detik tambahan jauh lebih murah daripada itu.
   *
   * Tidak dinaikkan lebih jauh karena timeout juga menentukan berapa lama jam
   * merasa "masih tersambung" padahal HP-nya sudah pergi -- selama itu ia tidak
   * mengiklan, dan tidak bisa ditemukan siapa pun. */
  /* Keadaan idle memakai INTERVAL YANG SAMA rapatnya, dan menghemat lewat slave
   * latency 20 -- bukan lewat interval yang dilonggarkan.
   *
   * Hasilnya sama untuk baterai: jam boleh melewatkan 20 selang berturut-turut,
   * jadi radionya tetap bangun sekitar sekali per 600 ms, persis seperti
   * interval 500 ms yang dipakai sebelumnya. Bedanya ada di dua hal yang justru
   * paling terasa:
   *
   *   1. Begitu jam PUNYA sesuatu untuk dikirim, ia boleh bangun di selang
   *      berikutnya -- 30 ms, bukan menunggu 500 ms. Latency adalah izin
   *      melewatkan, bukan kewajiban.
   *   2. Pusat menyimpan parameter terakhir yang diminta perangkat dan
   *      memakainya untuk koneksi BERIKUTNYA. Dengan interval longgar, setiap
   *      penyambungan ulang dimulai pada 495 ms, dan seluruh penelusuran GATT --
   *      puluhan perjalanan pulang-pergi -- berjalan pada kecepatan itu.
   *      Terukur langsung: 13-17 detik untuk menyambung, dan 3 detik hanya untuk
   *      membaca satu karakteristik. Dengan interval rapat + latency, angka yang
   *      sama turun ke sekitar 1 detik.
   *
   * Supervision timeout 6 detik tetap aman: syaratnya harus lebih besar dari
   * (1 + latency) x interval maks x 2 = 21 x 50 ms x 2 = 2,1 detik. */
  if (sibuk) s_server->updateConnParams(s_conn_handle, 24, 40,  0, 600);
  else       s_server->updateConnParams(s_conn_handle, 24, 40, 20, 600);
  Serial.printf("[ble] interval diminta: 30-50 ms latency=%s\n",
                sibuk ? "0 (ada pekerjaan)" : "20 (idle, hemat daya)");
}

const uint8_t *aw_ble_serial(void) { return s_serial; }
const char    *aw_ble_nama(void)   { return s_nama; }
