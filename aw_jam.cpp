#include <Arduino.h>
#include <string.h>
#include <math.h>

#include "aw_jam.h"
#include "aw_ble.h"
#include "aw_store.h"
#include "ppg.h"
#include "battery.h"

/* ---------------- Parameter pengukuran ================================
 * Pengukuran selesai karena DATANYA cukup, bukan karena waktunya habis.
 *
 * Versi pertama memakai batas 60 detik dan mengakhiri pengukuran begitu keempat
 * metrik pernah terlihat sah sekali. Itu keliru, dan cara kelirunya halus:
 * ppg.cpp menyatakan SpO2, glukosa, dan tekanan darah sah pada window yang SAMA
 * -- yaitu window pertama setelah filter DC tenang (8 detik) dan detak dianggap
 * konsisten -- sementara "konsisten" di sana cuma berarti MIN_STABLE_BEATS = 3
 * detak. Jadi keempat metrik lahir hampir bersamaan sekitar detik ke-9, dan
 * pengukuran langsung dinyatakan selesai. Di layar keempat titik belum sempat
 * digambar hijau (UI disegarkan tiap 500 ms), sehingga terlihat seperti
 * "selesai padahal belum selesai".
 *
 * Tiga detak juga memang terlalu sedikit untuk dipercaya: currentBPM di ppg.cpp
 * adalah rata-rata 4 interval terakhir, jadi dengan 3 detak satu interval yang
 * salah baca menggeser hasilnya puluhan bpm.
 *
 * Karena itu gerbangnya sekarang jumlah detak. 15 dipilih, bukan diambil dari
 * udara: pada 60-100 bpm itu 9-15 detik MENCACAH -- di atas 8 detik penenangan
 * DC, jadi total 17-23 detik per pengukuran. Cukup panjang untuk membuat
 * rata-rata IBI berarti dan beberapa window SpO2 terkumpul, masih cukup pendek
 * untuk menahan jari diam. Naikkan kalau hasilnya masih goyang; turunkan kalau
 * pengguna mengeluh kelamaan.
 */
#define UKUR_MIN_DETAK  15

/* Tidak ada batas waktu selama jari masih menempel: pengukuran menunggu sampai
 * datanya lengkap, apa pun lamanya. Dua penjaga di bawah ini bukan batas
 * kesabaran melainkan penjaga daya -- keduanya hanya menyala pada keadaan yang
 * jelas bukan "sedang mengukur".
 *
 * Tanpa keduanya, jam yang tergeletak di meja dengan sesi RUNNING akan menyala
 * LED-nya selamanya (puluhan mA) dan sesi dua jamnya tidak akan pernah selesai,
 * karena pengukuran index 3 tidak pernah berakhir. */
#define UKUR_TANPA_KONTAK_MS  90000UL    /* 90 dtk tanpa kulit menempel   */
#define UKUR_BATAS_KERAS_MS  300000UL    /* 5 menit, apa pun keadaannya   */

/* ---------------- Keadaan sesi ---------------- */
static uint8_t  s_status = AW_SESI_IDLE;
static uint8_t  s_sesi_id[16];
static uint32_t s_arm_uptime = 0;
static uint32_t s_t0_uptime  = 0;
static uint8_t  s_index_selesai = 0;      /* bitmask index yang sudah dijadwalkan */

/* ---------------- Keadaan pengukuran ---------------- */
static bool     s_ukur_aktif = false;
static uint8_t  s_ukur_index = 0;
static uint8_t  s_ukur_sesi[16];
static uint32_t s_ukur_mulai_ms = 0;
static uint32_t s_ukur_uptime_s = 0;
static uint32_t s_ukur_kontak_ms = 0;   /* terakhir kali kulit terdeteksi */
static uint16_t s_ukur_detak = 0;       /* detak yang sudah tercacah      */

/* Cek manual: pengukuran yang hasilnya HANYA untuk layar jam.
 *
 * Ia tidak menyentuh protokol sama sekali -- tidak ada entri sampel, tidak ada
 * event UKUR_GAGAL, tidak ada perubahan status sesi. Itu bukan kelalaian
 * melainkan syaratnya: dokumen 1 menempatkan jam sebagai sensor + buffer +
 * pencacah, dan dokumen 18 mengunci keputusan produk bahwa jam hanya MENGUKUR
 * UNTUK APLIKASI saat sesi makan. Angka yang lahir di luar sesi tidak punya
 * sesiId, tidak punya index yang berarti, dan tidak masuk analisis mana pun --
 * mengirimkannya hanya akan memenuhi buffer 64 entri dengan data yang aplikasi
 * sendiri tidak tahu harus diapakan.
 *
 * Bedakan dari opcode UKUR_SEKARANG (0x05): itu juga di luar sesi, tetapi
 * diminta APLIKASI untuk kalibrasi dan memang dijawab paket Sampel. Yang ini
 * diminta jari pengguna dan berhenti di layar. */
static bool s_ukur_lokal = false;

/* Akumulator: begitu sebuah metrik pernah terbaca sah selama jendela
 * pengukuran, nilainya dipegang. Sengaja bukan "ambil semua sekaligus di detik
 * terakhir": keempat metrik matang pada waktu berbeda (BPM lebih dulu, glukosa
 * dan tensi paling akhir), dan menunggu keempatnya matang BERSAMAAN berarti
 * sering menyerah dengan tangan kosong padahal tiga di antaranya sudah sempat
 * terbaca. 0 tetap berarti gagal -- lihat sentinel di dokumen 6. */
static uint16_t s_acc_gula = 0;
static uint8_t  s_acc_bpm = 0, s_acc_sis = 0, s_acc_dia = 0, s_acc_spo2 = 0;

/* Salinan hasil pengukuran terakhir, khusus untuk layar. Terpisah dari ring
 * buffer dengan sengaja: ring adalah antrean kirim, bukan tempat UI mengintip,
 * dan entri di sana bisa hilang begitu aplikasi meng-ack-nya. */
static bool     s_hasil_ada = false;
static uint16_t s_hasil_gula = 0;
static uint8_t  s_hasil_bpm = 0, s_hasil_sis = 0, s_hasil_dia = 0, s_hasil_spo2 = 0;

static uint8_t s_tolak = JAM_TOLAK_TIDAK_ADA;

/* Salinan paket Status terakhir, supaya notifikasi hanya dikirim saat isinya
 * benar-benar berubah. */
static uint8_t s_status_paket[AW_LEN_STATUS];
static bool    s_status_pernah = false;

static const uint8_t SESI_NOL[16] = { 0 };

/* ================= Utilitas ================= */
static bool baterai_kritis(void) {
  return battery_valid() && battery_percent() < AW_BATERAI_KRITIS_PCT;
}

/* Klem ke 1..255: 0 disisihkan sebagai sentinel "gagal diukur", jadi nilai sah
 * tidak boleh pernah membulat menjadi 0 dan menyamar sebagai kegagalan. */
static uint8_t klem_u8(float v) {
  long x = lroundf(v);
  if (x < 1)   x = 1;
  if (x > 255) x = 255;
  return (uint8_t)x;
}

static uint16_t klem_u16(float v) {
  long x = lroundf(v);
  if (x < 1)     x = 1;
  if (x > 65535) x = 65535;
  return (uint16_t)x;
}

/* ================= Balasan ACK / NAK =================
 * Dikirim LANGSUNG, sekali, tidak pernah lewat ring buffer dan tidak pernah
 * dikirim ulang. Ini kesalahan yang paling mahal dan paling lambat ketahuan
 * kalau dilanggar: aplikasi tidak pernah meng-ACK_EVENT sebuah balasan, jadi
 * ACK yang menunggu di-ack tidak akan pernah dibersihkan, buffer 64 entri
 * terisi penuh oleh ACK basi, lalu mulai membuang sampel sungguhan (dokumen 7).
 *
 * seq-nya 0, yang memang sudah disisihkan sebagai "bukan entri buffer". */
static void balas(uint8_t jenis, uint8_t payload) {
  uint8_t b[AW_LEN_PERISTIWA];
  memset(b, 0, sizeof(b));
  b[0] = 0;
  b[1] = jenis;
  memcpy(&b[2], s_sesi_id, 16);
  aw_tulis_u16(&b[18], aw_boot_id());
  aw_tulis_u32(&b[20], aw_uptime_s());
  b[24] = 0;
  b[25] = payload;
  aw_ble_kirim_peristiwa(b);
}

static void ack(uint8_t opcode) { balas(AW_EV_ACK, opcode); }
static void nak(uint8_t kode)   { balas(AW_EV_NAK, kode);   }

/* ================= Paket Status (dokumen 8) ================= */
static void kirim_status(void) {
  uint8_t b[AW_LEN_STATUS];
  b[0] = s_status;
  b[1] = aw_ring_tertunda();
  b[2] = battery_valid() ? (uint8_t)battery_percent() : 0;
  b[3] = (uint8_t)((s_ukur_aktif        ? AW_ST_SEDANG_MENGUKUR     : 0) |
                   (aw_kalibrasi_ada()  ? AW_ST_KALIBRASI_TERSIMPAN : 0) |
                   (baterai_kritis()    ? AW_ST_BATERAI_KRITIS      : 0) |
                   (aw_anchor_boot_ini() ? AW_ST_ADA_ANCHOR         : 0));
  aw_tulis_u32(&b[4], aw_uptime_s());

  /* uptime (byte 4..7) tidak ikut dibandingkan: ia berubah tiap detik, dan
   * memakainya sebagai pemicu berarti mengirim notifikasi Status sekali per
   * detik selamanya. Yang menarik bagi aplikasi adalah empat byte pertama. */
  if (s_status_pernah && memcmp(b, s_status_paket, 4) == 0) return;
  memcpy(s_status_paket, b, sizeof(b));
  s_status_pernah = true;
  aw_ble_set_status(b);
}

/* ================= Pengukuran ================= */
static void ukur_mulai(uint8_t index, const uint8_t *sesi_id, bool lokal) {
  s_ukur_aktif    = true;
  s_ukur_lokal    = lokal;
  s_ukur_index    = index;
  memcpy(s_ukur_sesi, sesi_id ? sesi_id : SESI_NOL, 16);
  s_ukur_mulai_ms = millis();
  s_ukur_kontak_ms = millis();
  s_ukur_uptime_s = aw_uptime_s();
  s_ukur_detak = 0;
  s_acc_gula = 0; s_acc_bpm = 0; s_acc_sis = 0; s_acc_dia = 0; s_acc_spo2 = 0;

  /* Hasil lama dibuang SEBELUM LED menyala. Tanpa ini, ppg_get() masih
   * menyimpan salinan hasil pengukuran sebelumnya (held=true) dan pengukuran
   * "selesai" dalam sepersekian detik dengan angka berjam-jam yang lalu --
   * kegagalan yang tidak akan terlihat di layar karena angkanya wajar. */
  ppg_clear_hold();
  ppg_set_enabled(true);        /* <- MAX30105 menyala di sini */

  Serial.printf("[ukur] mulai %s index %u, MAX30105 menyala\n",
                lokal ? "CEK MANUAL (tidak dikirim)" : "index", (unsigned)index);
  kirim_status();
}

/* Hentikan cek manual tanpa menghasilkan apa pun. Dipakai saat aplikasi
 * meng-ARM sesi di tengah cek manual: sesi punya prioritas, dan meninggalkan
 * MAX30105 menyala untuk pengukuran yang hasilnya toh dibuang berarti tombol
 * "Selesai Makan" ikut terkunci sampai cek itu selesai. */
static void ukur_batal_lokal(void) {
  if (!s_ukur_aktif || !s_ukur_lokal) return;
  ppg_set_enabled(false);
  s_ukur_aktif = false;
  s_ukur_lokal = false;
  Serial.println("[ukur] cek manual dibatalkan, sesi punya prioritas");
  kirim_status();
}

static void ukur_selesai(bool lengkap) {
  /* MAX30105 dipadamkan begitu datanya lengkap -- ini yang membuat LED tidak
   * menyala terus-menerus. Dua LED pada arus penuh 100 Hz menyedot puluhan mA
   * dan tetap mengalir walau tidak ada kulit menempel, jadi setiap detik
   * tambahan setelah datanya cukup adalah baterai yang terbuang percuma. */
  ppg_set_enabled(false);
  s_ukur_aktif = false;

  /* Kalibrasi diterapkan ke pembacaan mentah dan TIDAK menyentuh sentinel:
   * nol berarti "gagal diukur", bukan "nol mmHg" (dokumen 5 & 16). */
  if (aw_kalibrasi_ada()) {
    int16_t osis, odia;
    aw_kalibrasi_get(&osis, &odia);
    if (s_acc_sis) s_acc_sis = (uint8_t)constrain((int)s_acc_sis + osis, 1, 255);
    if (s_acc_dia) s_acc_dia = (uint8_t)constrain((int)s_acc_dia + odia, 1, 255);
  }

  bool ada = s_acc_gula || s_acc_bpm || s_acc_sis || s_acc_dia || s_acc_spo2;
  if (ada) {
    s_hasil_ada  = true;
    s_hasil_gula = s_acc_gula; s_hasil_bpm = s_acc_bpm;
    s_hasil_sis  = s_acc_sis;  s_hasil_dia = s_acc_dia;
    s_hasil_spo2 = s_acc_spo2;
  }

  /* Cek manual berhenti di sini: hasilnya sudah tersimpan di s_hasil_* untuk
   * layar, dan tidak ada satu byte pun yang masuk ring buffer. Perhatikan juga
   * bahwa cabang ini melewati logika "index 3 -> IDLE" di bawah -- cek manual
   * memang tidak pernah menyentuh mesin status sesi. */
  if (s_ukur_lokal) {
    s_ukur_lokal = false;
    Serial.printf("[ukur] cek manual selesai: bpm=%u spo2=%u glu=%u td=%u/%u "
                  "(tidak dikirim ke aplikasi), MAX30105 dimatikan\n",
                  s_acc_bpm, s_acc_spo2, s_acc_gula, s_acc_sis, s_acc_dia);
    kirim_status();
    return;
  }

  if (!ada) {
    /* Bila SEMUA metrik gagal, jangan kirim sampel -- catat UKUR_GAGAL dengan
     * payload index (dokumen 16). Sampel berisi lima nol tidak membawa
     * informasi apa pun selain "gagal", dan event-nya menyampaikan itu dengan
     * lebih jujur. */
    aw_ring_tambah_event(AW_EV_UKUR_GAGAL, s_ukur_sesi, s_ukur_index,
                         s_ukur_uptime_s);
    Serial.printf("[ukur] index %u GAGAL total, MAX30105 dimatikan\n",
                  (unsigned)s_ukur_index);
  } else {
    aw_ring_tambah_sampel(s_ukur_sesi, s_ukur_index, s_ukur_uptime_s,
                          s_acc_gula, s_acc_bpm, s_acc_sis, s_acc_dia,
                          s_acc_spo2);
    Serial.printf("[ukur] index %u %s: bpm=%u spo2=%u glu=%u td=%u/%u, "
                  "MAX30105 dimatikan\n",
                  (unsigned)s_ukur_index, lengkap ? "lengkap" : "sebagian",
                  s_acc_bpm, s_acc_spo2, s_acc_gula, s_acc_sis, s_acc_dia);
  }

  /* Sampel index 3 selesai -> kembali IDLE. Sampel yang belum di-ack tetap di
   * buffer; kepulangan ke IDLE tidak menunggu konfirmasi aplikasi, karena
   * seluruh siklus harus selesai walau HP tidak pernah tersambung sekali pun
   * (dokumen 12). */
  if (s_ukur_index == 3 && s_status == AW_SESI_RUNNING) {
    s_status = AW_SESI_IDLE;
    memset(s_sesi_id, 0, 16);
    s_t0_uptime = 0;
    s_index_selesai = 0;
    aw_ble_atur_interval(false);
  }
  kirim_status();
}

static void ukur_putar(void) {
  if (!s_ukur_aktif) return;

  ppg_data_t p;
  ppg_get(&p);

  uint32_t skrg = millis();
  bool kontak = (p.state == PPG_SETTLING || p.state == PPG_ACQUIRING ||
                 p.state == PPG_STABLE);
  if (kontak) s_ukur_kontak_ms = skrg;

  /* held=true berarti angkanya salinan hasil terakhir, bukan bacaan langsung.
   * Selama satu pengukuran hanya bacaan langsung yang boleh dipanen. */
  if (!p.held) {
    s_ukur_detak = p.beats;
    if (p.bpm_valid  && !s_acc_bpm)  s_acc_bpm  = klem_u8(p.bpm);
    if (p.spo2_valid && !s_acc_spo2) s_acc_spo2 = klem_u8(p.spo2);
    if (p.glu_valid  && !s_acc_gula) s_acc_gula = klem_u16(p.glucose);
    if (p.bp_valid   && !s_acc_sis) {
      s_acc_sis = klem_u8(p.sbp);
      s_acc_dia = klem_u8(p.dbp);
    }
  }

  /* Selesai hanya kalau KEEMPAT metrik sudah ada DAN detaknya sudah cukup.
   * Akumulator di atas tetap mengunci nilai pertama supaya keempat titik di
   * layar menyala satu per satu -- itu yang memperlihatkan kemajuan -- tetapi
   * nilai yang akhirnya dikirim diambil ulang di bawah, setelah 15 detak. */
  bool punya_semua = s_acc_bpm && s_acc_spo2 && s_acc_gula && s_acc_sis;
  bool cukup_detak = s_ukur_detak >= UKUR_MIN_DETAK;

  if (punya_semua && cukup_detak) {
    /* Angka final diambil dari rata-rata sesi, bukan dari nilai pertama yang
     * sempat terlihat belasan detik lalu. Rata-rata itulah alasan menunggu 15
     * detak: kalau hasilnya toh diambil dari window pertama, menunggu tidak
     * memperbaiki apa pun. Glukosa tidak punya rata-rata di statistik sesi,
     * jadi ia memakai nilai terkini yang sudah dihaluskan push_smoothed(). */
    if (p.stats_valid) {
      if (p.bpm_avg  > 0) s_acc_bpm  = klem_u8((float)p.bpm_avg);
      if (p.spo2_avg > 0) s_acc_spo2 = klem_u8((float)p.spo2_avg);
      if (p.sbp_avg  > 0) s_acc_sis  = klem_u8((float)p.sbp_avg);
      if (p.dbp_avg  > 0) s_acc_dia  = klem_u8((float)p.dbp_avg);
    }
    if (p.glu_valid) s_acc_gula = klem_u16(p.glucose);
    ukur_selesai(true);
    return;
  }

  /* Dua penjaga daya. Keduanya BUKAN batas kesabaran: selama jari menempel,
   * pengukuran menunggu selama apa pun. */
  if ((uint32_t)(skrg - s_ukur_kontak_ms) >= UKUR_TANPA_KONTAK_MS) {
    Serial.printf("[ukur] 90 dtk tanpa kulit menempel -- dihentikan "
                  "(detak %u/%u)\n", s_ukur_detak, UKUR_MIN_DETAK);
    ukur_selesai(false);
    return;
  }
  if ((uint32_t)(skrg - s_ukur_mulai_ms) >= UKUR_BATAS_KERAS_MS) {
    Serial.printf("[ukur] batas keras 5 menit tercapai -- dihentikan "
                  "(detak %u/%u)\n", s_ukur_detak, UKUR_MIN_DETAK);
    ukur_selesai(false);
  }
}

/* ================= Mesin status sesi (dokumen 12) ================= */
static void ke_idle(void) {
  s_status = AW_SESI_IDLE;
  memset(s_sesi_id, 0, 16);
  s_t0_uptime = 0;
  s_arm_uptime = 0;
  s_index_selesai = 0;
  aw_ble_atur_interval(false);
  kirim_status();
}

void jam_tekan_tombol(void) {
  if (s_status == AW_SESI_RUNNING) {
    /* Sesi sudah berjalan: t0 hanya boleh lahir sekali, dan menekan tombol lagi
     * bukan permintaan yang punya arti. Dipisahkan dari cabang di bawah supaya
     * pesannya benar -- "belum disiapkan aplikasi" justru salah di sini. */
    s_tolak = JAM_TOLAK_SESI_BERJALAN;
    return;
  }
  if (s_status != AW_SESI_ARMED) {
    /* Jangan "memperbaiki" ini menjadi selalu aktif: gating inilah yang
     * menjamin tidak ada sesi tanpa foto makanan (dokumen 12 & 14). */
    s_tolak = JAM_TOLAK_BELUM_ARM;
    Serial.println("[sesi] tombol ditekan di IDLE -- belum di-ARM aplikasi");
    return;
  }
  if (s_ukur_aktif) {
    s_tolak = JAM_TOLAK_SEDANG_UKUR;
    return;
  }
  if (baterai_kritis()) {
    s_tolak = JAM_TOLAK_BATERAI;
    Serial.println("[sesi] tombol ditolak: baterai kritis");
    return;
  }

  s_status    = AW_SESI_RUNNING;
  s_t0_uptime = aw_uptime_s();
  s_index_selesai = 0;

  /* Event tombol WAJIB masuk ring buffer seperti entri lain -- justru event
   * inilah yang paling sering terjadi saat HP tidak tersambung, dan ia adalah
   * sumber tunggal t0 (dokumen 7). */
  aw_ring_tambah_event(AW_EV_TOMBOL_SELESAI_MAKAN, s_sesi_id, 0, s_t0_uptime);

  aw_ble_atur_interval(true);
  s_index_selesai |= (1 << 1);
  ukur_mulai(1, s_sesi_id, false);
}

void jam_cek_manual(void) {
  /* Hanya DI LUAR sesi. Selama ARMED atau RUNNING, sensor adalah milik sesi:
   * cek manual di tengah sesi akan menyalakan MAX30105 pada saat pengukuran
   * terjadwal bisa jatuh tempo, dan menunda pengukuran itu berarti menggeser
   * jadwal yang seluruh analisis aplikasi bergantung padanya. Lebih buruk lagi,
   * hasilnya akan terlihat sama persis di layar dengan hasil sesi, sehingga
   * pengguna tidak punya cara membedakan angka yang terkirim dari yang tidak. */
  if (s_status != AW_SESI_IDLE) {
    s_tolak = JAM_TOLAK_SESI_AKTIF;
    Serial.println("[ukur] cek manual ditolak: sesi sedang berjalan");
    return;
  }
  if (s_ukur_aktif) {
    s_tolak = JAM_TOLAK_SEDANG_UKUR;
    return;
  }
  if (!ppg_present()) {
    s_tolak = JAM_TOLAK_SENSOR;
    Serial.println("[ukur] cek manual ditolak: MAX30105 tidak terdeteksi");
    return;
  }
  if (baterai_kritis()) {
    s_tolak = JAM_TOLAK_BATERAI;
    Serial.println("[ukur] cek manual ditolak: baterai kritis");
    return;
  }

  /* index 0 dan sesiId nol cuma pengisi: keduanya tidak pernah dipakai, karena
   * hasil cek manual tidak pernah menjadi entri. */
  ukur_mulai(0, SESI_NOL, true);
}

static void putar_sesi(void) {
  uint32_t skrg = aw_uptime_s();

  if (s_status == AW_SESI_ARMED) {
    /* Tanpa timeout ini, foto sarapan yang tombolnya tidak pernah ditekan akan
     * menyalakan tombol sampai malam. */
    if (skrg - s_arm_uptime >= AW_ARM_TIMEOUT_S) {
      aw_ring_tambah_event(AW_EV_SESI_KEDALUWARSA, s_sesi_id, 0, skrg);
      Serial.println("[sesi] ARM kedaluwarsa 4 jam");
      ke_idle();
    }
    return;
  }

  if (s_status != AW_SESI_RUNNING || s_ukur_aktif) return;

  /* Jadwal dihitung dari uptime_s ABSOLUT milik t0, tidak pernah dari "sisa
   * waktu" yang diakumulasikan sendiri -- sisa waktu akan hanyut setiap kali
   * ada penundaan (dokumen 12). */
  if (!(s_index_selesai & (1 << 2)) && skrg >= s_t0_uptime + AW_JADWAL_IDX2_S) {
    s_index_selesai |= (1 << 2);
    ukur_mulai(2, s_sesi_id, false);
    return;
  }
  if (!(s_index_selesai & (1 << 3)) && skrg >= s_t0_uptime + AW_JADWAL_IDX3_S) {
    s_index_selesai |= (1 << 3);
    ukur_mulai(3, s_sesi_id, false);
  }
}

/* ================= Perintah dari aplikasi (dokumen 5) ================= */
static bool sesi_cocok(const uint8_t *id) {
  return memcmp(id, s_sesi_id, 16) == 0;
}

/* Pemeriksaan yang sama untuk UKUR dan UKUR_SEKARANG. Mengembalikan 0 kalau
 * boleh mengukur, atau kode NAK kalau tidak. */
static uint8_t halangan_ukur(void) {
  if (s_ukur_aktif)    return AW_NAK_SEDANG_MENGUKUR;
  if (!ppg_present())  return AW_NAK_SENSOR_GAGAL;
  if (baterai_kritis()) return AW_NAK_BATERAI_RENDAH;
  return 0;
}

static void jalankan_perintah(const aw_perintah_t *p) {
  if (p->panjang < 1) return;
  uint8_t op = p->data[0];
  const uint8_t *arg = &p->data[1];
  uint8_t narg = (uint8_t)(p->panjang - 1);

  switch (op) {

    case AW_OP_ANCHOR_WAKTU: {
      if (narg < 6) { nak(AW_NAK_PAYLOAD_INVALID); return; }
      uint32_t epoch = aw_baca_u32(&arg[0]);
      uint16_t bid   = aw_baca_u16(&arg[4]);
      /* boot_id disertakan supaya jam bisa mem-NAK bila ia sempat reboot antara
       * handshake dan write -- tanpa itu, anchor bisa terpasang pada garis
       * waktu yang salah (dokumen 2.2). */
      if (bid != aw_boot_id()) { nak(AW_NAK_BOOT_ID_TIDAK_COCOK); return; }
      aw_simpan_anchor(aw_uptime_s(), epoch);
      ack(op);
      kirim_status();
      return;
    }

    case AW_OP_ARM_SESI: {
      if (narg < 16) { nak(AW_NAK_PAYLOAD_INVALID); return; }
      /* Sesi yang sudah RUNNING tidak bisa ditimpa; yang ARMED boleh, karena
       * hanya ada satu sesi ARMED pada satu waktu (dokumen 5). */
      if (s_status == AW_SESI_RUNNING) { nak(AW_NAK_SEDANG_MENGUKUR); return; }
      /* Cek manual yang sedang berjalan dibatalkan, bukan ditunggu: sesi punya
       * prioritas, dan hasil cek manual toh tidak pernah dikirim ke mana pun. */
      ukur_batal_lokal();
      memcpy(s_sesi_id, arg, 16);
      s_status = AW_SESI_ARMED;
      s_arm_uptime = aw_uptime_s();
      s_index_selesai = 0;
      s_t0_uptime = 0;
      ack(op);
      kirim_status();
      Serial.println("[sesi] ARMED -- tombol \"Selesai Makan\" aktif");
      return;
    }

    case AW_OP_BATAL_SESI: {
      if (narg < 16) { nak(AW_NAK_PAYLOAD_INVALID); return; }
      if (s_status == AW_SESI_IDLE || !sesi_cocok(arg)) {
        nak(AW_NAK_SESI_TAK_DIKENAL);
        return;
      }
      ack(op);
      ke_idle();
      return;
    }

    case AW_OP_UKUR: {
      if (narg < 17) { nak(AW_NAK_PAYLOAD_INVALID); return; }
      /* Jam hanya melayani UKUR dalam status ARMED atau RUNNING; permintaan
       * yang tiba selagi IDLE ditolak (dokumen 5). */
      if (s_status == AW_SESI_IDLE) { nak(AW_NAK_BELUM_ARM); return; }
      if (!sesi_cocok(arg))         { nak(AW_NAK_SESI_TAK_DIKENAL); return; }
      uint8_t index = arg[16];
      if (index > 3) { nak(AW_NAK_PAYLOAD_INVALID); return; }
      uint8_t halangan = halangan_ukur();
      if (halangan) { nak(halangan); return; }
      ack(op);
      s_index_selesai |= (uint8_t)(1 << index);
      ukur_mulai(index, s_sesi_id, false);
      return;
    }

    case AW_OP_UKUR_SEKARANG: {
      /* Pengukuran kalibrasi di luar sesi mana pun: boleh di status apa pun,
       * dijawab paket Sampel biasa dengan sesiId 16 byte nol dan index 0.
       * ACK-nya tetap dikirim lebih dulu (dokumen 5). */
      uint8_t halangan = halangan_ukur();
      if (halangan) { nak(halangan); return; }
      ack(op);
      ukur_mulai(0, SESI_NOL, false);
      return;
    }

    case AW_OP_SET_KALIBRASI: {
      if (narg < 4) { nak(AW_NAK_PAYLOAD_INVALID); return; }
      int16_t osis = (int16_t)aw_baca_u16(&arg[0]);
      int16_t odia = (int16_t)aw_baca_u16(&arg[2]);
      aw_kalibrasi_set(osis, odia);
      ack(op);
      kirim_status();
      return;
    }

    case AW_OP_SINKRON: {
      /* Semua yang masih ada di buffer memang belum di-ack, jadi semuanya
       * diantrekan ulang; byte seq dari aplikasi hanya informatif di v1.1. */
      (void)narg;
      aw_ring_antre_ulang_semua();
      ack(op);
      return;
    }

    case AW_OP_ACK_EVENT: {
      if (narg < 1) return;    /* tanpa balasan, termasuk saat payloadnya rusak */
      aw_ring_ack(arg[0]);
      /* SENGAJA tanpa ACK. Meng-ack sebuah ack adalah regresi tak berujung, dan
       * menunggu balasannya menambah satu perjalanan pulang-pergi untuk SETIAP
       * entri -- pada buffer 64 entri yang baru tersinkronisasi itu 64
       * perjalanan tambahan berturut-turut (dokumen 5). */
      kirim_status();
      return;
    }

    default:
      nak(AW_NAK_OPCODE_TAK_DIKENAL);
      return;
  }
}

static void putar_perintah_ble(void) {
  aw_perintah_t p;
  while (aw_ble_ambil_perintah(&p)) jalankan_perintah(&p);
}

/* ================= Pengirim buffer ================= */
/* Satu entri per giliran, berjeda. Mengosongkan 64 entri sekaligus akan
 * memenuhi antrean notifikasi NimBLE (dokumen 11). */
#define JEDA_KIRIM_MS 60

static void susun_flag(const aw_entri_t *e, uint8_t *flag) {
  /* Flag disusun SAAT KIRIM, bukan saat entri dibuat: keduanya bisa berubah
   * setelah entri masuk buffer. Anchor khususnya bisa datang berjam-jam setelah
   * entrinya dibuat, dan begitu ia datang, entri yang sama tidak lagi
   * "waktu tidak pasti" (dokumen 6). */
  *flag = 0;
  if (e->pernah_dikirim)               *flag |= AW_FLAG_DARI_BUFFER;
  if (!aw_boot_punya_anchor(e->boot_id)) *flag |= AW_FLAG_WAKTU_TIDAK_PASTI;
}

static bool kirim_entri(aw_entri_t *e) {
  uint8_t flag;
  susun_flag(e, &flag);

  if (e->jenis == 0) {                 /* sampel */
    uint8_t b[AW_LEN_SAMPEL];
    b[0] = e->seq;
    memcpy(&b[1], e->sesi_id, 16);
    b[17] = e->index_;
    b[18] = flag;
    aw_tulis_u16(&b[19], e->boot_id);
    aw_tulis_u32(&b[21], e->uptime_s);
    aw_tulis_u16(&b[25], e->gula);
    b[27] = e->bpm;
    b[28] = e->sis;
    b[29] = e->dia;
    b[30] = e->spo2;
    return aw_ble_kirim_sampel(b);
  }

  uint8_t b[AW_LEN_PERISTIWA];
  memset(b, 0, sizeof(b));
  b[0] = e->seq;
  b[1] = e->jenis;
  memcpy(&b[2], e->sesi_id, 16);
  aw_tulis_u16(&b[18], e->boot_id);
  aw_tulis_u32(&b[20], e->uptime_s);
  b[24] = flag;
  b[25] = e->index_;                   /* payload event */
  return aw_ble_kirim_peristiwa(b);
}

static void putar_pengirim(void) {
  static uint32_t terakhir_ms = 0;
  if (!aw_ble_siap_notifikasi()) return;
  if ((uint32_t)(millis() - terakhir_ms) < JEDA_KIRIM_MS) return;

  aw_entri_t *e = aw_ring_berikutnya();
  if (!e) return;
  if (!kirim_entri(e)) return;         /* gagal -> coba lagi giliran berikutnya */

  /* Entri TIDAK dihapus saat dikirim, hanya saat di-ACK_EVENT: aplikasi baru
   * meng-ack setelah entrinya tersimpan permanen di basis datanya. Duplikat
   * karena itu normal, dan jangan sekali-kali ditambahi anti-duplikat di
   * firmware (dokumen 1 & 11). */
  aw_ring_tandai_terkirim(e);
  terakhir_ms = millis();
}

/* ================= API publik ================= */
void jam_mulai(void) {
  /* Urutan ini tidak boleh dibalik (dokumen 13.4). */
  aw_store_begin();          /* NVS, boot_id++, muat ring tanpa dibersihkan */

  /* Reboot saat RUNNING mengakhiri sesi: uptime_s kembali nol dan t0 lama
   * tidak bisa dibandingkan lagi. Sampel yang terlanjur ada tetap di buffer
   * dengan boot_id lamanya, dan aplikasi menutup sesi itu sebagai tidak
   * lengkap. Jangan mencoba melanjutkan sesi lintas boot -- tanpa RTC itu
   * tidak mungkin, dan yang dihasilkan hanya data yang tampak sah tetapi
   * salah (dokumen 12 & 15). */
  s_status = AW_SESI_IDLE;
  memset(s_sesi_id, 0, 16);
  s_t0_uptime = 0;
  s_arm_uptime = 0;
  s_index_selesai = 0;

  aw_ble_begin();

  /* Event BOOT menunggu di buffer selama jam menyala tanpa HP; boot_id di
   * dalamnya sendiri yang memberi tahu aplikasi ada garis waktu baru. */
  aw_ring_tambah_event(AW_EV_BOOT, SESI_NOL, 0, aw_uptime_s());
}

void jam_putar(void) {
  putar_perintah_ble();      /* ambil dari antrean, jalankan opcode */
  putar_sesi();              /* timeout ARM, jadwal index 2 dan 3   */
  ukur_putar();              /* panen metrik, padamkan LED bila cukup */

  aw_ble_putar();            /* interval iklan cepat -> lambat        */

  if (aw_ble_ambil_flag_siap_baru()) {
    /* CCCD baru ditulis: BARU sekarang aman mulai menguras buffer. */
    aw_ring_antre_ulang_semua();
    aw_ble_atur_interval(s_status == AW_SESI_RUNNING);
    s_status_pernah = false;              /* paksa Status terkirim sekali */
    kirim_status();
  }
  if (aw_ring_ambil_flag_penuh()) {
    /* Dicatat dari LUAR fungsi penambah entri, supaya penambahan tidak
     * rekursif (dokumen 11 aturan 5). */
    aw_ring_tambah_event(AW_EV_BUFFER_PENUH, SESI_NOL, 0, aw_uptime_s());
  }

  putar_pengirim();
  aw_ring_simpan_jika_perlu();

  /* Baterai: ke karakteristik standar 0x2A19, hanya saat persennya berubah. */
  static int batt_terakhir = -1;
  if (battery_valid()) {
    int bp = battery_percent();
    if (bp != batt_terakhir) {
      batt_terakhir = bp;
      aw_ble_set_baterai((uint8_t)bp);
      kirim_status();
    }
  }
}

void jam_siap_mati(void) {
  /* Pengukuran yang sedang berjalan dibatalkan tanpa jejak, BUKAN diselesaikan
   * lewat ukur_selesai(false). Bedanya penting: kalau belum ada satu metrik pun
   * yang sempat terbaca, ukur_selesai() mencatat UKUR_GAGAL -- dan itu
   * berbohong. Sensornya tidak gagal; penggunanya yang menekan tombol mati.
   *
   * Sesi yang terpotong tidak perlu ditandai apa pun di sini: dokumen 12 sudah
   * menetapkan bahwa reboot saat RUNNING mengakhiri sesi, dan sampel yang
   * terlanjur ada tetap di buffer dengan boot_id lamanya untuk ditutup aplikasi
   * sebagai sesi tidak lengkap. */
  if (s_ukur_aktif) {
    s_ukur_aktif = false;
    s_ukur_lokal = false;
    Serial.println("[ukur] dibatalkan: jam dimatikan pengguna");
  }

  /* Tanpa syarat, bukan hanya saat sedang mengukur: fungsinya sendiri sudah
   * keluar lebih awal kalau sensor memang sudah mati, jadi ini menutup setiap
   * jalur yang mungkin meninggalkan LED menyala tanpa perlu tahu jalurnya. */
  ppg_set_enabled(false);

  /* Inilah alasan sebenarnya fungsi ini ada -- lihat aw_ring_simpan_sekarang()
   * di aw_store.h untuk apa yang hilang tanpa baris ini. */
  aw_ring_simpan_sekarang();
}

uint8_t  jam_status(void)            { return s_status; }
bool     jam_sedang_mengukur(void)   { return s_ukur_aktif; }
uint32_t jam_t0_uptime(void)         { return s_t0_uptime; }
uint32_t jam_uptime(void)            { return aw_uptime_s(); }
uint8_t  jam_tertunda(void)          { return aw_ring_tertunda(); }
bool     jam_terhubung(void)         { return aw_ble_terhubung(); }
bool     jam_siap_notifikasi(void)   { return aw_ble_siap_notifikasi(); }
bool     jam_ada_anchor(void)        { return aw_anchor_boot_ini(); }

uint8_t  jam_ukur_index(void)        { return s_ukur_index; }
uint16_t jam_ukur_detak(void)        { return s_ukur_detak; }
uint16_t jam_ukur_detak_perlu(void)  { return UKUR_MIN_DETAK; }

uint16_t jam_ukur_detik(void) {
  if (!s_ukur_aktif) return 0;
  return (uint16_t)((millis() - s_ukur_mulai_ms) / 1000);
}

/* Detak jantung dianggap "sudah" hanya kalau cacahannya cukup -- bukan sekadar
 * ada angkanya. Titik hijau di layar harus berarti hal yang sama dengan syarat
 * selesainya pengukuran; kalau tidak, keempat titik menyala hijau lalu
 * pengukuran tetap berjalan dan pengguna tidak tahu apa yang ditunggu. */
bool jam_ukur_punya_bpm(void) {
  return s_acc_bpm != 0 && s_ukur_detak >= UKUR_MIN_DETAK;
}
bool jam_ukur_punya_spo2(void)    { return s_acc_spo2 != 0; }
bool jam_ukur_punya_glukosa(void) { return s_acc_gula != 0; }
bool jam_ukur_punya_tensi(void)   { return s_acc_sis  != 0; }

void jam_snapshot(ppg_data_t *out) {
  /* Dasarnya tetap ppg_get(): state dan statistik sesi datang dari sana apa
   * adanya, karena keduanya hanya berarti selama sensor sungguhan mencacah. */
  ppg_get(out);

  uint16_t gula = s_ukur_aktif ? s_acc_gula : s_hasil_gula;
  uint8_t  bpm  = s_ukur_aktif ? s_acc_bpm  : s_hasil_bpm;
  uint8_t  sis  = s_ukur_aktif ? s_acc_sis  : s_hasil_sis;
  uint8_t  dia  = s_ukur_aktif ? s_acc_dia  : s_hasil_dia;
  uint8_t  spo2 = s_ukur_aktif ? s_acc_spo2 : s_hasil_spo2;
  if (!s_ukur_aktif && !s_hasil_ada) return;

  out->bpm_valid  = bpm  != 0;  out->bpm     = bpm;
  out->spo2_valid = spo2 != 0;  out->spo2    = spo2;
  out->glu_valid  = gula != 0;  out->glucose = gula;
  out->bp_valid   = sis  != 0;  out->sbp     = sis;  out->dbp = dia;
  out->held = !s_ukur_aktif;
}

uint8_t jam_umpan_balik_ditolak(void) {
  uint8_t f = s_tolak;
  s_tolak = JAM_TOLAK_TIDAK_ADA;
  return f;
}

bool jam_ukur_lokal(void)  { return s_ukur_lokal; }
bool jam_cek_manual_boleh(void) {
  return s_status == AW_SESI_IDLE && !s_ukur_aktif;
}
