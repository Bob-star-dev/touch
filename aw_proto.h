/*
 * AsaWatch -- protokol BLE v1.1, definisi kawat.
 *
 * Berkas ini adalah terjemahan langsung dari dokumen
 * "AsaWatch -- protokol BLE dan cara kerjanya" (v1.1), bagian 3-9. Dokumen itu
 * NORMATIF: sisi Flutter sudah selesai dan sudah diuji terhadap tabel-tabelnya,
 * jadi kalau kode di sini berbeda dari dokumen, kodenya yang salah. Jangan
 * mengubah satu pun angka di berkas ini tanpa mengubah dokumen dan versinya di
 * PR yang sama.
 *
 * Semua field multi-byte LITTLE-ENDIAN dan ditulis byte demi byte. Sengaja
 * tidak ada struct yang di-memcpy ke paket: paket kawat tidak boleh bergantung
 * pada padding maupun endianness kompiler.
 */
#pragma once

#include <stdint.h>

/* ---------------- Versi ---------------- */
#define AW_VERSI_MAYOR      1
#define AW_VERSI_MINOR      1
/* Dinaikkan manual tiap kali firmware dirilis. Aplikasi hanya melaporkannya,
 * tidak mengambil keputusan apa pun darinya. */
#define AW_FIRMWARE_BUILD   1

/* Kapasitas ring buffer, dilaporkan apa adanya di paket handshake. Kalau nilai
 * ini diubah, ubah juga AW_RING_KAP di aw_store.h -- keduanya harus sama, dan
 * yang dikirim ke aplikasi haruslah kapasitas yang sungguhan. */
#define AW_KAPASITAS_BUFFER 64

/* Kemampuan: bit0 gula darah, bit1 tekanan darah, bit2 SpO2, bit3 OTA.
 *
 * Ketiga bit pertama menyala karena jam ini memang menghasilkan ketiga angka
 * itu dari MAX30105. Perlu ditegaskan sekali di sini: gula darah dan tekanan
 * darah EKSPERIMENTAL dan tidak punya dasar fisiologis tervalidasi (lihat
 * peringatan panjang di ppg.h). Bit-nya tetap menyala karena artinya adalah
 * "jam mengirim metrik ini", bukan "angkanya layak dipakai secara klinis" --
 * dan mematikannya justru menyembunyikan angkanya dari UI aplikasi tanpa
 * menghilangkan satu pun risikonya.
 *
 * bit3 mati: OTA belum ada (dokumen 18). */
#define AW_KEMAMPUAN        0x07

/* ---------------- UUID (dibekukan, dokumen 3.1) ---------------- */
#define AW_UUID_SERVICE   "A5A70001-6B4C-4E2A-9D31-0F8C2E5A7B10"
#define AW_UUID_INFO      "A5A70002-6B4C-4E2A-9D31-0F8C2E5A7B10"  /* read + write */
#define AW_UUID_KONTROL   "A5A70003-6B4C-4E2A-9D31-0F8C2E5A7B10"  /* write w/ resp */
#define AW_UUID_PERISTIWA "A5A70004-6B4C-4E2A-9D31-0F8C2E5A7B10"  /* notify */
#define AW_UUID_SAMPEL    "A5A70005-6B4C-4E2A-9D31-0F8C2E5A7B10"  /* notify */
#define AW_UUID_STATUS    "A5A70006-6B4C-4E2A-9D31-0F8C2E5A7B10"  /* read + notify */

/* Baterai memakai layanan standar Bluetooth SIG, bukan karakteristik kustom. */
#define AW_UUID_SVC_BATT  ((uint16_t)0x180F)
#define AW_UUID_CHR_BATT  ((uint16_t)0x2A19)

/* Company id 0xFFFF = "untuk pengujian", dipakai dokumen 3.2 apa adanya. */
#define AW_COMPANY_ID     0xFFFF

/* ---------------- Ukuran paket ---------------- */
#define AW_LEN_INFO       20
#define AW_LEN_SAMPEL     31
#define AW_LEN_PERISTIWA  26
#define AW_LEN_STATUS     8

/* Perintah terpanjang: opcode + 16B sesiId + 1B index = 18. Dibulatkan ke 20
 * supaya perintah masa depan yang sedikit lebih panjang tidak diam-diam
 * terpotong di antrean. */
#define AW_MAKS_PERINTAH  20

/* ---------------- Opcode kontrol (dokumen 5) ---------------- */
typedef enum {
  AW_OP_ANCHOR_WAKTU  = 0x01,   /* 4B epoch UTC LE + 2B boot_id            */
  AW_OP_ARM_SESI      = 0x02,   /* 16B sesiId                              */
  AW_OP_BATAL_SESI    = 0x03,   /* 16B sesiId                              */
  AW_OP_UKUR          = 0x04,   /* 16B sesiId + 1B index                   */
  AW_OP_UKUR_SEKARANG = 0x05,   /* --                                      */
  AW_OP_SET_KALIBRASI = 0x06,   /* 2B offset sistolik + 2B diastolik int16 */
  AW_OP_SINKRON       = 0x07,   /* 1B seq terakhir yang diterima app       */
  AW_OP_ACK_EVENT     = 0x08,   /* 1B seq -- SATU-SATUNYA yang tidak dibalas */
} aw_opcode_t;

/* ---------------- Jenis peristiwa (dokumen 7) ---------------- */
typedef enum {
  AW_EV_TOMBOL_SELESAI_MAKAN = 0x01,  /* sumber tunggal t0                 */
  AW_EV_SESI_KEDALUWARSA     = 0x02,
  AW_EV_SESI_DIBATALKAN_JAM  = 0x03,
  AW_EV_UKUR_GAGAL           = 0x04,  /* payload = index sampel            */
  AW_EV_ACK                  = 0x05,  /* payload = opcode yang di-ack      */
  AW_EV_NAK                  = 0x06,  /* payload = kode error              */
  AW_EV_BUFFER_PENUH         = 0x07,
  AW_EV_BOOT                 = 0x08,
} aw_event_t;

/* ---------------- Kode error NAK (dokumen 9) ---------------- */
typedef enum {
  AW_NAK_OPCODE_TAK_DIKENAL = 0x01,
  AW_NAK_PAYLOAD_INVALID    = 0x02,
  AW_NAK_BELUM_ARM          = 0x03,
  AW_NAK_SESI_TAK_DIKENAL   = 0x04,
  AW_NAK_SEDANG_MENGUKUR    = 0x05,
  AW_NAK_BATERAI_RENDAH     = 0x06,
  AW_NAK_SENSOR_GAGAL       = 0x07,
  AW_NAK_KALIBRASI_KOSONG   = 0x08,
  AW_NAK_BOOT_ID_TIDAK_COCOK = 0x09,
} aw_nak_t;

/* ---------------- Flag ---------------- */
#define AW_FLAG_DARI_BUFFER        0x01   /* bit0 di paket Sampel & Peristiwa */
#define AW_FLAG_WAKTU_TIDAK_PASTI  0x02   /* bit1, sda                        */

/* Flag di paket Status (dokumen 8, offset 3). */
#define AW_ST_SEDANG_MENGUKUR      0x01
#define AW_ST_KALIBRASI_TERSIMPAN  0x02
#define AW_ST_BATERAI_KRITIS       0x04
#define AW_ST_ADA_ANCHOR           0x08

/* ---------------- Status sesi (dokumen 12) ---------------- */
typedef enum {
  AW_SESI_IDLE    = 0,
  AW_SESI_ARMED   = 1,
  AW_SESI_RUNNING = 2,
} aw_sesi_t;

/* ---------------- Jadwal (dokumen 12) ---------------- */
#define AW_ARM_TIMEOUT_S   (4UL * 3600UL)   /* ARMED -> kedaluwarsa           */
#define AW_JADWAL_IDX2_S   (1UL * 3600UL)   /* t0 + 1 jam                     */
#define AW_JADWAL_IDX3_S   (2UL * 3600UL)   /* t0 + 2 jam                     */

/* Baterai kritis, dipakai bit2 flag Status dan NAK 0x06 (dokumen 9 & 14). */
#define AW_BATERAI_KRITIS_PCT  10

/* ---------------- Penulis/pembaca little-endian ---------------- */
static inline void aw_tulis_u16(uint8_t *b, uint16_t v) {
  b[0] = (uint8_t)(v & 0xFF);
  b[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void aw_tulis_u32(uint8_t *b, uint32_t v) {
  b[0] = (uint8_t)(v & 0xFF);
  b[1] = (uint8_t)((v >> 8) & 0xFF);
  b[2] = (uint8_t)((v >> 16) & 0xFF);
  b[3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline uint16_t aw_baca_u16(const uint8_t *b) {
  return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

static inline uint32_t aw_baca_u32(const uint8_t *b) {
  return (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
         ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

/* Detik sejak boot. Diturunkan dari esp_timer (mikrodetik 64-bit), BUKAN dari
 * millis(): millis() berputar setiap 49,7 hari, dan jam tangan yang dibiarkan
 * menyala memang bisa mencapai itu. Nilai ini monoton naik dalam satu masa
 * hidup daya -- itulah seluruh jaminan yang dijanjikan dokumen 2. */
uint32_t aw_uptime_s(void);
