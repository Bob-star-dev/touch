/*
 * AsaWatch -- penyimpanan tak-hilang (NVS) + ring buffer.
 *
 * Dokumen bagian 2 dan 11. Isinya empat hal yang harus menyeberangi batas boot:
 *   boot_id        - naik tepat satu tiap boot, penanda garis waktu
 *   record anchor  - terjemahan uptime_s -> epoch, dipasang aplikasi
 *   daftar boot beranchor - menentukan flag waktu_tidak_pasti tiap entri
 *   ring buffer 64 entri  - satu-satunya hal yang menampung data pengguna
 *   offset kalibrasi tensi
 *
 * KENAPA DI FLASH, BUKAN RAM: buffer adalah satu-satunya hal yang menyeberangi
 * batas boot. Jam yang menyala sendirian sepanjang siang mengumpulkan sampel
 * tanpa HP di dekatnya; kalau buffernya di RAM, satu reset menghapus seluruh
 * hasil hari itu.
 *
 * RTC memory juga bukan tempatnya: ia selamat dari deep sleep tetapi tidak dari
 * baterai habis -- dan justru baterai habis itulah kejadian yang boot_id ada
 * untuk menandainya.
 *
 * KONTEKS: seluruh fungsi di sini menyentuh flash dan karena itu HANYA boleh
 * dipanggil dari konteks loop, tidak pernah dari callback NimBLE (dokumen 13.1;
 * menulis flash sambil BLE aktif bisa memblokir cukup lama untuk mengganggu
 * jadwal koneksi). Satu-satunya pengecualian dibuat eksplisit: aw_boot_id(),
 * aw_anchor_boot_ini(), dan aw_boot_punya_anchor() membaca salinan RAM, jadi
 * aman dipanggil dari onRead.
 */
#pragma once

#include <stdint.h>
#include "aw_proto.h"

/* Harus sama dengan AW_KAPASITAS_BUFFER yang dilaporkan ke aplikasi. */
#define AW_RING_KAP  AW_KAPASITAS_BUFFER

/* Satu entri ring buffer. Event dan sampel berbagi satu ruang seq dan satu
 * ruang slot -- dokumen 11. Untuk event, `index_` dipakai sebagai payload. */
typedef struct {
  uint8_t  dipakai;
  uint8_t  seq;             /* 1..255; 0 tidak pernah dipakai            */
  uint8_t  jenis;           /* 0 = sampel, selain itu aw_event_t         */
  uint8_t  index_;          /* sampel: index 0..3; event: payload        */
  uint8_t  sesi_id[16];
  uint16_t boot_id;
  uint32_t uptime_s;
  uint16_t gula;            /* mg/dL, 0 = gagal diukur                   */
  uint8_t  bpm;             /* bpm,   0 = gagal                          */
  uint8_t  sis;             /* mmHg,  0 = gagal                          */
  uint8_t  dia;             /* mmHg,  0 = gagal                          */
  uint8_t  spo2;            /* %,     0 = gagal                          */
  uint8_t  pernah_dikirim;  /* -> flag dariBuffer saat dikirim ulang     */
  uint8_t  perlu_dikirim;
  uint32_t ord;             /* urutan penambahan; yang terkecil = tertua */
} aw_entri_t;

/* Buka NVS, naikkan boot_id, muat ring + anchor + kalibrasi.
 * Ring TIDAK dibersihkan: entri lintas boot hidup berdampingan (dokumen 11
 * aturan 7), dan entri yang selamat langsung diantrekan ulang untuk dikirim. */
void aw_store_begin(void);

/* ---- Identitas garis waktu (semua murni RAM, aman dari callback BLE) ---- */
uint16_t aw_boot_id(void);
bool     aw_anchor_boot_ini(void);
bool     aw_boot_punya_anchor(uint16_t boot_id);

/* Pasang anchor untuk boot yang sedang berjalan. Menulis NVS -- konteks loop. */
void aw_simpan_anchor(uint32_t uptime_s, uint32_t epoch_s);

/* ---- Kalibrasi tekanan darah (offset, bukan nilai referensi) ---- */
bool aw_kalibrasi_ada(void);
void aw_kalibrasi_get(int16_t *offset_sis, int16_t *offset_dia);
void aw_kalibrasi_set(int16_t offset_sis, int16_t offset_dia);

/* ---- Ring buffer ----
 * Kedua fungsi penambah mengembalikan seq entri baru (1..255), atau 0 kalau
 * entri ditolak. Keduanya TIDAK mengirim event BUFFER_PENUH sendiri: itu
 * dilakukan pemanggil lewat aw_ring_ambil_flag_penuh(), supaya penambahan entri
 * tidak pernah rekursif (dokumen 11 aturan 5). */
uint8_t aw_ring_tambah_sampel(const uint8_t *sesi_id, uint8_t index,
                              uint32_t uptime_s, uint16_t gula, uint8_t bpm,
                              uint8_t sis, uint8_t dia, uint8_t spo2);
uint8_t aw_ring_tambah_event(uint8_t jenis, const uint8_t *sesi_id,
                             uint8_t payload, uint32_t uptime_s);

/* Entri tertua yang masih perlu dikirim, atau NULL. */
aw_entri_t *aw_ring_berikutnya(void);

/* Tandai entri sudah terkirim (bukan menghapusnya -- lihat aw_ring_ack). */
void aw_ring_tandai_terkirim(aw_entri_t *e);

/* SATU-SATUNYA jalan entri keluar dari buffer. Ack untuk seq yang sudah tidak
 * ada bukan error (dokumen 5). */
void aw_ring_ack(uint8_t seq);

/* Antrekan ulang semua entri yang belum di-ack -- dipakai opcode SINKRON dan
 * saat aplikasi baru selesai berlangganan. */
void aw_ring_antre_ulang_semua(void);

/* Jumlah entri belum di-ack, untuk field sampel_tertunda di paket Status. */
uint8_t aw_ring_tertunda(void);

/* true SEKALI setiap kali entri tertua terpaksa dibuang karena buffer penuh. */
bool aw_ring_ambil_flag_penuh(void);

/* Tulis flash yang ditunda: dipanggil tiap iterasi, benar-benar menulis hanya
 * setelah ~3 detik tanpa perubahan baru (dokumen 11). */
void aw_ring_simpan_jika_perlu(void);
