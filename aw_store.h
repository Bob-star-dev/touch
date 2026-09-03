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

/* ---- Titik yang ter-ARM (ARM_TITIK, dokumen 5 & 12, v1.3) ----
 * Menyalakan tombol ukur fisik untuk SATU titik. Ia menghuni NVS karena pola
 * pemakaian v1.3 adalah nyalakan-ukur-matikan: tombol yang lupa dirinya setiap
 * kali daya diputus tidak akan pernah berguna, sebab penyalaan di tengah sesi
 * justru keadaan yang paling lazim.
 *
 * Hanya SATU titik ter-ARM pada satu waktu; yang baru menimpa yang lama. Tidak
 * ada keadaan setengah jalan yang perlu dijaga.
 *
 * aw_titik_set() MEMBANDINGKAN DULU: ARM_TITIK berulang dengan isi yang sama
 * tidak menyentuh flash sama sekali (dokumen 11). Aplikasi boleh mengirim ulang
 * sesukanya tanpa mengikis flash. */
bool aw_titik_ada(void);
void aw_titik_get(uint8_t *sesi_id_out, uint8_t *index_out);
void aw_titik_set(const uint8_t *sesi_id, uint8_t index);
void aw_titik_hapus(void);

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

/* ---- Label uji (nomor 1-99 yang diatur MANUAL lewat konsol serial, "id N")
 * ----
 * BUKAN bagian protokol/kontrak kawat -- murni kenyamanan lab. Sempat
 * dihapus (MAC 6-hex dianggap cukup unik), tapi PENGUJIAN NYATA pada batch
 * 10 unit membuktikan itu salah: bukan cuma 2 byte terakhir MAC yang bisa
 * kebetulan sama pada beberapa unit sekaligus (3 dari 10, ditemukan
 * duluan), TERNYATA 3 byte terakhir pun masih bisa kebetulan sama (unit
 * "jam ke-4"). Dipasang lagi sebagai jalan pasti yang tidak bergantung pada
 * variasi MAC unit sama sekali. Kalau diatur (bukan 0), nama BLE jadi
 * "AsaWatch NN" alih-alih suffix hex MAC -- lihat aw_ble_begin(). 0 =
 * belum diatur, jatuh kembali ke suffix hex MAC seperti biasa. */
uint8_t aw_label_get(void);
void    aw_label_set(uint8_t n);

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
 * setelah ~3 detik tanpa perubahan baru (dokumen 11).
 *
 * SEJAK v1.3 JEDA INI ASIMETRIS, dan asimetrinya yang menentukan. Entri BARU
 * (sampel dan peristiwa) ditulis SEKETIKA; yang boleh ditunda hanya perubahan
 * yang kalau hilang cuma menghasilkan kiriman ulang -- ack, penanda terkirim,
 * antre ulang. Lihat aw_store.cpp untuk alasan lengkapnya; ringkasnya: pola
 * pemakaian v1.3 adalah nyalakan-ukur-MATIKAN, jadi daya yang putus di dalam
 * jendela 3 detik itu bukan kasus tepi melainkan yang diharapkan terjadi. */
void aw_ring_simpan_jika_perlu(void);

/* Tulis SEKARANG, melewati jeda tunda di atas. Hanya untuk satu keadaan:
 * beberapa milidetik sebelum daya sengaja diputus (tombol PWR ditekan).
 *
 * Jeda 3 detik itu ada supaya satu sesi tidak menghasilkan belasan penulisan
 * flash, dan itu benar untuk operasi normal. Tetapi ia juga berarti setiap
 * entri yang lahir dalam 3 detik terakhir masih hanya ada di RAM -- termasuk,
 * pada waktu yang paling buruk, sampel yang baru saja selesai diukur. Memutus
 * daya tanpa memanggil ini membuang entri itu tanpa jejak.
 *
 * Jangan dipakai di jalur normal: memanggilnya tiap perubahan akan
 * mengembalikan persis pola penulisan yang JEDA_SIMPAN_MS hindari. */
void aw_ring_simpan_sekarang(void);
