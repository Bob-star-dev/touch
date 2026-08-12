/*
 * AsaWatch -- lapisan BLE (NimBLE-Arduino 2.x).
 *
 * Dokumen bagian 3, 4, 10, 11, dan 13.1. Berkas ini SENGAJA tipis: ia tidak
 * tahu apa-apa soal sesi, jadwal, atau sensor. Tugasnya cuma tiga:
 *   1. mengiklankan diri dengan benar supaya jam bisa ditemukan sama sekali,
 *   2. menyajikan lima karakteristik kustom + baterai standar,
 *   3. memindahkan byte antara task host NimBLE dan konteks loop.
 *
 * ATURAN TASK (dokumen 13.1) -- alasan berkas ini ada sebagai lapisan terpisah:
 * seluruh callback BLE (onWrite, onRead, onSubscribe, onConnect, onDisconnect)
 * berjalan di task host NimBLE. Di dalamnya DILARANG menyentuh LVGL (gejalanya
 * corrupt rendering yang muncul sekali dalam sepuluh menit), NVS/flash
 * (gejalanya koneksi putus-putus di bawah beban), dan ring buffer. Yang
 * dilakukan onWrite di sini hanyalah menaruh byte mentahnya ke antrean
 * FreeRTOS; aw_jam.cpp yang mengambilnya di konteks loop.
 */
#pragma once

#include <stdint.h>
#include "aw_proto.h"

/* Satu perintah mentah dari karakteristik Kontrol. */
typedef struct {
  uint8_t panjang;
  uint8_t data[AW_MAKS_PERINTAH];
} aw_perintah_t;

/* Nyalakan radio, bangun GATT, mulai mengiklan. Panggil dari setup() setelah
 * aw_store_begin() -- paket handshake memuat boot_id, dan boot_id baru ada
 * setelah NVS dibuka. */
void aw_ble_begin(void);

/* Urusan berkala yang harus di konteks loop: mengganti interval iklan dari
 * cepat ke lambat setelah 60 detik pertama. Panggil dari jam_putar(). */
void aw_ble_putar(void);

/* ---- Perintah masuk ----
 * Ambil satu perintah dari antrean. false = antrean kosong. */
bool aw_ble_ambil_perintah(aw_perintah_t *out);

/* ---- Pengiriman ----
 * Ketiganya mengembalikan false kalau tidak ada yang berlangganan; pemanggil
 * memperlakukan itu sebagai "belum terkirim", bukan sebagai error. */
bool aw_ble_kirim_sampel(const uint8_t *paket);      /* AW_LEN_SAMPEL byte    */
bool aw_ble_kirim_peristiwa(const uint8_t *paket);   /* AW_LEN_PERISTIWA byte */
void aw_ble_set_status(const uint8_t *paket);        /* AW_LEN_STATUS byte    */
void aw_ble_set_baterai(uint8_t persen);

/* ---- Keadaan ----
 * Tiga keadaan, bukan dua (dokumen 14). Tersambung saja TIDAK cukup untuk mulai
 * mengirim: aplikasi menyambung, membaca Info, baru berlangganan, dan
 * notifikasi yang dikirim di sela itu hilang tanpa jejak sementara entrinya
 * terlanjur ditandai sudah dikirim. */
bool aw_ble_terhubung(void);
bool aw_ble_siap_notifikasi(void);   /* terhubung DAN dilanggani keduanya */

/* true SEKALI setiap kali langganan baru saja lengkap -- pemicu pengurasan
 * buffer (dokumen 11). */
bool aw_ble_ambil_flag_siap_baru(void);

/* Interval koneksi: rapat saat sesi berjalan, longgar saat idle (dokumen 10). */
void aw_ble_atur_interval(bool sesi_berjalan);

/* Serial 6 byte dari MAC efuse dan nama yang diiklankan ("AsaWatch 3F1A"). */
const uint8_t *aw_ble_serial(void);
const char    *aw_ble_nama(void);
