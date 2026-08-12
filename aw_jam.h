/*
 * AsaWatch -- mesin status sesi, rutin pengukuran, dan pengirim buffer.
 *
 * Dokumen bagian 5, 12, 13, dan 16. Ini "logika jam"-nya, dan sengaja tetap
 * setipis mungkin: verdict, kualitas respons, waktu pemulihan, tren, "lonjakan"
 * -- semuanya dihitung di aplikasi dan tidak pernah disimpan di sini. Jam
 * adalah sensor + buffer + pencacah (dokumen 1). Kalau sebuah fitur terasa
 * seperti "jam yang pintar", kemungkinan besar ia salah tempat.
 *
 * KONTEKS: seluruh berkas ini berjalan di task yang sama dengan
 * lv_timer_handler() -- yaitu loop(). Itu keputusan sadar dari dokumen 13.2:
 * dengan begitu callback tombol LVGL boleh memanggil jam_tekan_tombol()
 * langsung dan pembaca status UI boleh membaca variabel sesi langsung, tanpa
 * mutex dan tanpa antrean kedua. Yang menyeberang task tinggal satu, yaitu
 * antrean perintah BLE yang memang sudah ada di aw_ble.
 */
#pragma once

#include <stdint.h>
#include "aw_proto.h"
#include "ppg.h"

/* Urutannya tidak boleh dibalik: NVS -> boot_id naik -> muat ring (tanpa
 * dibersihkan) -> status sesi dipaksa IDLE -> BLE mulai -> catat event BOOT. */
void jam_mulai(void);

/* Dipanggil tiap iterasi loop, sebelum lv_timer_handler(). */
void jam_putar(void);

/* Dipanggil beberapa milidetik sebelum daya sengaja diputus (tombol PWR).
 * Memadamkan sensor dan memaksa ring buffer tersimpan ke NVS.
 *
 * Ia TIDAK mengirim apa pun lewat BLE dan TIDAK mencatat event apa pun:
 * protokol v1.1 tidak punya jenis peristiwa "dimatikan" (dokumen 18), dan
 * mengarang satu di sini akan membuat firmware menyimpang dari dokumen yang
 * sisi Flutter-nya sudah diuji. Aplikasi mengetahui jam mati dengan cara yang
 * memang sudah dirancang: boot_id yang naik pada koneksi berikutnya. */
void jam_siap_mati(void);

/* Dari lv_event_cb tombol "Selesai Makan". Di IDLE ia sengaja tidak
 * menghasilkan apa-apa selain umpan balik -- lihat jam_umpan_balik_ditolak(). */
void jam_tekan_tombol(void);

/* Dari lv_event_cb tombol "Cek manual". Menjalankan pengukuran yang hasilnya
 * HANYA muncul di layar jam: tidak ada entri sampel, tidak ada event, tidak ada
 * satu byte pun yang dikirim ke aplikasi. Hanya boleh di luar sesi -- lihat
 * alasannya di aw_jam.cpp. */
void jam_cek_manual(void);

/* true kalau tombol cek manual boleh ditekan sekarang (IDLE dan tidak sedang
 * mengukur). Dipakai UI untuk meredupkan tombolnya, bukan untuk menjaga
 * keamanan: jam_cek_manual() memeriksa ulang sendiri. */
bool jam_cek_manual_boleh(void);

/* true kalau pengukuran yang sedang berjalan adalah cek manual. Dipakai layar
 * pengukuran untuk mengatakan terus terang bahwa angka ini tidak dikirim. */
bool jam_ukur_lokal(void);

/* ---- Pembaca untuk UI. Semuanya murni RAM dan murah. ---- */
uint8_t  jam_status(void);             /* aw_sesi_t: 0 IDLE, 1 ARMED, 2 RUNNING */
bool     jam_sedang_mengukur(void);
uint32_t jam_t0_uptime(void);          /* 0 bila belum RUNNING                  */
uint32_t jam_uptime(void);
uint8_t  jam_tertunda(void);           /* entri belum di-ack                    */
bool     jam_terhubung(void);
bool     jam_siap_notifikasi(void);    /* terhubung DAN dilanggani              */
bool     jam_ada_anchor(void);

/* ---- Pembaca khusus layar "sedang mengukur" ----
 * Dengan sensor sungguhan satu pengukuran makan puluhan detik dan pengguna
 * harus diam; itu perlu tampilannya sendiri, bukan sekadar ikon (dokumen 14). */
uint8_t  jam_ukur_index(void);         /* 0..3, berlaku saat sedang mengukur    */
uint16_t jam_ukur_detik(void);         /* lama pengukuran berjalan              */
/* Detak yang sudah tercacah dan yang dibutuhkan. Inilah gerbang sebenarnya --
 * pengukuran tidak lagi dibatasi waktu, jadi kemajuan yang ditampilkan ke
 * pengguna harus angka ini, bukan detik yang berjalan. */
uint16_t jam_ukur_detak(void);
uint16_t jam_ukur_detak_perlu(void);
bool     jam_ukur_punya_bpm(void);
bool     jam_ukur_punya_spo2(void);
bool     jam_ukur_punya_glukosa(void);
bool     jam_ukur_punya_tensi(void);

/* Apa yang layar kesehatan tampilkan.
 *
 * Sejak MAX30105 hanya menyala selama pengukuran, membaca ppg_get() langsung
 * dari UI berarti keempat halaman detail menampilkan "--" hampir sepanjang
 * waktu -- benar, tetapi tidak berguna. Fungsi ini menambal itu: struktur yang
 * sama, tetapi kelima metriknya diisi dari pengukuran yang sedang berjalan,
 * atau dari hasil pengukuran terakhir kalau tidak ada yang berjalan. Statistik
 * sesi (chip min/avg/maks) sengaja TIDAK ditambal -- angka itu hanya berarti
 * selama sensor benar-benar mencacah.
 *
 * held=true saat angkanya salinan hasil terakhir, sama seperti arti field itu
 * di ppg.h. */
void jam_snapshot(ppg_data_t *out);

/* Alasan sebuah tekanan tombol ditolak. Umpan baliknya wajib ada dan harus
 * spesifik: pengguna harus tahu KENAPA tombolnya diam (dokumen 14), dan
 * "belum disiapkan aplikasi" adalah jawaban yang salah untuk tombol cek manual
 * yang terkunci karena sesi sedang berjalan. */
typedef enum {
  JAM_TOLAK_TIDAK_ADA = 0,
  JAM_TOLAK_BELUM_ARM,      /* "Selesai Makan" ditekan selagi IDLE      */
  JAM_TOLAK_SESI_AKTIF,     /* cek manual ditekan selagi ARMED/RUNNING  */
  JAM_TOLAK_SESI_BERJALAN,  /* tombol sesi ditekan lagi selagi RUNNING  */
  JAM_TOLAK_SEDANG_UKUR,
  JAM_TOLAK_BATERAI,
  JAM_TOLAK_SENSOR,
} jam_tolak_t;

/* Alasan penolakan terakhir, sekali baca lalu hangus. 0 = tidak ada. */
uint8_t jam_umpan_balik_ditolak(void);
