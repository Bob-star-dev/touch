/*
 * AsaWatch -- mesin status sesi, rutin pengukuran, dan pengirim buffer.
 *
 * Dokumen bagian 5, 12, 13, dan 16 -- protokol kawat v1.3. Ini "logika jam"-nya, dan sengaja tetap
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
 * protokol v1.3 tidak punya jenis peristiwa "dimatikan" (dokumen 18), dan
 * mengarang satu di sini akan membuat firmware menyimpang dari dokumen yang
 * sisi Flutter-nya sudah diuji. Aplikasi mengetahui jam mati dengan cara yang
 * memang sudah dirancang: boot_id yang naik pada koneksi berikutnya. */
void jam_siap_mati(void);

/* SATU tombol fisik, DUA makna (dokumen 12 poin 5 & 13.4, v1.3).
 *
 * Artinya ditentukan keadaan jam, dan PEMILIHANNYA ADA DI DALAM aw_jam, bukan di
 * UI:
 *
 *   ada ARM_TITIK  -> "Ukur": titik itu diukur sekarang, di status apa pun --
 *                     termasuk IDLE, yang justru paling lazim karena jam baru
 *                     saja dinyalakan kembali di tengah sesi
 *   ARMED          -> "Selesai Makan": mencatat t0, memancarkan peristiwanya
 *   RUNNING        -> ditolak dengan JAM_TOLAK_SESI_BERJALAN; t0 lahir sekali
 *   IDLE           -> umpan balik "belum di-ARM"
 *
 * UI TIDAK BOLEH memanggil dua fungsi berbeda berdasarkan tebakannya sendiri.
 * Keadaan bisa berubah antara UI membaca dan pengguna menekan -- sebuah
 * ARM_TITIK bisa tiba dalam jeda itu -- dan yang dihasilkan adalah pengukuran
 * untuk titik yang salah. UI hanya membaca jam_titik_armed() untuk MENULISKAN
 * LABEL.
 *
 * Pada jalur "Selesai Makan" ia TIDAK menyalakan sensor: rutinnya hanya mencatat
 * t0 dan memancarkan peristiwanya, dan index 1 diambil putar_sesi() pada iterasi
 * berikutnya. Itulah yang membuat tombol tidak pernah bisa gagal gara-gara
 * sensor sedang sibuk (dokumen 12.1). Pada jalur "Ukur" sebaliknya: sensor sibuk
 * membuat tekanan itu DIABAIKAN, tidak diantrekan, dan tombolnya tetap menyala.
 *
 * Opcode MULAI_SESI (0x09) TIDAK memanggil fungsi ini melainkan rutin "Selesai
 * Makan" yang sama persis di baliknya -- kalau ia memanggil dispatcher ini,
 * ARM_TITIK yang tersimpan akan membuat perintah dari aplikasi mengukur titik
 * alih-alih menetapkan t0. Bagi protokol, tombol fisik dan tombol dari aplikasi
 * tetap peristiwa yang sama dan tetap tidak bisa dibedakan.
 *
 * Lihat jam_umpan_balik_ditolak() untuk cara membaca penolakannya. */
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
/* 0 bila belum RUNNING. INFORMASIONAL SAJA sejak v1.3: t0 yang mengikat hidup
 * di aplikasi sebagai wall clock. Masih berguna untuk menampilkan "sesi dimulai
 * sekian menit lalu" DALAM MASA HIDUP DAYA INI; jangan membangun jadwal apa pun
 * di atasnya, dan jangan menampilkan hitung mundur ke titik berikutnya -- jam
 * tidak tahu kapan titik berikutnya jatuh tempo, dan biasanya sudah mati saat
 * itu tiba (dokumen 14). */
uint32_t jam_t0_uptime(void);
uint32_t jam_uptime(void);
uint8_t  jam_tertunda(void);           /* entri belum di-ack                    */
bool     jam_terhubung(void);
bool     jam_siap_notifikasi(void);    /* terhubung DAN dilanggani              */
bool     jam_ada_anchor(void);

/* ---- Titik yang ter-ARM (v1.3) ----
 * HANYA untuk menuliskan label tombol di layar (dokumen 12 poin 5 & 14). Jangan
 * dipakai untuk memilih fungsi mana yang dipanggil saat tombol ditekan -- lihat
 * jam_tekan_tombol(). jam_titik_index() hanya berarti bila jam_titik_armed(). */
bool     jam_titik_armed(void);
uint8_t  jam_titik_index(void);

/* ---- Pembaca khusus layar "sedang mengukur" ----
 * Dengan sensor sungguhan satu pengukuran makan puluhan detik dan pengguna
 * harus diam; itu perlu tampilannya sendiri, bukan sekadar ikon (dokumen 14). */
uint8_t  jam_ukur_index(void);         /* lebar byte penuh sejak v1.3           */
uint16_t jam_ukur_detik(void);         /* lama pengukuran berjalan              */
/* Detak yang sudah tercacah dan yang dibutuhkan. Inilah gerbang sebenarnya --
 * pengukuran tidak lagi dibatasi waktu, jadi kemajuan yang ditampilkan ke
 * pengguna harus angka ini, bukan detik yang berjalan. */
uint16_t jam_ukur_detak(void);
uint16_t jam_ukur_detak_perlu(void);

/* Kemajuan pengukuran 0..100, diambil dari syarat yang paling tertinggal
 * (detak, waktu minimum, kelengkapan metrik). Tidak pernah mundur selama satu
 * pengukuran, dan hanya mencapai 100 saat pengukurannya benar-benar tuntas --
 * itulah yang membuatnya bisa dipakai sebagai cincin tepi layar yang tidak boleh
 * bertemu titik awalnya sebelum selesai. 0 kalau tidak sedang mengukur. */
uint8_t jam_ukur_persen(void);

/* Perkiraan sisa waktu pengukuran dalam detik, dihitung ulang dari laju detak
 * yang sebenarnya terjadi -- memendek saat nadi mudah ditemukan, memanjang saat
 * susah. 0 kalau tidak sedang mengukur. Ini PERKIRAAN, bukan hitung mundur:
 * yang mengakhiri pengukuran tetap kecukupan data. */
uint16_t jam_ukur_sisa_detik(void);
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
