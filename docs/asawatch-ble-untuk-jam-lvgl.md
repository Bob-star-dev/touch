# AsaWatch — protokol BLE dan cara kerjanya

Spesifikasi lengkap lapisan BLE jam tangan AsaWatch, untuk diimplementasikan di firmware jam
berbasis LVGL yang sudah ada.

Berkas ini **berdiri sendiri**: seluruh tabel byte, opcode, dan kode error ada di sini. Tidak ada
rujukan ke berkas lain yang perlu dibuka. Salin ke repo firmware Anda (mis. `docs/`), lalu tunjuk
dari `CLAUDE.md` supaya agen menemukannya.

Berkas ini **normatif**. Sisi aplikasi Flutter sudah selesai dan sudah diuji terhadap tabel-tabel di
bawah; bila kode firmware dan dokumen ini berbeda, kodenya yang bug. Versi kawat: **v1.3**.

> **Yang berubah di v1.3, dan kenapa.** Satu penyebab, dan seluruh isi versi ini akibatnya: **jam
> tidak bertahan lebih dari ~50 menit menyala, sedangkan satu sesi berdurasi lebih dari dua jam.**
> Layarnya tidak bisa dipadamkan — sudah dicoba dan tidak berhasil — dan layar itulah yang
> mendominasi anggaran daya, bukan radio, jadi light sleep pun tidak menolong. Satu-satunya tuas yang
> tersisa adalah **memutus daya**, dan itu memang yang dilakukan penggunanya: nyalakan jam, ukur satu
> titik, matikan lagi.
>
> Jam karena itu **pasti** mati di tengah setiap sesi. Penjadwal di firmware tidak pernah bisa
> menyelesaikan tugasnya, jadi ia dicabut. **Yang menjadwalkan sekarang aplikasi; jam mengukur saat
> disuruh.**
>
> Lima perubahan kawat: `UKUR` (`0x04`) dilayani di ketiga status dan tidak lagi divalidasi terhadap
> sesi mana pun (§5); opcode baru `ARM_TITIK` (`0x0A`) menyalakan tombol ukur fisik untuk satu titik
> dan bertahan di NVS (§5); firmware berhenti menjadwalkan (§12); `t0` yang mengikat pindah ke
> aplikasi sebagai wall clock, meski bentuk paketnya tidak berubah (§5); dan aturan buang-sampel-
> beda-`boot_id` dicabut di sisi aplikasi.
>
> Ditambah tiga keputusan yang tidak menambah paket tetapi mengikat firmware dan UI: **satu tombol
> fisik dengan dua makna**, **`ARM_SESI` menghapus titik yang ter-ARM**, dan **dedup `(sesiId, index)`
> di RAM** (§12).
>
> Yang **tidak** berubah: jam tetap tidak punya wall clock, anchor tetap ditulis di setiap koneksi,
> buffer dan `SINKRON` apa adanya, ARM timeout 4 jam tetap ada, dan tombol fisik tetap ada — untuk
> `t0` maupun untuk tiap titik ukur.

Potongan kode di bawah adalah **NimBLE-Arduino ≥ 2.0** dan sudah berjalan di papan ESP32-C3. Ia
disertakan untuk bagian-bagian yang paling sering salah, bukan sebagai kerangka lengkap — struktur
berkas dan penamaan silakan mengikuti gaya repo Anda.

---

## Daftar isi

1. [Prinsip rancangan](#1-prinsip-rancangan)
2. [Waktu tanpa RTC](#2-waktu-tanpa-rtc)
3. [Identitas dan iklan](#3-identitas-dan-iklan)
4. [Handshake](#4-handshake)
5. [Karakteristik Kontrol dan opcode](#5-karakteristik-kontrol-dan-opcode)
6. [Karakteristik Sampel](#6-karakteristik-sampel)
7. [Karakteristik Peristiwa](#7-karakteristik-peristiwa)
8. [Karakteristik Status dan baterai](#8-karakteristik-status-dan-baterai)
9. [Kode error](#9-kode-error)
10. [Koneksi dan keamanan](#10-koneksi-dan-keamanan)
11. [Ring buffer dan ack](#11-ring-buffer-dan-ack)
12. [Mesin status sesi](#12-mesin-status-sesi)
13. [Struktur kode di firmware LVGL](#13-struktur-kode-di-firmware-lvgl)
14. [Apa yang ditampilkan UI](#14-apa-yang-ditampilkan-ui)
15. [Jebakan](#15-jebakan)
16. [Sensor](#16-sensor)
17. [Menguji dan checklist](#17-menguji-dan-checklist)
18. [Yang sengaja tidak ada di v1](#18-yang-sengaja-tidak-ada-di-v1)

---

## 1. Prinsip rancangan

Enam aturan yang menjelaskan hampir semua keputusan di dokumen ini.

**1. Jam adalah sensor + buffer + pencacah. Bukan tempat logika.**
Verdict, kualitas respons, waktu pemulihan, tren, "lonjakan" — semuanya dihitung di aplikasi dan
tidak pernah disimpan di jam. Firmware tidak perlu tahu satu pun konsep itu. Ini bukan sekadar
pembagian kerja: logika di firmware hanya bisa diperbaiki lewat OTA (yang belum ada), logika di
aplikasi lewat update biasa. Kalau sebuah fitur terasa seperti "jam yang pintar", kemungkinan besar
ia salah tempat — termasuk di UI: layar jam boleh **menampilkan** angka, tidak boleh menyimpulkan
apa pun darinya.

**2. Jam tidak pernah mengirim wall clock.** Ia hanya mengirim `uptime_s` dan `boot_id`. Aplikasi
yang menerjemahkannya ke waktu sungguhan (§2). Jam tanpa RTC yang berpura-pura tahu jam berapa akan
berbohong setiap kali baterainya habis.

**3. Jam adalah satu-satunya sumber `t0`.** Aplikasi tidak punya tombol yang setara, dan tidak boleh
menghitung `t0` dari waktu pesan tiba — pesannya bisa datang berjam-jam terlambat lewat buffer.

**4. Jam tidak menjadwalkan apa pun (v1.3).** Jadwal titik ukur hidup di aplikasi; jam mengukur saat
disuruh lewat `UKUR`, atau saat tombolnya ditekan setelah dinyalakan `ARM_TITIK`. Satu-satunya
pengukuran yang tersisa atas inisiatif jam adalah index 1, dan itu bukan jadwal melainkan akibat
langsung tombol yang ditekan (§12). Yang masih memakai `uptime_s` — dan tetap tidak boleh memakai
wall clock — adalah ARM timeout 4 jam, stempel `uptime_s` di tiap entri buffer, dan tidak lebih.

**5. Pengiriman at-least-once dengan ack eksplisit.** Duplikat adalah perilaku normal, bukan error.
Aplikasi sudah men-dedup dengan kunci `(sesiId, index)` — **jangan menambahkan anti-duplikat di
firmware.**

**6. Versi protokol dinegosiasikan sejak byte pertama.** App lama + firmware baru harus gagal dengan
pesan jelas, bukan salah membaca byte.

**Dua batasan perangkat keras membentuk seluruh dokumen.**

**Pertama: jam tidak punya RTC.** Ia tidak punya cara apa pun mengetahui jam berapa sekarang, dan
kehilangan seluruh pengetahuan waktu setiap kali daya putus.

**Kedua (v1.3): jam tidak bertahan semalam sesi.** ~50 menit menyala versus sesi lebih dari dua jam,
karena layarnya tidak bisa dipadamkan. Jam **pasti** dimatikan di tengah setiap sesi — keadaan yang
dirancang untuk terjadi, bukan kegagalan.

Dua konsekuensi yang paling mudah dilanggar, disebut di muka: **daya yang putus di tengah sesi adalah
jalur utama**, bukan kasus tepi; dan **apa pun yang harus selamat melewatinya wajib ada di NVS** —
hari ini itu `boot_id`, record anchor, offset kalibrasi, ring buffer, dan `ARM_TITIK`, tidak lebih.

---

## 2. Waktu tanpa RTC

Bagian ini kontra-intuitif sampai alasannya masuk. Baca sampai habis sebelum menulis baris pertama
yang menyentuh waktu.

Jam tidak menyimpan wall clock sama sekali. Ia punya dua hal:

- **`uptime_s`** — uint32, detik sejak boot, monoton naik dalam satu masa hidup daya.
- **`boot_id`** — uint16 di NVS, naik tepat satu setiap boot, tidak pernah direset.

Alternatifnya — jam menyimpan salinan wall clock di RAM — menghasilkan stempel waktu yang **tampak
sah tetapi salah** setiap kali daya sempat putus, dan kesalahan seperti itu jauh lebih berbahaya
daripada waktu yang jujur mengaku tidak diketahui.

### 2.1 `boot_id`

Disimpan di NVS, dinaikkan satu setiap boot. uint16 cukup: jam yang di-boot sekali sehari baru
berputar setelah ~180 tahun.

Fungsinya: menandai bahwa `uptime_s` dari dua entri boleh dibandingkan. Dua entri dengan `boot_id`
sama berada di garis waktu yang sama; `boot_id` berbeda berarti ada jeda daya yang **panjangnya
tidak diketahui siapa pun**.

**RTC memory bukan tempatnya.** Ia selamat dari deep sleep tetapi tidak dari baterai habis — dan
justru baterai habis itulah kejadian yang `boot_id` ada untuk menandainya. NVS.

**Sejak v1.3, `boot_id` naik beberapa kali dalam satu sesi makan, dan itu benar.** Ia bukan gejala,
bukan sesi yang rusak, dan bukan alasan membersihkan apa pun saat boot. Aplikasi tidak lagi membuang
sampel yang `boot_id`-nya berbeda dari titik lain di sesi yang sama; yang menentukan sampel milik
siapa adalah `sesiId` + `index`.

### 2.2 Anchor

Pada **setiap koneksi**, setelah handshake, aplikasi menulis `ANCHOR_WAKTU` (opcode `0x01`) berisi
epoch UTC saat itu dan `boot_id` yang baru saja dibacanya. Jam mencatat ke NVS:

```
anchor = (boot_id, uptime_s saat perintah diterima, epoch_s dari aplikasi)
```

`boot_id` disertakan supaya jam bisa mem-NAK bila ia sempat reboot antara handshake dan write —
tanpa itu, anchor bisa terpasang pada garis waktu yang salah.

Konversi di aplikasi, untuk entri apa pun:

```
epoch_entri = anchor.epoch_s + (entri.uptime_s − anchor.uptime_s)
```

**Rumus ini berlaku juga untuk entri yang terjadi sebelum anchor dipasang**, karena selisihnya boleh
negatif. Inilah yang menyelamatkan kasus paling umum: jam menyala sendirian sepanjang siang, tombol
ditekan, sampel terkumpul, lalu HP baru tersambung malam harinya. **Satu anchor di akhir cukup untuk
menerjemahkan seluruh isi buffer boot itu dengan akurat.**

Konsekuensi untuk firmware: jangan menahan apa pun menunggu anchor. Entri tetap dibuat, tetap masuk
buffer, tetap dikirim — anchor tidak mengubah satu byte pun di dalamnya.

### 2.3 Boot tanpa anchor

Bila satu boot penuh berlalu tanpa pernah sekali pun tersambung, entri dari boot itu tidak punya
anchor dan **tidak akan pernah punya** — jam tidak tahu berapa lama ia mati sebelum boot berikutnya.

Jam menandainya sendiri: entri dari boot yang belum pernah beranchor dikirim dengan flag bit1
`waktu_tidak_pasti`. Aplikasi memperlakukan sesi seperti itu secara khusus (waktunya null, tidak
masuk analisis) tetapi tetap menyimpan sampelnya — bentuk kurvanya tetap benar.

Karena satu buffer bisa memuat entri lintas boot, yang disimpan di NVS **bukan cuma anchor terakhir**
melainkan juga daftar `boot_id` yang pernah beranchor:

```c
void catat_boot_beranchor(uint16_t boot_id);
bool boot_punya_anchor(uint16_t boot_id);   // menentukan flag waktu_tidak_pasti tiap entri
```

### 2.4 Drift

Osilator, bukan RTC terkompensasi suhu: drift 50–100 ppm, sekitar 4–9 detik per hari. Untuk protokol
ini itu tidak berarti apa-apa — jadwal sesi berskala jam. Jangan mengoreksinya di v1.

---

## 3. Identitas dan iklan

### 3.1 UUID

Base UUID kustom, **sudah dibekukan**. Mengganti UUID setelah ada perangkat di tangan pengguna
berarti perangkat itu tidak akan pernah ditemukan lagi.

```
Service AsaWatch : A5A70001-6B4C-4E2A-9D31-0F8C2E5A7B10
```

| Karakteristik | UUID (suffix `-6B4C-4E2A-9D31-0F8C2E5A7B10`) | Properti | Arah |
|---|---|---|---|
| Info & Handshake | `A5A70002` | Read, Write | ↔ |
| Kontrol | `A5A70003` | Write with response | App → Jam |
| Peristiwa (Event) | `A5A70004` | Notify | Jam → App |
| Sampel | `A5A70005` | Notify | Jam → App |
| Status | `A5A70006` | Read, Notify | Jam → App |

Battery Level memakai standar `0x180F` / `0x2A19`, bukan karakteristik kustom.

### 3.2 Iklan — butir paling mudah dilewatkan

Paket iklan wajib memuat **Complete List of 128-bit Service UUIDs**. Scan response memuat
**Complete Local Name** (`AsaWatch<4 hex terakhir serial>`, tanpa spasi, mis. `AsaWatch3F1A`) dan
**Manufacturer Specific Data** (1 byte versi mayor, agar app bisa menandai firmware terlalu tua
sebelum menyambung).

> **Ketiganya tidak muat dalam satu paket iklan legacy.** Batas 31 byte; UUID 128-bit memakan 18
> byte, nama 15 byte, manufacturer data 5 byte — total 38. Nama dan manufacturer data **harus**
> pindah ke scan response; **UUID-nya yang tetap tinggal di paket iklan.**
>
> Urutan ini tidak bisa dibalik. Aplikasi memfilter service UUID **di level OS**, dan filter itu
> bekerja pada paket iklan. Jam yang menaruh UUID-nya di scan response **tidak akan pernah terlihat
> sama sekali** — bukan muncul lalu gagal, melainkan tidak muncul, dengan gejala yang di layar tidak
> bisa dibedakan dari jam yang mati.

```cpp
NimBLEAdvertisementData data_iklan;
data_iklan.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
data_iklan.setCompleteServices(NimBLEUUID(UUID_SERVICE));   // WAJIB di sini

NimBLEAdvertisementData data_scan;
data_scan.setName(nama);                                    // "AsaWatch3F1A"
std::vector<uint8_t> md = {0xFF, 0xFF, VERSI_MAYOR};        // 0xFFFF = company id uji
data_scan.setManufacturerData(md);

NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
adv->setAdvertisementData(data_iklan);
adv->setScanResponseData(data_scan);
adv->enableScanResponse(true);
```

Jam **selalu kembali beriklan** setelah putus, dan **tidak pernah memutus koneksi sendiri** saat
idle — setiap koneksi yang terbentuk adalah kesempatan memasang anchor.

### 3.3 Interval iklan — dibaca dari bond, bukan dari tombol

| Keadaan jam | Interval |
|---|---|
| Tidak punya bond (`NimBLEDevice::getNumBonds() == 0`) | 100 ms, **terus-menerus, tanpa batas waktu** |
| Punya bond, 30 detik pertama sesudah boot atau sesudah putus | 100 ms |
| Punya bond, dan ada entri baru yang menunggu diambil (5 menit) | 100 ms |
| Punya bond, sesudah itu | 1000 ms |

**Tidak ada "mode pairing", dan tidak ada tombol yang memicunya.** Rancangan awal memakai iklan
cepat selama 60 detik setelah tombol pairing ditekan; itu dibatalkan karena penggunanya lansia.
Tombol yang harus ditekan-tahan, dalam urutan yang harus dihafal, dengan tenggat 60 detik, adalah
satu langkah penuh yang bisa dihapus tanpa kehilangan apa pun. Jam yang belum pernah tersandingkan
tidak sedang mengerjakan hal lain, jadi tidak ada baterai yang perlu dihemat di sana.

Ini bukan pengaturan kenyamanan melainkan **penghapus satu langkah dari alur pengguna**: Android
hanya bisa menyambung pada jendela iklan, jadi interval lambat membuat setiap percobaan `connect`
memakan detik demi detik dan sering kehabisan waktu — di layar itu terbaca sebagai "pemasangan
pertama selalu sulit", bukan sebagai iklan yang lambat.

**Jendela 30 detik di baris tengah bukan sisa rancangan tombol pairing** dan tidak boleh dipakai
untuk menghidupkannya kembali. Ia melayani kasus lain: jam yang baru saja terputus dan **masih
membawa sampel di buffer-nya**. Ponsel yang kembali mendekat menemukannya dalam hitungan detik,
dengan biaya 30 detik iklan cepat per peristiwa putus.

**Baris ketiga dihidupkan kembali di v1.3, dan bentuknya yang menentukan.** Mekanisme "iklan cepat
selama ada entri menunggu" pernah ada di v1.1 lalu dicabut di v1.2: ia memakai 10 menit dan
dinyalakan ulang oleh entri mana pun yang masih ada, sehingga jam yang ditinggal seharian dengan
hasil yang tidak pernah diambil mengiklan 10x lebih sering sepanjang hari demi HP yang memang tidak
datang.

Yang berubah bukan penalarannya melainkan pola pemakaiannya. Sejak v1.3 jam **tidak** ditinggal
menyala — ia dinyalakan sebentar, diukur, lalu dimatikan lagi — jadi "baru menyala dan masih
memegang hasil" hampir selalu berarti seseorang sedang berdiri di depan HP-nya menunggu
sinkronisasi. Justru di situ 1000 ms paling merugikan: Android `connect()` memindai dengan duty
cycle rendah, dan pada pengiklan satu detik, timeout 15 detik sudah marjinal bahkan tanpa gangguan
apa pun.

**Gejalanya menyesatkan, dan itu sebabnya baris ini ditulis eksplisit:** jam TERLIHAT saat
memindai — iklan hanya perlu TX sekali tembak — lalu setiap percobaan menyambung timeout, karena
connect request menuntut jam MENDENGAR di jendela sesaat sesudah paket iklan. "Ditemukan tapi selalu
timeout" karena itu harus dibaca sebagai keluhan interval, bukan sebagai jam yang rusak.

Dua batas menjaganya tidak mengulang kesalahan v1.1: jendelanya **5 menit**, dan yang menyalakannya
ulang adalah entri **baru**, bukan entri yang sekadar masih ada. Jam dengan tujuh hasil basi yang
tidak bertambah karena itu gesit lima menit sesudah dinyalakan, lalu diam. Boot ikut tercakup tanpa
aturan tambahan: hitungan tertunda mulai dari nol, jadi ring buffer yang dimuat tidak kosong selalu
terbaca sebagai kenaikan.

**Jam yang bond-nya dihapus harus kembali mengiklan cepat** — dalam segala hal ia kembali menjadi
jam yang belum pernah dipasangkan. Kalau keputusannya dibaca dari `getNumBonds()` setiap kali,
ini terjadi dengan sendirinya; yang perlu dipastikan hanya rutin penghapus bond memicu evaluasi
ulang, bukan menunggu boot berikutnya.

Evaluasinya di loop, **bukan di dalam callback NimBLE** — menata ulang iklan dari task host saat
`onDisconnect` berarti menyentuh stack dari dalam stack. Yang boleh dilakukan `onDisconnect` cuma
menyalakan bendera:

```cpp
// onDisconnect: startAdvertising() lalu nyalakan bendera. Jendelanya dipasang di loop.
baru_putus = true;

// di loop, tiap iterasi:
void perbarui_interval_iklan(bool paksa) {
  bool ada_bond = NimBLEDevice::getNumBonds() > 0;
  uint16_t mau = (!ada_bond || dalam_jendela_cepat()) ? 160 : 1600;   // satuan 0,625 ms
  if (!paksa && mau == interval_sekarang) return;                     // jangan restart iklan tiap loop
  adv->stop();
  adv->setMinInterval(mau);
  adv->setMaxInterval(mau);
  if (!terhubung) adv->start();
  interval_sekarang = mau;
}
```

`paksa` hanya perlu saat jumlah bond baru saja berubah (mis. sesudah menghapus bonding), karena saat
itu nilai lamanya sudah tidak bisa dipercaya.

Saat menguji: sediakan cara melihat **alasan** interval yang sedang berlaku, bukan cuma angkanya —
`100 ms (tanpa bond)` / `100 ms (jendela 21 dtk lagi)` / `1000 ms (ber-bond, hemat)`. Tanpa itu,
"1000 ms" saat menguji pemasangan pertama tidak bisa dibedakan dari bug.

### 3.4 Serial perangkat

6 byte, diturunkan dari MAC efuse. Stabil lintas boot dan lintas platform — tidak seperti id
perangkat dari OS (MAC di Android, UUID di iOS).

---

## 4. Handshake

Karakteristik **Info** (`A5A70002`) dibaca aplikasi segera setelah koneksi terbentuk, sebelum
operasi lain apa pun.

Read → **20 byte**:

| Offset | Ukuran | Field | Catatan |
|---|---|---|---|
| 0 | 1 | `versi_mayor` | `1` |
| 1 | 1 | `versi_minor` | `3` |
| 2 | 6 | `serial` | Identitas stabil lintas platform |
| 8 | 2 | `firmware_build` | uint16 LE |
| 10 | 1 | `kapasitas_buffer` | Jumlah entri ring buffer — `64` |
| 11 | 1 | `kemampuan` | bit0 gula darah, bit1 tekanan darah, bit2 SpO2, bit3 OTA |
| 12 | 2 | `boot_id` | uint16 LE |
| 14 | 4 | `uptime_s` | uint32 LE |
| 18 | 1 | `flag` | bit0: boot ini sudah punya anchor tersimpan |
| 19 | 1 | *reserved* | Nol |

Aturan versi di sisi aplikasi:

| Kondisi | Perilaku aplikasi |
|---|---|
| `versi_mayor` > yang didukung app | Tolak. "Jam perlu aplikasi versi lebih baru." |
| `versi_mayor` < yang didukung app | Tolak. "Firmware jam perlu diperbarui." |
| `versi_minor` berbeda | Lanjut; field tak dikenal diabaikan |

**`kemampuan` harus jujur.** Metrik yang bitnya 0 disembunyikan aplikasi dari UI — dan itu jauh
lebih baik daripada menampilkan "—" seolah pengukurannya gagal.

Paket disusun **tepat saat dibaca**, supaya `uptime_s`-nya segar:

```cpp
// Dipanggil dari task host NimBLE. WAJIB murni RAM: tidak boleh menyentuh NVS
// maupun ring buffer (lihat §13).
static void susun_info(uint8_t* b) {
  memset(b, 0, 20);
  b[0] = VERSI_MAYOR;
  b[1] = VERSI_MINOR;
  memcpy(&b[2], serial_dev, 6);
  tulis_u16(&b[8], FIRMWARE_BUILD);
  b[10] = 64;                 // kapasitas buffer
  b[11] = KEMAMPUAN;
  tulis_u16(&b[12], boot_id);
  tulis_u32(&b[14], uptime_s());
  b[18] = anchor_boot_ini ? 0x01 : 0x00;   // disalin ke RAM saat boot, bukan dibaca dari NVS
  b[19] = 0;
}
```

Semua field multi-byte **little-endian**, ditulis byte demi byte — jangan `memcpy` dari struct;
paket kawat tidak boleh bergantung pada padding maupun endianness kompiler.

```cpp
inline void tulis_u16(uint8_t* b, uint16_t v) { b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF; }
inline void tulis_u32(uint8_t* b, uint32_t v) {
  b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF; b[2] = (v >> 16) & 0xFF; b[3] = (v >> 24) & 0xFF;
}
inline uint16_t baca_u16(const uint8_t* b) { return b[0] | (b[1] << 8); }
inline uint32_t baca_u32(const uint8_t* b) {
  return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}
```

---

## 5. Karakteristik Kontrol dan opcode

`A5A70003`, Write with response. Byte 0 = opcode, sisanya payload.

Jam membalas lewat **karakteristik Peristiwa** dengan `ACK` atau `NAK` yang membawa opcode asal —
bukan lewat write response, karena beberapa perintah butuh waktu.

| Opcode | Nama | Payload |
|---|---|---|
| `0x01` | `ANCHOR_WAKTU` | 4B epoch UTC LE + 2B `boot_id` |
| `0x02` | `ARM_SESI` | 16B sesiId (UUID biner) |
| `0x03` | `BATAL_SESI` | 16B sesiId |
| `0x04` | `UKUR` | 16B sesiId + 1B index |
| `0x05` | `UKUR_SEKARANG` | — |
| `0x06` | `SET_KALIBRASI` | 2B offset sistolik + 2B offset diastolik (int16 LE) |
| `0x07` | `SINKRON` | 1B seq terakhir yang sudah diterima app |
| `0x08` | `ACK_EVENT` | 1B seq |
| `0x09` | `MULAI_SESI` | 16B sesiId |
| `0x0A` | `ARM_TITIK` | 16B sesiId + 1B index |

**Setiap opcode dibalas `ACK`/`NAK` — kecuali `ACK_EVENT`, yang tidak dibalas sama sekali.**
Meng-ack sebuah ack adalah regresi tak berujung, dan menunggu balasannya menambah satu perjalanan
pulang-pergi untuk **setiap** entri: pada buffer 64 entri yang baru tersinkronisasi, itu 64
perjalanan tambahan berturut-turut. Jaminannya tidak hilang — `ACK_EVENT` yang lenyap berarti jam
mengirim entrinya lagi, dan duplikat memang normal.

Perintah yang tidak dibalas membuat aplikasi mengulanginya tiga kali lalu menyerah, dan di layar itu
terlihat seperti jam yang tidak merespons.

### Catatan per opcode

**`ANCHOR_WAKTU`** — datang pada setiap koneksi, sebelum perintah lain. Murah dan idempoten. Bila
`boot_id` di payload tidak cocok dengan `boot_id` jam sekarang, **NAK `0x09`** (jam sempat reboot
antara handshake dan write). Boleh masuk di status sesi mana pun; ia tidak menyentuh mesin status.

**`ARM_SESI`** — satu-satunya hal yang menyalakan tombol "Selesai Makan" di jam. Selama belum
di-ARM, menekan tombol tidak menghasilkan apa-apa. Inilah mekanisme yang menjamin **tidak ada sesi
tanpa foto makanan.** `ARM_SESI` menimpa sesi ARMED sebelumnya yang belum ditekan; hanya satu sesi
ARMED pada satu waktu. Sesi yang sudah RUNNING tidak bisa ditimpa (NAK `0x05`).

**`MULAI_SESI`** (v1.2) — **tombol "Selesai Makan" jam yang ditekan dari aplikasi.** Payloadnya
`sesiId` saja, dan **ketiadaan waktu di dalamnya adalah seluruh isi perintah ini**: jam membaca
`uptime_s`-nya sendiri saat perintah tiba, pindah ARMED → RUNNING, mengukur index 1, lalu mengirim
`TOMBOL_SELESAI_MAKAN` — persis seperti kalau tombol fisiknya yang ditekan. Sejak v1.3 tidak ada
lagi jadwal `+1 jam`/`+2 jam` yang dipasang di sini, atau di mana pun di firmware.

Kenapa bukan aplikasi yang mengirim `t0`-nya sendiri, padahal sejak v1.3 `t0` yang mengikat justru
dipegang aplikasi sebagai wall clock: **payload tanpa waktu adalah yang menjaga jam tidak punya dua
sumber waktu.** Aplikasi menstempel `t0` saat `TOMBOL_SELESAI_MAKAN` **tiba**; perintahnya sendiri
tetap hanya berarti "tekan tombolmu". Kalau suatu saat ada yang tergoda menaruh epoch di payload ini,
firmware mendadak punya dua sumber waktu yang salah satunya pasti bohong, dan seluruh §2 runtuh.

**Tombol fisik di jam tetap ada dan tetap yang utama.** Ia satu-satunya yang bekerja saat ponsel
jauh, mati, atau tidak dipegang — dan itu justru keadaan yang paling lazim saat orang sedang makan.
`MULAI_SESI` melayani keadaan sebaliknya: ponsel di tangan, jam di pergelangan.

- **Hanya dilayani dalam ARMED**, dengan `sesiId` yang sama. IDLE → NAK `0x03`, sesi lain → NAK
  `0x04`. Ini yang menjaga "tidak ada sesi tanpa foto makanan" tetap berlaku untuk tombol baru ini.
  Penjaga ini **tetap ada di v1.3** — perhatikan ia tidak ikut dicabut bersama penjaga `UKUR` di
  bawah, dan alasannya beda: `t0` hanya boleh ada satu per sesi, sedangkan titik ukur boleh datang
  kapan saja dari penyalaan daya mana pun.
- **Tidak pernah ditolak karena sensor sedang sibuk** — satu-satunya opcode pengukuran-adjacent yang
  tidak boleh menjawab NAK `0x05`. Lihat §12.1; sebabnya `t0` adalah **stempel waktu, bukan
  pengukuran**.
- **Idempoten.** `MULAI_SESI` untuk sesi yang **sudah RUNNING** cukup di-`ACK` lalu diabaikan —
  jangan menetapkan `t0` kedua, jangan mengirim `TOMBOL_SELESAI_MAKAN` lagi. ACK bisa hilang di
  udara dan aplikasi akan mengulang sampai tiga kali; dua `t0` untuk satu sesi adalah kerusakan yang
  tidak bisa diperbaiki siapa pun sesudahnya.
- **Peristiwanya tidak boleh dibedakan dari tombol fisik.** Tidak ada flag "dari aplikasi", dan tidak
  boleh ada: bagi seluruh sisa dokumen ini keduanya adalah peristiwa yang sama. Di kode, panggil
  fungsi tombol yang sudah ada — jangan menyalin jalurnya. Jalur kembar adalah cara paling pasti
  membuat keduanya lambat laun berbeda.

**`UKUR`** — **satu-satunya cara sebuah titik sesi terukur** (v1.3). Sampai v1.2 ia hanya dipakai
untuk baseline (index 0) saat shutter kamera ditekan; sejak jam berhenti menjadwalkan (§12),
**setiap** titik datang lewat opcode ini. **Bentuk paketnya tidak berubah sama sekali** — ia sejak
awal berarti "ukur titik index N milik sesi S, sekarang"; yang berubah hanya siapa yang memicunya.

Tiga perubahan perilaku, dan semuanya melonggarkan:

- **Dilayani di IDLE, ARMED, maupun RUNNING.** Penjaga v1.2 (IDLE → NAK `0x03`) **dicabut**.
- **`sesiId` tidak lagi dicocokkan.** Penjaga v1.2 (sesi lain → NAK `0x04`) **dicabut**. Jam
  meneruskan `(sesiId, index)` dari payload apa adanya ke paket Sampel, tanpa memvalidasinya.
- **Batas `index > 3` dilonggarkan** ke lebar byte penuh, karena jadwal titik kini data sisi
  aplikasi.

**Mencabut keduanya bukan kelalaian, dan jangan dikembalikan.** Keduanya benar di v1.2 dan salah di
v1.3: setelah jam dimatikan di antara titik ukur, ia **selalu** ada di IDLE dan **selalu** sudah lupa
`sesiId`-nya, jadi setiap titik sesudah yang pertama akan di-NAK. Yang memvalidasi sesi adalah
aplikasi — satu-satunya yang tahu sesi apa yang sedang berjalan.

**`UKUR` idempoten per `(sesiId, index)` dalam satu masa hidup daya.** Permintaan kedua untuk
pasangan yang sama cukup di-`ACK` lalu diabaikan; alasannya sama dengan `MULAI_SESI` — dua tombol
bisa memicu perintah yang sama dan ACK bisa hilang di udara. Aplikasi sudah men-dedup dengan kunci
yang sama, jadi ini murni penghematan sensor dan baterai. Bentuk lengkapnya, termasuk kenapa kuncinya
di RAM dan bukan NVS, ada di §12.

Perhatikan ini menuntut firmware **tetap menyimpan `sesiId`**, meski ia berhenti memvalidasinya. Yang
dicabut adalah `sesiId` sebagai **izin**; yang tersisa adalah `sesiId` sebagai **kunci dedup**.

**`ARM_TITIK`** (`0x0A`, v1.3) — **menyalakan tombol ukur fisik di jam untuk satu titik tertentu.**
Payloadnya `sesiId` + `index` saja; **tidak ada waktu di dalamnya**, dan itu disengaja (lihat kotak
di bawah). Polanya mengikuti `ARM_SESI` persis: sebelum di-ARM, menekan tombolnya tidak menghasilkan
apa-apa.

Ia ada karena jam yang baru dinyalakan **tidak tahu ia sedang mengukur titik yang mana** — pengetahuan
yang tidak bisa ia simpulkan sendiri setelah dimatikan.

- **Disimpan ke NVS begitu diterima**, sehingga tombolnya **tetap menyala setelah jam dimatikan dan
  dihidupkan lagi.** Itu justru keadaan yang paling sering. Tombol yang lupa dirinya setiap kali daya
  diputus tidak akan pernah berguna.
- **Padam sendiri setelah satu kali tekan yang berhasil**, dan catatan NVS-nya dihapus. Tekanan kedua
  tidak menghasilkan apa-apa sampai aplikasi meng-ARM titik berikutnya. Ini yang mencegah satu
  tekanan tidak sengaja mengisi titik yang belum jatuh tempo — titik yang sudah terisi tidak bisa
  diperbaiki.
- **Pengukuran yang gagal meninggalkan tombolnya tetap menyala**, sebagai percobaan ulang. Padam
  mengikuti keberhasilan, bukan penekanan.
- **`ARM_TITIK` baru menimpa yang lama**, di RAM maupun NVS. Hanya satu titik ter-ARM pada satu waktu;
  tidak ada keadaan setengah jalan yang perlu dijaga.
- **Sesi yang BERAKHIR memadamkannya**, bila titik yang ter-ARM memang milik sesi itu: `BATAL_SESI`,
  dan ARM yang kedaluwarsa 4 jam. Keduanya jalur yang sama di firmware (`ke_idle()`), dan
  perbandingan `sesiId`-nya harus terjadi sebelum `sesiId` sesi dihapus.

  Ini **tidak** bertabrakan dengan "ARM_TITIK hidup lebih lama daripada mesin status" (§12 poin 5).
  Aturan itu tentang IDLE yang lahir dari **daya diputus**, dan jalur itu tidak lewat sini sama
  sekali — ia memuat titiknya dari NVS saat boot. Yang lewat sini hanya sesi yang benar-benar
  selesai, dan titik milik sesi yang selesai tidak akan pernah bisa terisi: aplikasi membuang sampel
  yang `sesiId`-nya tidak dikenalnya. Tombol yang tetap menyala untuknya adalah kebohongan di layar
  jam, jenis yang sama persis dengan yang dilarang dua butir di bawah, dan dengan akibat yang sama —
  pengguna menekannya, satu siklus sensor terbuang di perangkat yang umur nyalanya ~50 menit, dan
  tidak ada yang sampai ke mana pun.

  §12 poin 5 sudah mengandaikan aturan ini. Kalimatnya soal titik basi — "`BATAL_SESI` penutupnya
  tidak sampai karena jam sedang mati" — hanya masuk akal kalau `BATAL_SESI` yang **sampai** memang
  memadamkannya.
- **`UKUR` untuk titik yang sedang ter-ARM memadamkan tombolnya dan menghapus catatan NVS-nya**,
  sesudah pengukurannya tuntas. Jam tahu keduanya titik yang sama karena `ARM_TITIK` menyimpan
  `(sesiId, index)` dan `UKUR` membawa `(sesiId, index)`.

  Tanpa ini, jalur yang paling lazim meninggalkan tombol menyala untuk titik yang sudah terisi:
  pengguna mengukur dari aplikasi, dan tombol di jam tetap menyala. Dua akibatnya, dan yang kedua
  lebih penting: pengguna yang tidak yakin pengukurannya berhasil akan menekan tombol itu juga —
  wajar, apalagi bagi pengguna lansia — dan satu siklus sensor terbuang di perangkat yang umur
  nyalanya ~50 menit; dan **tombol yang menyala tetapi tidak menghasilkan apa-apa adalah kebohongan
  di layar jam.** Menyala berarti "ada yang menunggu ditekan"; setelah titiknya terukur, tidak ada.
- **Dedup dipakai di kedua jalur**, handler `UKUR` maupun penekanan tombol. Menaruhnya hanya di
  handler `UKUR` menyisakan jalur tombol yang melewatinya sama sekali.

> **Kenapa `ARM_TITIK` tidak membawa penundaan.** Rancangan pertamanya membawa `2B detik_tunda`: jam
> menyalakan tombolnya sendiri setelah sekian detik. Dibuang sebelum implementasi, dan bersamanya
> lima mekanisme (tabel matang/belum-matang, aturan tulis-NVS, pematangan di loop, pemulihan saat
> boot, dan seluruh asimetri pengetahuan yang menyertainya).
>
> Sebabnya satu kalimat: **penundaan tidak pernah selamat melewati mati-hidup, dan mati-hidup adalah
> keadaan normal di v1.3.** Jam tidak bisa tahu berapa lama ia mati (§2), jadi penundaan yang belum
> matang harus dibuang saat boot — dan aplikasi harus mengirim ulang begitu jam menyala. Penundaan
> hanya berguna bila jam **tetap menyala** sementara ponselnya yang pergi, dan pada jam yang tahan
> ~50 menit sementara jarak antar titik satu jam, itu tidak pernah terjadi.
>
> Karena `ARM_TITIK` sama-sama butuh koneksi seperti `UKUR`, **aplikasi mengirimnya saat titiknya
> jatuh tempo**, bukan di awal sesi. Penjaga "jangan diukur terlalu cepat" pindah sepenuhnya ke sisi
> aplikasi, tempat ia cuma keputusan penjadwalan — bukan mekanisme di kawat yang harus tetap benar
> melintasi pemutusan daya.
>
> Yang **tidak** ikut lenyap, dan jangan dihidupkan lagi karena kelihatan seperti pasangannya: premis
> bahwa **tombol `t0` harus tetap bekerja saat ponsel tidak di tangan** tetap berlaku penuh. Orang
> tidak memegang ponselnya sambil makan. Yang berubah hanya premis untuk **titik ukur**, di mana
> ponsel memang ada di dekat jam — karena penggunanya sendiri yang menyalakan jam untuk mengukur.

**`UKUR_SEKARANG`** — pengukuran di luar sesi mana pun ("Pindai Kesehatan"). Dijawab **paket Sampel
biasa** dengan `sesiId` **16 byte nol** dan `index` 0. ACK-nya tetap dikirim seperti biasa,
mendahului paket Sampel-nya.

**Dilayani di ketiga status** (v1.2) — IDLE, ARMED, **dan RUNNING**. Pengguna boleh menekan
"Pindai Kesehatan" kapan saja, termasuk di tengah sesi yang sedang berjalan. Handler-nya karena itu
sengaja **tidak** memeriksa status sesi sama sekali.

Ia **tidak menyentuh mesin status** (§12): tidak memindahkan status, tidak menyentuh titik yang
ter-ARM, dan tidak menghabiskan index. Hasilnya bukan bagian dari sesi makan mana pun — `sesiId`
nol itulah yang menyatakannya.

Bila sensornya kebetulan sedang mengerjakan titik ukur sesi → NAK `0x05`, dan aplikasi mengulang
setelah 5 detik. **Jangan** menjawab dengan pembacaan lama yang masih hangat di memori: ini
pengukuran yang diminta orang untuk detik ini.

**`SET_KALIBRASI`** — mengirim **offset**, bukan nilai referensi. Jam hanya menambahkannya ke
pembacaan mentah; nilai referensi tensimeter tidak perlu diketahui firmware. Disimpan di NVS agar
bertahan melewati boot. Sentinel 0 **tidak** ikut digeser — nol berarti "gagal diukur", bukan "nol
mmHg".

**`SINKRON`** — memaksa pengiriman ulang. Semua yang masih ada di buffer memang belum di-ack, jadi
semuanya diantrekan ulang; yang sudah pernah dikirim menyalakan flag `dariBuffer`.

**`ACK_EVENT`** — satu-satunya jalan entri keluar dari buffer. Ack untuk seq yang sudah tidak ada
bukan error.

### Urutan yang benar setelah koneksi terbentuk

Aplikasi: baca Info → langgani Peristiwa, Sampel, Status (tulis CCCD) → **baru** `ANCHOR_WAKTU` dan
perintah lain. Firmware harus menghormati sisi lain dari kontrak ini — lihat §11 soal CCCD.

---

## 6. Karakteristik Sampel

`A5A70005`, Notify, **31 byte**.

| Offset | Ukuran | Field | Sentinel gagal |
|---|---|---|---|
| 0 | 1 | `seq` — nomor urut ring buffer | — |
| 1 | 16 | `sesiId` (UUID biner; 16 byte nol untuk `UKUR_SEKARANG`) | — |
| 17 | 1 | `index` (0..3) | — |
| 18 | 1 | `flag` — bit0 `dariBuffer`, bit1 `waktu_tidak_pasti` | — |
| 19 | 2 | `boot_id` uint16 LE | — |
| 21 | 4 | `uptime_s` uint32 LE — waktu ukur di garis waktu boot itu | — |
| 25 | 2 | `gula_darah` uint16 LE, mg/dL | `0` |
| 27 | 1 | `detak_jantung` uint8, bpm | `0` |
| 28 | 1 | `sistolik` uint8, mmHg | `0` |
| 29 | 1 | `diastolik` uint8, mmHg | `0` |
| 30 | 1 | `spo2` uint8, % | `0` |

**Sentinel `0` = metrik gagal diukur** — bukan nilai terakhir yang diketahui. Aplikasi mengubahnya
menjadi "tidak terukur" dan menampilkannya apa adanya; nilai basi yang dikirim seolah segar akan
ditafsirkan pengguna sebagai pengukuran sungguhan. Nol aman karena tidak ada nilai fisiologis nol
yang sah untuk kelima metrik ini.

Arti `index`: **0** baseline (saat foto), **1** saat tombol ditekan (`t0`), **2** di `t0+1 jam`,
**3** di `t0+2 jam`. Sejak v1.3 arti itu **milik aplikasi, bukan firmware**: jam meneruskan `index`
dari payload `UKUR` apa adanya dan tidak tahu jam ke berapa ia mewakili. Yang tetap dipilih jam
sendiri hanya index 1.

Jam **tidak** mengirim sampel "menunggu" atau "terlewat". Kedua status itu diturunkan aplikasi dari
jadwal.

**Flag disusun saat kirim, bukan saat entri dibuat** — keduanya bisa berubah setelah entri masuk
buffer:

```cpp
uint8_t flag = 0;
if (e->pernah_dikirim)              flag |= 0x01;  // dariBuffer
if (!boot_punya_anchor(e->boot_id)) flag |= 0x02;  // waktu_tidak_pasti — anchor bisa datang
                                                   // berjam-jam setelah entrinya dibuat
uint8_t b[31];
b[0] = e->seq;
memcpy(&b[1], e->sesi_id, 16);
b[17] = e->index;
b[18] = flag;
tulis_u16(&b[19], e->boot_id);
tulis_u32(&b[21], e->uptime_s);
tulis_u16(&b[25], e->gula_darah);
b[27] = e->detak_jantung;
b[28] = e->sistolik;
b[29] = e->diastolik;
b[30] = e->spo2;
```

---

## 7. Karakteristik Peristiwa

`A5A70004`, Notify, **26 byte**.

| Offset | Ukuran | Field |
|---|---|---|
| 0 | 1 | `seq` (0 untuk ACK/NAK — lihat di bawah) |
| 1 | 1 | `jenis` |
| 2 | 16 | `sesiId` (nol bila tidak relevan) |
| 18 | 2 | `boot_id` uint16 LE |
| 20 | 4 | `uptime_s` uint32 LE |
| 24 | 1 | `flag` — bit0 `dariBuffer`, bit1 `waktu_tidak_pasti` |
| 25 | 1 | payload/kode |

| `jenis` | Nama | Arti | Payload |
|---|---|---|---|
| `0x01` | `TOMBOL_SELESAI_MAKAN` | **Sumber tunggal `t0`.** `uptime_s` = saat tombol ditekan | 0 |
| `0x02` | `SESI_KEDALUWARSA` | ARM timeout 4 jam terlewat | 0 |
| `0x03` | `SESI_DIBATALKAN_JAM` | Dibatalkan dari jam (mis. baterai kritis) | alasan |
| `0x04` | `UKUR_GAGAL` | Pengukuran gagal total | index sampel |
| `0x05` | `ACK` | — | opcode yang di-ack |
| `0x06` | `NAK` | — | kode error (§9) |
| `0x07` | `BUFFER_PENUH` | Entri tertua dibuang | 0 |
| `0x08` | `BOOT` | `boot_id` baru; memicu `ANCHOR_WAKTU` + `SINKRON` di app | 0 |

`TOMBOL_SELESAI_MAKAN` **wajib masuk ring buffer** seperti entri lain — justru event inilah yang
paling sering terjadi saat HP tidak tersambung. Begitu juga `BOOT`: selama jam menyala tanpa HP, ia
menunggu di buffer, dan `boot_id` di dalamnya sendiri yang memberi tahu aplikasi ada garis waktu
baru.

### ACK dan NAK tidak pernah masuk ring buffer

Ini kesalahan yang paling mahal dan paling lambat ketahuan. Aplikasi **tidak pernah** meng-`ACK_EVENT`
sebuah balasan — jadi ACK yang menunggu di-ack tidak akan pernah dibersihkan. Buffer 64 entri terisi
penuh oleh ACK basi, lalu mulai membuang **sampel sungguhan**. Gejalanya baru muncul setelah puluhan
perintah, jauh dari penyebabnya.

ACK/NAK adalah percakapan sesaat, bukan riwayat: dikirim langsung, sekali, tidak pernah dikirim
ulang. `seq`-nya **0**, yang memang sudah disisihkan sebagai "bukan entri buffer". Balasan yang tiba
berjam-jam kemudian lewat buffer tidak punya siapa pun yang masih menunggunya.

```cpp
static void balas(uint8_t jenis, uint8_t payload) {   // jenis = 0x05 ACK atau 0x06 NAK
  uint8_t b[26];
  b[0] = 0;                        // seq 0: tidak perlu di-ACK_EVENT, tidak masuk buffer
  b[1] = jenis;
  memcpy(&b[2], sesi_id, 16);
  tulis_u16(&b[18], boot_id);
  tulis_u32(&b[20], uptime_s());
  b[24] = 0;
  b[25] = payload;
  ble_kirim_peristiwa(b, sizeof(b));   // langsung, tanpa lewat ring
}
```

---

## 8. Karakteristik Status dan baterai

`A5A70006`, Read + Notify, **10 byte** sejak v1.4 (8 byte sebelumnya).

| Offset | Ukuran | Field |
|---|---|---|
| 0 | 1 | `status_sesi`: 0 idle, 1 armed, 2 running |
| 1 | 1 | `sampel_tertunda` — jumlah entri belum di-ack di buffer |
| 2 | 1 | `baterai` % |
| 3 | 1 | `flag`: bit0 sedang mengukur, bit1 kalibrasi tersimpan, bit2 baterai kritis, bit3 boot ini sudah punya anchor |
| 4 | 4 | `uptime_s` uint32 LE |
| 8 | 1 | `ukur_persen` 0..100 — kemajuan pengukuran, 0 bila tidak mengukur *(v1.4)* |
| 9 | 1 | `ukur_sisa_detik` — perkiraan sisa, **jenuh di 255**, 0 bila tidak mengukur *(v1.4)* |

Dikirim sebagai notifikasi setiap kali salah satu isinya berubah (ARM, tombol, mulai/selesai ukur,
anchor tersimpan, kalibrasi tersimpan, kembali ke IDLE), dan disegarkan juga saat dibaca.

### 8.1 Denyut selama mengukur (v1.4)

**Selagi `flag` bit0 menyala, paket ini dikirim ulang setiap 2 detik walau isinya tidak berubah.**
Itu satu-satunya penyimpangan dari aturan "kirim hanya saat berubah" di atas, dan ia ada karena
aturan itu membuat bit0 tidak bisa dipercaya:

- selama pengukuran, byte 0..3 diam sama sekali, jadi bit0 hanya pernah terkirim **dua kali** —
  sekali di `ukur_mulai()`, sekali di `ukur_selesai()`;
- akibatnya jam yang mati, tertidur, atau keluar jangkauan di tengah pengukuran meninggalkan bit0
  menyala **selamanya** di sisi aplikasi, tidak bisa dibedakan dari jam yang benar-benar sedang
  bekerja dengan nadi yang sulit ditemukan;
- notifikasi "selesai" yang hilang di udara memberi gejala yang persis sama.

Karena itu yang menjadi bukti hidup adalah **paket yang sampai**, bukan angka yang bergeser: denyut
tetap dikirim walau `ukur_persen` kebetulan tidak berubah — persen yang mandek adalah keadaan sah
yang punya kalimatnya sendiri di aplikasi ("rapatkan jam"), bukan alasan menggagalkan pengukuran.

Byte 8..9 **tidak** ikut dalam perbandingan "berubah atau tidak" (seperti `uptime_s`), karena
`ukur_sisa_detik` berubah tiap detik dan akan memaksa notifikasi sekali per detik. Yang mengatur
kedatangannya hanyalah kadensi 2 detik itu.

Biayanya terbatas: pengukuran terpanjang (batas keras 5 menit) menghasilkan ~150 paket 10 byte, dan
lalu lintasnya berhenti sendiri begitu pengukurannya selesai.

Aturan aplikasi yang berpasangan dengannya (`BleAsliService.ukurSekarang`):

| Kejadian | Sikap aplikasi |
|---|---|
| Denyut datang, `ukur_persen` bergerak | Tunggu terus — tidak ada tenggat total |
| Denyut datang, `ukur_persen` diam ≥ 60 dtk | Tunggu terus, layar berkata "jam belum menemukan nadi" |
| Tiga denyut terlewat (8 dtk) | Hentikan: "jam berhenti mengabari" |
| Paket 8 byte (firmware ≤ v1.3) | Tidak ada denyut untuk dijaga; pakai batas keras 5 menit |
| bit0 padam, sampel belum datang | Beri 8 detik — Sampel memang berangkat lewat ring buffer sesudah paket Status ini |
| Tautan putus | Hentikan seketika: "jam terputus" |
| `UKUR_GAGAL` ber-`sesiId` nol | Hentikan seketika: "jam tidak berhasil membaca" |

Baterai diduplikasi ke **Battery Level standar** `0x180F` / `0x2A19` (Read + Notify) supaya satu kali
baca cukup bagi aplikasi.

---

## 9. Kode error (payload `NAK`)

| Kode | Arti | Perilaku aplikasi |
|---|---|---|
| `0x01` | Opcode tidak dikenal | Bug versi. Catat, jangan retry |
| `0x02` | Payload tidak valid | Bug. Catat, jangan retry |
| `0x03` | Jam belum di-ARM | Sesi jadi "menunggu perangkat" |
| `0x04` | Sesi tidak dikenal | Sesi sudah kedaluwarsa di jam; batalkan di app |
| `0x05` | Sedang mengukur | Retry setelah 5 detik |
| `0x06` | Baterai terlalu rendah | Tampilkan; jangan retry |
| `0x07` | Sensor gagal | Tampilkan; sampel jadi "terlewat" |
| `0x08` | Kalibrasi belum ada | Tekanan darah dikirim tanpa koreksi |
| `0x09` | `boot_id` tidak cocok | Jam reboot di tengah perintah; app baca ulang handshake, anchor ulang, ulangi perintah |

Semua write dari aplikasi memakai timeout 5 detik dan maksimal 3 percobaan, kecuali yang dilarang
retry di atas.

---

## 10. Koneksi dan keamanan

| Parameter | Nilai | Alasan |
|---|---|---|
| MTU | minta 185, terima ≥ 35 | Sampel butuh **31 byte utuh dalam satu notifikasi** — jangan dipecah |
| Connection interval | 30–50 ms saat sesi RUNNING, 200–500 ms saat idle | Hemat baterai di luar sesi |
| Bonding | Wajib, LE Secure Connections, **Just Works** | Data kesehatan |
| Enkripsi | Wajib pada **semua** karakteristik kustom | — |

```cpp
NimBLEDevice::init(nama);
NimBLEDevice::setMTU(185);
NimBLEDevice::setSecurityAuth(true, false, true);             // bonding, MITM=false, SC
NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);    // Just Works

// Izin READ_ENC / WRITE_ENC di kelima karakteristik kustom:
c_info = svc->createCharacteristic(UUID_INFO,
    NIMBLE_PROPERTY::READ  | NIMBLE_PROPERTY::READ_ENC |
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC);
c_kontrol = svc->createCharacteristic(UUID_KONTROL,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC);
c_sampel = svc->createCharacteristic(UUID_SAMPEL,
    NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ_ENC);
// ... peristiwa dan status serupa
```

**Jam tidak pernah memulai security request sendiri** — biarkan ponsel yang memulai. Peripheral yang
mengirim security request duluan akan memunculkan dialog penyandingan pada **setiap** sambung-ulang
di latar belakang, dan alur pemasangan di sisi aplikasi dirancang justru untuk mencegah itu.

**Bond wajib bertahan di NVS melewati reboot jam.** Aplikasi menghentikan lingkaran sambung-ulangnya
begitu bond hilang dan menampilkan "Jam Tidak Tersandingkan"; ia **tidak** akan menyandingkan ulang
dari latar belakang. Jam yang kehilangan bond-nya sendiri setelah reboot karena itu tidak akan
tersambung lagi sampai pengguna menyandingkannya secara manual.

Interval koneksi diubah lewat `updateConnParams` (satuan 1,25 ms):

```cpp
if (sesi_berjalan) server->updateConnParams(conn_handle,  24,  40, 0, 400);   // 30–50 ms
else               server->updateConnParams(conn_handle, 160, 400, 0, 400);   // 200–500 ms
```

> **Kenapa Just Works, bukan passkey.** Jam tidak punya layar khusus pairing, jadi satu-satunya
> passkey yang bisa dipakai adalah angka yang dipatok di firmware. Angka seperti itu bukan
> perlindungan MITM melainkan tampilannya saja: ia tertulis di firmware, di dokumen ini, dan di
> setiap salinan keduanya. Just Works menyatakan dengan jujur apa yang sebenarnya didapat — tautan
> terenkripsi dan bonding yang bertahan, tanpa perlindungan MITM saat pairing. Bonding, LESC, dan
> enkripsi tetap wajib; yang dilepas hanya otentikasi pairing-nya.
>
> **Karena itu flag MITM di `setSecurityAuth` adalah `false`, dan itu disengaja.** Dengan IOCap
> `NO_INPUT_OUTPUT`, MITM tidak akan pernah tercapai — memintanya berarti menuntut jaminan yang
> tidak bisa dipenuhi, dan itu bukan keamanan tambahan melainkan risiko pairing ditolak. Jangan
> mengembalikannya ke `true` karena "kelihatannya lebih aman": yang benar-benar melindungi data
> kesehatan di sini adalah enkripsi tautan + bonding, dan keduanya tetap menyala.
>
> Ini bisa ditinjau ulang **hanya bila jam menampilkan passkey acak di layarnya** — passkey acak
> yang ditampilkan memberi perlindungan sungguhan. Jangan "memperbaikinya" menjadi passkey tetap:
> hasilnya lebih buruk, bukan lebih baik.

---

## 11. Ring buffer dan ack

Ring buffer **64 entri di flash/NVS**, event dan sampel bercampur dalam satu ruang `seq`. 64 entri
cukup untuk ~16 sesi penuh tanpa sinkronisasi sama sekali.

**Flash, bukan RAM** — buffer adalah satu-satunya hal yang menyeberangi batas boot.

Aturan, seluruhnya:

1. Setiap entri dapat `seq` **1..255**, berputar. **`0` tidak pernah dipakai** — aplikasi memakainya
   sebagai "belum pernah menerima apa pun" saat mengirim `SINKRON`, dan ACK/NAK memakainya sebagai
   "bukan entri buffer".
2. **Entri tidak dihapus saat dikirim, hanya saat di-`ACK_EVENT`.** Aplikasi baru meng-ack setelah
   entrinya tersimpan permanen di basis datanya. Duplikat karena itu normal.
3. Saat tersambung **dan sudah dilanggani**, jam mengirim seluruh entri belum-di-ack secara
   berurutan dari yang tertua, lalu entri baru secara realtime.
4. Entri yang dikirim ulang menyalakan flag bit0 `dariBuffer`.
5. Buffer penuh → entri **tertua** dibuang, lalu kirim event `BUFFER_PENUH`. Bukan menimpa
   diam-diam. Kirim event-nya dari luar fungsi penambah entri, supaya penambahan tidak rekursif.
6. `SINKRON` mengantrekan ulang semua entri yang belum di-ack.
7. **Entri lintas boot hidup berdampingan.** Masing-masing membawa `boot_id`-nya sendiri; **jangan
   membersihkan buffer saat boot.** Entri yang selamat diantrekan ulang untuk dikirim.
8. Record anchor disimpan **terpisah** dari ring buffer dan tidak pernah ditimpa entri baru.
9. ACK/NAK tidak pernah masuk (§7).

### Menunggu CCCD, bukan menunggu koneksi

**Tersambung saja tidak cukup untuk mulai mengirim.** Aplikasi menyambung, membaca Info, baru
berlangganan — dan notifikasi yang dikirim di sela itu **hilang tanpa jejak**, sementara entrinya
terlanjur ditandai "sudah dikirim" dan tidak datang lagi sampai ada `SINKRON` berikutnya.

Pengurasan buffer karena itu dipicu oleh `onSubscribe` untuk **Sampel dan Peristiwa keduanya**:

```cpp
void onSubscribe(NimBLECharacteristic* c, NimBLEConnInfo& info, uint16_t nilai) override {
  *bendera_ = (nilai != 0);
  if (langganan_sampel && langganan_peristiwa) siap_baru = true;   // dibaca & direset di loop
}

void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
  langganan_sampel = langganan_peristiwa = false;   // CCCD ditulis ulang tiap koneksi
  NimBLEDevice::startAdvertising();
}
```

### Kirim berjeda, jangan satu loop

Mengosongkan 64 entri sekaligus akan memenuhi antrean notifikasi NimBLE. Satu entri per giliran:

```cpp
static const uint32_t JEDA_KIRIM_MS = 60;

static void putar_pengirim() {
  static uint32_t terakhir_ms = 0;
  if (!ble_siap_notifikasi()) return;                        // terhubung DAN dilanggani
  if (millis() - terakhir_ms < JEDA_KIRIM_MS) return;
  Entri* e = ring_berikutnya_untuk_dikirim();                // tertua yang perlu dikirim
  if (!e) return;
  kirim_entri(e);
  e->pernah_dikirim = 1;
  e->perlu_dikirim  = 0;
  terakhir_ms = millis();
}
```

### Tulis flash ditunda — tetapi hanya untuk ack (v1.3)

Menulis flash sambil BLE aktif bisa memblokir cukup lama untuk mengganggu jadwal koneksi — dan di
firmware LVGL, cukup lama untuk menjatuhkan frame. Karena itu perubahan digabungkan: tandai "kotor",
lalu tulis sekali setelah ~3 detik tenang. Itu yang mencegah pengurasan 64 entri menjadi 64 tulis
berturut-turut, dan yang datang berombongan memang ack.

**Tetapi jeda itu asimetris, dan sejak v1.3 asimetrinya menentukan.**

| Perubahan | Kapan ditulis | Kalau daya putus sebelum tertulis |
|---|---|---|
| Entri baru (sampel / peristiwa) | **Seketika** | Sampelnya hilang **permanen** |
| Ack (entri dihapus) | Boleh ditunda ~3 detik | Entrinya terkirim ulang — perilaku normal, dan aplikasi men-dedup-nya |

Pola pemakaian yang dikunci v1.3 adalah: nyalakan jam, ukur satu titik, **matikan lagi**. Tidak ada
alasan bagi siapa pun menunggu sesudah pengukurannya selesai, jadi daya yang putus di dalam jendela
3 detik itu bukan kasus tepi melainkan yang diharapkan terjadi. Yang hilang adalah titik ukur yang
**tidak bisa diulang** — `t0+1 jam` cuma terjadi sekali — dan hilangnya tidak menghasilkan gejala apa
pun selain titik yang tetap kosong: di aplikasi ia terlihat persis seperti sensor yang gagal atau
pengguna yang lupa. Tidak ada yang akan tahu itu bug.

Biayanya ~5 tulis tambahan per hari. Kalau suatu saat ada yang menyeragamkan kedua jalur "supaya
rapi", ia menghapus perlindungan ini tanpa satu pun gejala.

**Dua godaan yang tersisa**, dan keduanya sudah menjelaskan diri lewat tabel di atas: menulis setiap
perubahan status, dan menunda tulisan yang membawa data baru.

**Nilai balik tulisan wajib diperiksa.** Mengabaikannya lalu menurunkan flag "kotor" berarti buffer
berhenti persisten **diam-diam** saat NVS penuh atau gagal — tanpa gejala apa pun sampai jam reboot
dan seluruh isinya lenyap. Bila gagal: "kotor" tetap menyala supaya percobaan berikutnya
mengulanginya, dan kegagalan yang berulang harus terlihat.

**`ARM_TITIK` ikut menghuni NVS** sejak v1.3 (§5). Ia ditulis begitu diterima — tombol yang lupa
dirinya saat daya diputus tidak berguna — tetapi penulisnya **membandingkan dulu**: `ARM_TITIK`
berulang dengan isi yang sama tidak boleh menyentuh flash sama sekali.

---

## 12. Mesin status sesi

**Disederhanakan di v1.3: jam tidak lagi menjadwalkan apa pun.** Sebabnya ada di kotak pembuka
dokumen ini — jam pasti mati di tengah sesi, jadi penjadwal di firmware tidak pernah bisa
menyelesaikan tugasnya, dan mempertahankannya hanya menghasilkan sesi yang selalu berakhir tidak
lengkap.

```
     ┌──────┐  ARM_SESI          ┌───────┐  tombol ditekan   ┌─────────┐
     │ IDLE │ ─────────────────▶ │ ARMED │  atau MULAI_SESI  │ RUNNING │
     └──────┘                    └───────┘ ────────────────▶ └─────────┘
        ▲                            │  timeout 4 jam            │
        │                            │  / BATAL_SESI             │ BATAL_SESI
        └────────────────────────────┴───────────────────────────┘  atau daya putus

     ANCHOR_WAKTU, UKUR, UKUR_SEKARANG, dan ARM_TITIK boleh masuk di status mana pun —
     tidak satu pun menyentuh mesin ini.
```

- **IDLE** — tombol "Selesai Makan" tidak berfungsi, dan `MULAI_SESI` dijawab NAK `0x03`.
  `UKUR_SEKARANG` dan `UKUR` tetap dilayani, dan **tombol ukur berfungsi bila ada `ARM_TITIK`
  tersimpan di NVS** — itu justru keadaan yang paling lazim, karena jam yang baru dinyalakan di
  tengah sesi selalu ada di IDLE.
- **ARMED** — tombol berarti "Selesai Makan", dan `MULAI_SESI` (§5) melakukan hal yang sama persis
  dengannya. **Timeout 4 jam** `uptime_s`: lewat dari itu kirim `SESI_KEDALUWARSA` dan kembali IDLE.
  Tanpa ini, foto sarapan yang tombolnya tidak pernah ditekan akan menyalakan tombol sampai malam.
- **RUNNING** — index 1 diukur segera setelah tombol ditekan, lalu jam **tidak menjadwalkan apa pun**.
  Titik berikutnya datang sebagai `UKUR` dari aplikasi, atau dari tombol ukur yang di-ARM.
- **Daya putus tidak lagi mengakhiri sesi.** Jam kembali ke IDLE dan lupa sesinya; yang mengingat
  sesi itu aplikasi. Satu-satunya yang bertahan di NVS adalah `ARM_TITIK` yang belum terpakai.

Enam hal yang wajib dipegang.

**1. Tidak ada satu pun timer jadwal di firmware.** Tidak ada `t0 + 3600`, tidak ada `t0 + 7200`, dan
tidak ada konstanta untuk keduanya. Godaan terbesarnya adalah menghidupkannya kembali "sebagai
cadangan kalau HP tidak datang"; cadangan itu tidak pernah bisa benar — jamnya sudah mati saat
titiknya jatuh tempo.

**2. Index 1 tetap diukur jam sendiri, dan hanya index 1.** Aturan v1.3 adalah "jam tidak pernah
**menjadwalkan**", bukan "jam tidak pernah mengukur tanpa disuruh": index 1 bukan jadwal melainkan
akibat langsung sebuah peristiwa lokal, sekelas dengan jawaban atas `UKUR_SEKARANG`.

Bedanya menentukan. Tombol "Selesai Makan" fisik ada justru untuk keadaan ponsel tidak di tangan, dan
itu keadaan paling lazim saat orang sedang makan. Kalau index 1 menunggu `UKUR` dari aplikasi, tombol
yang ditekan tanpa HP di dekatnya menghasilkan `t0` **tanpa** pengukurannya — dan pengukuran itu baru
terjadi saat HP tersambung, bisa berjam-jam kemudian, di titik yang sudah bukan "baru selesai makan"
lagi. Yang hilang bukan presisi melainkan artinya.

**3. `SESI_RUNNING` tetap ada dan tidak boleh dihapus**, meski isinya tinggal sedikit. Ia masih
menegakkan tiga hal: tombol "Selesai Makan" hanya bekerja di ARMED, `MULAI_SESI` tidak pernah
menetapkan `t0` kedua, dan index 1 diukur tepat satu kali.

**4. Dedup `(sesiId, index)` di RAM, bukan NVS**, dipakai jalur `UKUR` **dan** jalur tombol:

```
di RAM (bukan NVS):  sesi_dedup[16]  +  index_selesai bitmask

UKUR(sesiId, index):
  bila sesiId != sesi_dedup:            sesi_dedup = sesiId; index_selesai = 0
  bila bit index sudah menyala:         ACK, lalu berhenti
  selain itu:                           ACK, ukur
  bit index dinyalakan SETELAH pengukurannya tuntas, dan HANYA bila berhasil
```

Empat hal yang membuat bentuk ini yang dipilih:

- **RAM, bukan NVS.** Cakupannya memang satu masa hidup daya, jadi reboot yang menghapusnya adalah
  perilaku yang benar, bukan kehilangan — dan dedup jadi tidak menyentuh flash sama sekali.
- **`sesiId` berbeda me-reset bitmask, tidak pernah menolak.** Jam tidak tahu sesi mana yang sedang
  berjalan dan tidak boleh berpura-pura tahu; satu sesi baru cukup menggantikan yang lama.
- **Bit dinyalakan sesudah pengukuran tuntas**, supaya titik yang tertunda karena sensor sibuk masih
  bisa dicoba lagi, bukan tercatat selesai padahal belum.
- **Dan hanya bila berhasil.** Pengukuran yang gagal total tidak boleh mengunci titiknya — kalau
  bitnya menyala, percobaan ulang lewat `UKUR` maupun lewat tombol akan ditolak sebagai "sudah
  terisi" padahal tidak ada satu pun sampel yang keluar.

**5. Satu tombol fisik, dua makna.** Jam punya satu tombol, dan artinya ditentukan keadaannya: ARMED
berarti "Selesai Makan", `ARM_TITIK` yang tersimpan berarti "Ukur". **Layar LVGL yang menuliskan
artinya saat itu**, sehingga tidak ada yang perlu menghafal konteks — ini bagian Anda, dan ia bukan
hiasan: dua tombol fisik lebih mahal daripada satu tombol yang layarnya menjelaskan diri, dan
penggunanya lansia.

Bila keduanya bisa aktif bersamaan, **`ARM_SESI` yang menang — dan ia menghapus `ARM_TITIK`**, di RAM
maupun NVS.

Ini kebalikan dari yang disarankan asimetri §12.1 ("titik ukur selalu menang"), dan alasannya harus
dibaca sampai habis sebelum diubah. Aplikasi hanya mengizinkan satu sesi aktif pada satu waktu, dan
`ARM_TITIK` hanya dikirim setelah `t0` ada — saat itu jam sudah RUNNING, bukan ARMED. Maka
satu-satunya cara `ARM_SESI` dan `ARM_TITIK` bertemu adalah: `ARM_TITIK` itu **milik sesi yang sudah
berakhir**, dan `BATAL_SESI` penutupnya tidak sampai karena jam sedang mati. Membiarkannya menang
berarti tombol basi milik sesi mati mencuri `t0` sesi yang hidup, lalu mengirim sampel yang di
aplikasi tidak jatuh ke mana-mana — yang hilang justru yang tidak bisa diulang.

**6. Reboot mengembalikan jam ke IDLE**, dan sampel lama tetap di buffer dengan `boot_id` lamanya.
Yang berubah di v1.3 adalah **artinya**: ini keadaan normal yang diharapkan, bukan sesi yang gagal.
Sesinya tetap hidup di aplikasi dan titik berikutnya masih bisa diukur. Tetap **jangan mencoba
melanjutkan sesi lintas boot di firmware** — bukan lagi karena mustahil tanpa RTC, melainkan karena
tidak ada yang perlu dilanjutkan di sini.

**Premis yang dibatalkan.** "Seluruh siklus harus selesai walau HP tidak pernah tersambung sekali
pun" **tidak berlaku lagi** — itu premis v1.2, dan ia mengandaikan jam hidup 2,5 jam terus-menerus.
Tanpa HP tidak ada `UKUR` dan tidak ada `ARM_TITIK`, jadi tidak ada titik sesudah index 1 yang
terukur.

Yang tersisa dari "jam bekerja sendirian", dan yang memang harus tetap ada: **tombol yang ditekan
tanpa HP di dekatnya tetap menghasilkan `t0` dan index 1**, dan keduanya menunggu di buffer sampai HP
datang.

### 12.1 Perebutan sensor

Sejak ada dua hal yang bisa meminta sensor pada saat yang sama — titik ukur sesi yang jatuh tempo
sendiri, dan permintaan yang dipicu jari manusia — urutan menangnya harus tertulis, bukan diserahkan
ke siapa yang kebetulan lebih dulu.

Satu kalimat yang mengatur semuanya:

> **Titik ukur sesi selalu menang. Ia ditunda, tidak pernah dibatalkan.**

Alasannya asimetri **nilai**, bukan asimetri teknis: titik ukur sesi adalah data produknya dan
**tidak bisa diulang** — `t0+1 jam` cuma terjadi sekali. Pindai atas permintaan bisa diminta lagi
kapan saja, dan pengguna yang memintanya sedang memegang ponselnya. Membiarkan yang kedua membuat
yang pertama terlewat berarti menukar data yang tidak tergantikan dengan data yang tergantikan.
Ditulis sebagai asimetri nilai supaya tetap benar kalau kelak ada pemicu kelima.

| Yang tiba | Sensor sedang sibuk | Perilaku |
|---|---|---|
| `MULAI_SESI` | apa pun | **Selalu diterima.** `t0` dicatat, `TOMBOL_SELESAI_MAKAN` dikirim, index 1 ditunda sampai sensor bebas. Tidak pernah NAK `0x05` |
| Index 1 (akibat tombol) | `UKUR_SEKARANG` berjalan | **Ditunda, diambil segera setelah sensor bebas.** Jangan dibatalkan, dan jangan biarkan pengukuran yang sedang jalan menabraknya |
| `UKUR_SEKARANG` | pengukuran sesi berjalan | NAK `0x05`; aplikasi mengulang setelah 5 detik. **Satu-satunya yang boleh ditolak** |
| `UKUR` (index berapa pun) | apa pun | NAK `0x05`; aplikasi yang memutuskan mengulang atau tidak |
| Tombol ukur ditekan | apa pun | **Diabaikan, tidak diantrekan.** Tombolnya tetap menyala dan boleh ditekan lagi setelah sensor bebas |

**Yang hilang dari tabel ini di v1.3**: baris "titik ukur sesi jatuh tempo" untuk index 2 dan 3 —
tidak ada lagi yang jatuh tempo di firmware. Asimetri nilainya tetap ditulis di sini karena ia masih
mengatur index 1, dan karena ia tetap benar kalau kelak ada pemicu kelima.

**Kenapa `MULAI_SESI` tidak boleh ikut ditolak.** `t0` adalah **stempel waktu, bukan pengukuran**.
Keduanya kebetulan dipicu peristiwa yang sama, tetapi tidak punya kendala yang sama. Kasusnya bukan
kasus tepi melainkan urutan yang **paling lazim**: `UKUR` index 0 dikirim saat shutter kamera
ditekan, lalu pengguna menekan tombol di layar beberapa detik kemudian — dengan sensor sungguhan,
baseline itu masih berjalan. NAK `0x05` di situ membuat aplikasi mengulang 5 detik lagi, sehingga
`t0` **bergeser 5 detik dari saat tombol benar-benar ditekan**, bergeser lagi tiap pengulangan, lalu
menyerah setelah percobaan ketiga sementara penggunanya sudah menekan dan mengira sesinya jalan.

Ini juga satu-satunya bacaan yang konsisten dengan aturan di §5 bahwa peristiwanya tidak boleh
dibedakan dari tombol fisik: tombol fisik tidak punya jalur "ditolak karena sibuk", jadi tombol dari
aplikasi pun tidak boleh punya.

**Bentuk kode yang membuat penundaan aman.** Satu penjaga "sedang mengukur", plus bitmask yang baru
menyala **setelah** pengukuran tuntas — titik yang terlewati satu putaran otomatis dicoba lagi di
putaran berikutnya, bukan hilang:

```c
void putar_sesi(void) {
  if (status == ARMED && sekarang - armed_uptime >= TIMEOUT_ARM_S) {
    catat_peristiwa(EV_SESI_KEDALUWARSA, sesi_id, 0);
    ke_idle();
    return;
  }
  if (status != RUNNING) return;

  if (sedang_mengukur) return;            // ditunda, tidak dibatalkan

  if (!(index_selesai & (1 << 1))) ukur(1);   // index 1, dan HANYA index 1
}
```

Perhatikan index 1 ikut di sini, **bukan** di dalam rutin tombol. Rutin tombol hanya mencatat `t0`,
memancarkan peristiwanya, dan selesai; itulah yang membuat tombol tidak pernah bisa gagal gara-gara
sensor sibuk. Dan perhatikan tidak ada apa pun lagi di bawahnya — itulah seluruh isi penjadwal
sekarang.

> **Catatan silang, supaya tidak ada yang mengandalkan yang salah.** `uptime_s` sampel yang tertunda
> memang jujur mencatat kapan ia benar-benar diukur, dan itu benar untuk disimpan — tetapi
> **aplikasi tidak menampilkannya**: ia menormalkan waktu relatif tiap titik ke slot jadwalnya
> (`0` / `3600` / `7200`), karena label di layar menjanjikan "+1 jam" dan bukan "+1 jam 40 detik".
> Penundaan berskala detik karena itu tidak terlihat di mana pun, dan memang tidak perlu terlihat.
> Yang tidak boleh terjadi bukan penundaannya, melainkan **titiknya hilang**.

Penjaga "sedang mengukur" ini **tidak menimbulkan gejala apa pun selama sensornya masih stub** yang
selesai seketika — dan itu justru alasan menulisnya sekarang, bukan nanti. Tanpa penjaga itu, index 1
akan menabrak `UKUR_SEKARANG` yang sedang berjalan pada hari pertama sensor sungguhan terpasang,
jauh dari perubahan yang menyebabkannya. Sejak v1.3 penjaga yang sama juga melindungi `UKUR` dan
penekanan tombol ukur, yang keduanya kini bisa datang kapan saja.

---

## 13. Struktur kode di firmware LVGL

Masalah pokoknya: **LVGL tidak thread-safe** — tidak ada satu pun `lv_*` yang boleh dipanggil dari
task lain tanpa kunci. Sementara itu ada satu task yang bukan buatan Anda dan tidak bisa dihindari.

### 13.1 Aturan task

**Task host NimBLE** menjalankan seluruh callback BLE: `onWrite`, `onRead`, `onSubscribe`,
`onConnect`, `onDisconnect`. Di dalamnya **dilarang**:

- menyentuh **LVGL** — gejalanya corrupt rendering yang muncul sekali dalam sepuluh menit;
- menyentuh **NVS/flash** — gejalanya koneksi putus-putus di bawah beban;
- menyentuh **ring buffer**.

Yang boleh dilakukan `onWrite` hanyalah menaruh byte mentahnya ke antrean:

```cpp
void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
  NimBLEAttValue v = c->getValue();
  PerintahMasuk p;
  p.panjang = min((size_t)v.length(), MAKS_PERINTAH);
  memcpy(p.data, v.data(), p.panjang);
  xQueueSend(antrean, &p, 0);      // loop yang mengambilnya
}
```

Penyedia paket Info/Status (`onRead`) **wajib murni RAM**. Perhatikan `anchor_boot_ini` di §4: ia
disalin ke variabel RAM saat boot justru karena membacanya dari NVS di dalam callback terlarang.

### 13.2 Satu task untuk protokol + LVGL

Cara paling sederhana dan paling sedikit jebakannya:

> **Jalankan logika protokol di task yang sama dengan `lv_timer_handler()`.**

Dengan begitu callback tombol LVGL boleh memanggil fungsi sesi langsung, dan pembaca status UI boleh
membaca variabel sesi langsung — tanpa mutex, tanpa antrean kedua. Yang menyeberang task tinggal
satu: antrean perintah BLE yang memang sudah ada.

```c
void tugas_jam(void* _) {
  for (;;) {
    jam_putar();                       // protokol
    lv_timer_handler();                // UI
    vTaskDelay(pdMS_TO_TICKS(5));      // vTaskDelay, BUKAN delay()
  }
}
```

Kalau arsitektur Anda menuntut LVGL punya task sendiri, semua panggilan dari sisi jam ke LVGL harus
lewat `lv_async_call()` atau mutex LVGL, dan sisi UI tidak boleh lagi membaca variabel sesi langsung
— sediakan snapshot yang disalin sekali per frame.

### 13.3 Isi `jam_putar()`

Urutannya penting: perintah masuk dulu, mesin status, lalu pengiriman, lalu tulis flash.

```c
void jam_putar() {
  putar_perintah_ble();        // ambil dari antrean, jalankan opcode (§5)
  putar_sesi();                // timeout ARM + index 1; TIDAK ada jadwal lain (§12)

  if (ble_ambil_flag_siap_baru()) {      // CCCD baru ditulis (§11)
    ring_antre_ulang_semua();
    ble_atur_interval(status == RUNNING);
    kirim_status();
  }
  if (ring_ambil_flag_penuh()) {         // dari luar ring_tambah_*, supaya tidak rekursif
    catat_peristiwa(EV_BUFFER_PENUH, NULL, 0);
  }

  putar_pengirim();            // satu entri per 60 ms (§11)
  ring_simpan_jika_perlu();    // tulis flash tertunda (§11)
}
```

### 13.4 Permukaan untuk UI

UI **tidak pernah** menyentuh ring buffer atau NVS langsung. Ia memanggil ini:

```c
void jam_mulai();          // NVS, boot_id++, muat ring, ble_mulai(), catat event BOOT
void jam_putar();          // dipanggil tiap iterasi task

void jam_tekan_tombol();   // dari lv_event_cb SATU tombol fisik — lihat catatan

// Pembaca, semuanya murni RAM dan murah:
uint8_t  jam_status();            // 0 IDLE, 1 ARMED, 2 RUNNING
bool     jam_sedang_mengukur();
uint32_t jam_t0_uptime();         // 0 bila belum RUNNING — informasional saja (v1.3)
uint32_t jam_uptime();
uint8_t  jam_tertunda();          // entri belum di-ack
bool     jam_terhubung();
bool     jam_siap_notifikasi();   // terhubung DAN dilanggani
bool     jam_ada_anchor();

// v1.3 — untuk menuliskan arti tombol di layar (§12, §14):
bool     jam_titik_armed();       // ada ARM_TITIK yang menunggu ditekan
uint8_t  jam_titik_index();       // index-nya; hanya berarti bila di atas true
```

**`jam_tekan_tombol()` melayani satu tombol dengan dua makna**, dan pemilihannya ada di dalam
`jam_*`, bukan di UI: bila `jam_titik_armed()` ia mengukur titik itu, selain itu ia berarti "Selesai
Makan". UI **tidak boleh** memanggil dua fungsi berbeda berdasarkan tebakannya sendiri — keadaan bisa
berubah antara UI membaca dan pengguna menekan, dan yang dihasilkan adalah pengukuran untuk titik
yang salah. UI hanya membaca `jam_titik_armed()` untuk **menuliskan label**.

`jam_t0_uptime()` sejak v1.3 **informasional saja** — `t0` yang mengikat hidup di aplikasi sebagai
wall clock. Ia masih berguna untuk menampilkan "sesi dimulai sekian menit lalu"; jangan membangun
jadwal apa pun di atasnya.

Urutan di `jam_mulai()` yang tidak boleh dibalik: NVS dulu → `boot_id` naik → muat ring (**tanpa
dibersihkan**) → **muat `ARM_TITIK` dari NVS** (tanpa ini tombol ukur justru mati setelah jam
dinyalakan kembali di tengah sesi — persis saat ia paling dibutuhkan) → status sesi dipaksa IDLE →
`ble_mulai()` → catat event `BOOT` ke buffer.

---

## 14. Apa yang ditampilkan UI

Bukan spesifikasi layar — rancangannya milik Anda. Ini daftar keadaan yang **harus punya tampilan**,
karena tanpanya pengguna tidak bisa membedakan jam yang bekerja dari jam yang rusak.

**Tombol fisik — satu tombol, dua makna (v1.3).** Layarlah yang menuliskan artinya saat itu; ini
bagian yang paling menentukan apakah jam ini bisa dipakai orang lansia.

- **Ada `ARM_TITIK` (`jam_titik_armed()`)**: tombol berarti **"Ukur"**, apa pun status sesinya —
  termasuk IDLE, yang justru keadaan paling lazim karena jam baru saja dinyalakan kembali di tengah
  sesi. Labelnya harus menyebut itu pengukuran, bukan "Selesai Makan".
- **ARMED, tanpa titik ter-ARM**: tombol berarti **"Selesai Makan"**, aktif dan menonjol. Ini keadaan
  yang pengguna tunggu.
- **IDLE, tanpa titik ter-ARM**: mati/redup. Menekannya tidak menghasilkan apa-apa selain umpan balik
  "belum disiapkan" (getaran pendek + pesan). Jangan "memperbaikinya" menjadi selalu aktif — inilah
  yang menjamin tidak ada sesi tanpa foto makanan. Umpan baliknya wajib ada: pengguna harus tahu
  **kenapa** tombolnya diam.
- **RUNNING, tanpa titik ter-ARM**: tampilan kemajuan sesi; tidak ada yang perlu ditekan sampai
  aplikasi meng-ARM titik berikutnya.

**Tombol ukur padam setelah satu pengukuran yang berhasil**, dan itu harus terlihat di layar seketika
— termasuk saat yang mengukur adalah aplikasi lewat `UKUR`, bukan tombolnya. Tombol menyala berarti
"ada yang menunggu ditekan"; setelah titiknya terukur, tidak ada, dan tombol yang masih menyala
adalah kebohongan yang akan membuat pengguna menekannya lagi. Pengukuran yang **gagal** justru
meninggalkannya menyala, sebagai percobaan ulang.

**Sesi berjalan.** **Jangan menampilkan hitung mundur ke titik berikutnya** (v1.3): jam tidak tahu
kapan titik berikutnya jatuh tempo — yang tahu aplikasi, dan jam biasanya sudah mati saat itu tiba.
Hitung mundur dari `jam_t0_uptime()` akan berhenti dan menyesatkan pada penyalaan berikutnya. Yang
bisa ditampilkan jujur: sesi sedang berjalan, sudah berapa lama sejak `t0` **dalam masa hidup daya
ini**, dan apakah ada titik yang menunggu diukur.

**Sedang mengukur.** Dengan sensor sungguhan ini makan puluhan detik dan pengguna harus diam; itu
perlu layarnya sendiri, bukan sekadar ikon.

**Koneksi — tiga keadaan, bukan dua.** Tidak tersambung / tersambung / tersambung dan sudah
berlangganan. Hanya yang ketiga yang berarti data sedang mengalir keluar.

**Entri tertunda.** Informasi yang menenangkan, bukan peringatan: buffer 64 entri setara ~16 sesi.

**Waktu tidak pasti.** Bila `jam_ada_anchor()` false, jam memang tidak tahu jam berapa sekarang.
Kalau layar Anda menampilkan jam dinding, keadaan ini harus terlihat — menampilkan waktu tebakan
seolah sungguhan lebih buruk daripada mengaku tidak tahu.

**Baterai kritis.** Di bawah 10%, nyalakan bit2 di flag Status. Bila Anda membatalkan sesi
karenanya, kirim event `SESI_DIBATALKAN_JAM` — jangan diam-diam kembali IDLE.

---

## 15. Jebakan

Diurutkan dari yang paling mahal.

**Service UUID di scan response.** Jam tidak akan pernah terlihat. Lihat §3.2. Bahayanya bukan hanya
saat menulis kode pertama kali, tetapi juga saat menambahkan sesuatu ke iklan nanti dan mendorong
UUID-nya keluar.

**Menyentuh flash, ring buffer, atau LVGL dari callback NimBLE.** Lihat §13.1.

**`delay()` di task yang menjalankan LVGL.** Ganti `vTaskDelay`. Termasuk `delay()` di dalam rutin
pengukuran dan di umpan balik tombol.

**Deep sleep.** Bangun dari deep sleep adalah reset bagi CPU: `esp_timer_get_time()` kembali dari
nol sedangkan pencacah RTC terus berjalan. Kalau firmware memakai deep sleep, dua hal wajib berubah
bersamaan: `uptime_s` **harus** diturunkan dari pencacah RTC, dan `boot_id` **tidak boleh** naik saat
bangun (itu masih masa hidup daya yang sama). Salah satu saja cukup untuk membuat setiap sesi yang
melintasi tidur menjadi salah waktu — dan salahnya tidak akan terlihat sampai ada yang membandingkan
dengan jam dinding. **Cara paling aman di v1: jangan pakai deep sleep sama sekali.** Light sleep
tidak punya masalah ini — dan sejak v1.3 tidak ada yang bisa diselamatkan sleep mana pun: layar tidak
bisa dipadamkan, dan layarnyalah yang mendominasi anggaran daya, bukan radio. Satu-satunya tuas
penghematan adalah memutus daya, dan itu memang yang dilakukan penggunanya.

**MTU dan pemecahan paket.** Sampel 31 byte harus utuh dalam satu notifikasi.

**Membersihkan buffer saat boot.** Entri lintas boot hidup berdampingan.

**Menghidupkan kembali penjadwal di firmware** (v1.3). Timer `t0 + 3600` / `+ 7200` dicabut, dan
godaan terbesarnya adalah memasangnya lagi "sebagai cadangan kalau HP tidak datang". Cadangan itu
tidak pernah bisa benar: jamnya sudah mati saat titiknya jatuh tempo.

**Mengembalikan penjaga `UKUR` yang dicabut** — status harus bukan IDLE, `sesiId` harus cocok.
Keduanya terlihat seperti validasi yang hilang dan sebenarnya adalah v1.2 yang sudah tidak berlaku;
memasangnya lagi membuat setiap titik sesudah yang pertama di-NAK (§5).

**Melanjutkan sesi lintas reboot.** Tetap jangan — tetapi sejak v1.3 alasannya berubah: bukan karena
mustahil tanpa RTC, melainkan karena tidak ada yang perlu dilanjutkan. Sesinya hidup di aplikasi.

**Lupa memuat `ARM_TITIK` dari NVS saat boot.** Gejalanya persis kebalikan dari yang dicari: tombol
ukur mati justru pada penyalaan di tengah sesi, saat ia paling dibutuhkan (§13.4).

**Mengirim nilai terakhir yang diketahui saat pengukuran gagal.** Sentinel `0`.

**Bit `kemampuan` yang tidak jujur.** Metrik yang bitnya 0 disembunyikan aplikasi dari UI.

**Ukuran flash.** LVGL + NimBLE + font gampang melewati 1,3 MB. Pilih skema partisi `Huge APP` atau
kustom **sejak awal** — menemukannya setelah UI setengah jadi berarti mengatur ulang partisi dengan
NVS yang sudah berisi `boot_id` dan buffer pengguna.

**Stack task.** NimBLE + LVGL di satu task butuh stack lebih besar dari bawaan; naikkan sebelum
mengejar crash yang terlihat acak.

---

## 16. Sensor

Kerjakan **paling akhir**, dan pakai nilai palsu sampai iklan, waktu, buffer, dan mesin status
tuntas. Hampir semua yang bisa salah adalah soal waktu, buffer, dan ack — bukan soal sensor. Kalau
keduanya dikembangkan bersamaan, setiap kegagalan punya dua tersangka dan waktu habis untuk menebak
yang mana.

Permukaan yang perlu disediakan:

```c
struct Pembacaan {
  uint16_t gula_darah;      // mg/dL, 0 = gagal
  uint8_t  detak_jantung;   // bpm
  uint8_t  sistolik;        // mmHg
  uint8_t  diastolik;       // mmHg
  uint8_t  spo2;            // %
};
Pembacaan sensor_baca(uint8_t index);   // index cuma diteruskan; jam tidak menafsirkannya
uint8_t   sensor_baterai();
```

Stub-nya sebaiknya mengikuti bentuk kurva sesi makan yang wajar, bukan angka acak — supaya grafik di
aplikasi terlihat masuk akal saat pengujian. Sediakan juga saklar "paksa semua metrik gagal" untuk
menguji sentinel 0 dan `UKUR_GAGAL`.

Bila **semua** metrik gagal, jangan kirim sampel — catat event `UKUR_GAGAL` dengan payload index.
Bila hanya sebagian, kirim sampel dengan metrik yang gagal bernilai `0`.

Kalibrasi diterapkan di sini, ke pembacaan mentah, dan **tidak menyentuh sentinel**:

```c
if (kalibrasi.valid) {
  if (p.sistolik)  p.sistolik  = constrain(p.sistolik  + kalibrasi.offset_sistolik,  1, 255);
  if (p.diastolik) p.diastolik = constrain(p.diastolik + kalibrasi.offset_diastolik, 1, 255);
}
```

Dua hal berubah begitu sensor sungguhan masuk, dan keduanya menyentuh UI: pengukuran menjadi puluhan
detik (rutin ukur harus jadi mesin status sendiri, atau LVGL membeku), dan `sensor_baterai()` harus
membaca ADC sungguhan.

---

## 17. Menguji dan checklist

### Tanpa aplikasi

**nRF Connect** (Android/iOS) cukup untuk memverifikasi iklan, handshake, dan buffer: ia membaca
karakteristik Info, menulis opcode mentah ke Kontrol, dan menampilkan notifikasi masuk. Sebagian
besar bug byte-level tertangkap di sini, jauh lebih cepat daripada lewat aplikasi.

**Konsol serial** sangat disarankan: cetak setiap paket yang dikirim sebagai hex, dan sediakan
perintah untuk menyuntik opcode Kontrol persis seperti aplikasi menulisnya — lewat **jalur yang sama
dengan tulisan BLE**, bukan dengan memanggil rutin internalnya, supaya yang teruji handler-nya
termasuk cabang NAK dan idempotensinya.

Satu sesi utuh tanpa HP, sejak v1.3:

```
anchor -> arm -> ukur 0 -> tombol -> titik 2 -> tombol -> titik 3 -> tombol
```

(`titik <idx>` = `ARM_TITIK`; `mulai` = `MULAI_SESI`, yang menggantikan `tombol` pertama untuk
menguji jalur aplikasi — jalankan dua kali untuk menguji idempotensinya.)

**Pengali `skala` tinggal punya satu pemakai: ARM timeout 4 jam.** Tidak ada lagi jadwal titik ukur
yang bisa dipercepat, karena jamnya tidak lagi menunggu apa pun. Jangan menghapus pengalinya —
mencabut penjadwal tidak mencabut satu-satunya cara menguji tenggat empat jam tanpa menunggu empat
jam.

### Checklist firmware

- [ ] Service UUID ada di **paket iklan**, nama + versi mayor di scan response
- [ ] Jam tanpa bond mengiklan 100 ms **tanpa batas waktu**, dan terlihat dalam satu detik pertama
      pemindaian
- [ ] Jam ber-bond mengiklan cepat 30 detik sesudah boot/putus, lalu turun ke 1000 ms
- [ ] Jam ber-bond yang dinyalakan **membawa entri di buffer** mengiklan cepat 5 menit, dan bisa
      disambungkan dalam sekali coba sepanjang jendela itu
- [ ] `BATAL_SESI` memadamkan tombol ukur yang ter-ARM untuk sesi itu — ARM titik → batalkan sesi →
      tombolnya padam, dan tetap padam sesudah jam dimatikan-dihidupkan
- [ ] Menghapus jam dari Pengaturan Bluetooth ponsel membuatnya **kembali mengiklan cepat**
- [ ] Interval iklan tidak pernah ditata ulang dari dalam callback NimBLE
- [ ] Handshake mengembalikan 20 byte sesuai §4
- [ ] `boot_id` naik tepat satu setiap daya diputus-sambung, bertahan di NVS
- [ ] `uptime_s` monoton naik dalam satu masa hidup daya
- [ ] Tidak ada satu pun paket yang memuat wall clock
- [ ] Setiap opcode dibalas ACK/NAK — **kecuali `ACK_EVENT`**
- [ ] ACK/NAK ber-`seq` 0, **tidak masuk ring buffer**, tidak pernah dikirim ulang
- [ ] Pengurasan buffer baru dimulai **setelah CCCD ditulis**, bukan saat koneksi terbentuk
- [ ] `ANCHOR_WAKTU` dengan `boot_id` tidak cocok di-NAK `0x09`
- [ ] Record anchor bertahan melewati reboot, terpisah dari ring buffer
- [ ] Entri dari boot tanpa anchor dikirim dengan flag `waktu_tidak_pasti`
- [ ] `UKUR_SEKARANG` dijawab paket Sampel ber-`sesiId` 16 byte nol, `index` 0
- [ ] `UKUR_SEKARANG` dilayani di **ketiga status**, tidak memindahkan status, dan tidak menyentuh
      titik yang ter-ARM
- [ ] `UKUR` dilayani di **IDLE, ARMED, maupun RUNNING** (v1.3), dan `(sesiId, index)` dari payload
      diteruskan apa adanya ke paket Sampel **tanpa divalidasi** terhadap sesi mana pun
- [ ] `UKUR` untuk `(sesiId, index)` yang sudah terukur dalam masa hidup daya yang sama di-`ACK`
      lalu **diabaikan**, bukan diukur ulang
- [ ] Kunci dedup hidup di **RAM**, dan `sesiId` berbeda **me-reset** bitmask-nya alih-alih menolak
      perintahnya
- [ ] Dedup dipakai jalur `UKUR` **dan** jalur tombol, bukan salah satu
- [ ] Bit dedup menyala **setelah** pengukuran tuntas dan **hanya bila berhasil** — pengukuran yang
      gagal total masih bisa diulang
- [ ] `ARM_TITIK` menyalakan tombol ukur seketika, dan tombolnya **padam setelah satu pengukuran
      yang berhasil**
- [ ] `ARM_TITIK` **bertahan melewati pemutusan daya**: ARM titik → matikan jam → hidupkan lagi →
      tombolnya menyala tanpa aplikasi mengirim apa pun
- [ ] `ARM_TITIK` baru menimpa yang lama, di RAM maupun NVS; hanya satu titik ter-ARM pada satu waktu
- [ ] `UKUR` untuk titik yang sedang ter-ARM **memadamkan tombolnya dan menghapus catatan NVS-nya**:
      ARM titik → ukur lewat `UKUR` → tekan tombolnya → tidak ada pengukuran kedua, dan tombolnya
      sudah padam sebelum ditekan
- [ ] Pengukuran yang **gagal** meninggalkan tombolnya menyala, sebagai percobaan ulang
- [ ] `ARM_SESI` menghapus `ARM_TITIK` yang tersimpan
- [ ] Firmware **tidak menjadwalkan** index mana pun sendiri — tidak ada timer `t0 + 3600`
- [ ] Label tombol di layar mengikuti keadaan: "Ukur" bila ada titik ter-ARM, "Selesai Makan" bila
      ARMED, mati bila tidak keduanya
- [ ] `MULAI_SESI` di ARMED menghasilkan `TOMBOL_SELESAI_MAKAN` yang **tidak bisa dibedakan** dari
      tombol fisik; di IDLE di-NAK `0x03`, dengan `sesiId` lain di-NAK `0x04`
- [ ] `MULAI_SESI` yang diulang untuk sesi yang **sudah RUNNING** di-ACK lalu **diabaikan** — bukan
      `t0` kedua, bukan `TOMBOL_SELESAI_MAKAN` kedua
- [ ] `MULAI_SESI` **tidak pernah** dijawab NAK `0x05`, walau sensor sedang sibuk (§12.1)
- [ ] Titik ukur sesi yang bertabrakan dengan `UKUR_SEKARANG` **ditunda, bukan dibatalkan** — ada
      penjaga "sedang mengukur", walau stub sensor belum menimbulkan gejala
- [ ] Tombol mati di IDLE, hidup di ARMED, dengan umpan balik saat ditekan di IDLE
- [ ] ARM timeout 4 jam menghasilkan `SESI_KEDALUWARSA`
- [ ] Reboot saat RUNNING → kembali IDLE, sampel lama tetap di buffer dengan `boot_id` lama. Sejak
      v1.3 ini **keadaan normal yang diharapkan**, bukan kegagalan: sesinya tetap hidup di aplikasi
- [ ] Ring buffer bertahan melewati reset dan memuat entri lintas boot
- [ ] Entri kirim-ulang menyalakan flag `dariBuffer`
- [ ] Buffer penuh mengirim `BUFFER_PENUH`, bukan menimpa diam-diam
- [ ] Metrik gagal dikirim sebagai `0`
- [ ] Offset kalibrasi bertahan melewati boot
- [ ] Pairing Just Works + LE Secure Connections (flag MITM `false`), enkripsi di kelima
      karakteristik
- [ ] Bond bertahan melintasi reboot jam; jam **tidak pernah** memulai security request sendiri
- [ ] ~~Siklus sesi penuh selesai **tanpa HP tersambung sama sekali**~~ — **dicabut di v1.3.** Tanpa
      HP tidak ada `UKUR` dan tidak ada `ARM_TITIK`. Yang menggantikannya:
- [ ] Tombol yang ditekan **tanpa HP di dekatnya** tetap menghasilkan `t0` **dan** index 1, dan
      keduanya menunggu di buffer sampai HP datang
- [ ] LVGL tetap responsif selama pengiriman buffer, penulisan flash, dan pengukuran

### Uji integrasi

- [ ] Satu sesi penuh (baseline → t0 → +1 jam → +2 jam) dengan hardware nyata
- [ ] **Bluetooth HP dimatikan sebelum foto diambil, dinyalakan setelah +2 jam** → seluruh sesi
      diterjemahkan dengan benar dari satu anchor di akhir
- [ ] **Jam dimatikan dan dinyalakan lagi di tengah sesi → sesi tetap berjalan**, titik berikutnya
      masih bisa diukur, dan tombol yang ter-ARM sebelum daya putus menyala kembali sendiri (v1.3;
      sebelumnya sesi berakhir tidak lengkap di sini). Sampel dari sebelum reboot tetap masuk dengan
      waktu yang benar
- [ ] Jam kehabisan daya, boot, dipakai satu sesi penuh, mati lagi, baru tersambung → sesi itu
      ditandai `waktu_tidak_pasti`

Butir kedua adalah yang paling banyak menemukan bug, dan karena itu **jangan ditinggalkan terakhir**:
ia menguji buffer, `uptime_s`, anchor, dan mesin status sekaligus — dan ia kasus pemakaian yang
sebenarnya, bukan kasus tepi. Orang meninggalkan HP-nya. Sejak v1.3 yang lolos lewat jalur itu hanya
`t0` dan index 1; titik `+1 jam`/`+2 jam` menuntut HP hadir, jadi butir ini kini menguji buffer dan
anchor, bukan lagi kelengkapan sesi.

Butir ketiga adalah **butir inti v1.3**, dan satu-satunya yang menguji `ARM_TITIK` di NVS.

---

## 18. Yang sengaja tidak ada di v1

Ditulis eksplisit supaya tidak diam-diam masuk lewat UI.

| Ditunda | Alasan |
|---|---|
| OTA firmware | Butuh infrastruktur sendiri. Bit `kemampuan` sudah disediakan |
| Penjadwal di firmware | Dicabut di v1.3. Jadwal titik ukur hidup di aplikasi sebagai data per sesi, sehingga menggeser atau menambah titik tidak menuntut flash ulang |
| Penundaan waktu di `ARM_TITIK` | Dibuang sebelum implementasi bersama lima mekanisme; penundaan tidak pernah selamat melewati mati-hidup (§5) |
| Melanjutkan sesi lintas reboot | Tidak berlaku lagi sebagai larangan (v1.3): sesi tidak hidup di jam, jadi tidak ada yang perlu dilanjutkan. Yang tetap mustahil tanpa RTC adalah jam mengetahui sudah lewat berapa lama — dan itu sekarang tidak pernah ditanyakan kepadanya |
| Pengukuran terjadwal di luar sesi | Keputusan produk yang sudah dikunci: jam **hanya** mengukur saat sesi makan. `UKUR_SEKARANG` bukan pelanggarannya — yang memicunya jari manusia, bukan timer |
| Notifikasi dari HP ke jam | Bukan bagian dari konsep produk |
| Koreksi drift osilator | Skema anchor sudah menyiapkan tempatnya |
| Satu jam dipakai lintas HP | Anchor hidup di satu HP |
| Multi-sesi bersamaan | Tepat satu sesi aktif |
| Data mentah PPG | Volumenya jauh melampaui BLE dan tidak dipakai UI mana pun |

---

## Bila protokol perlu berubah

Naikkan `versi_minor` (atau `versi_mayor` bila tidak kompatibel mundur) **dan** perbarui dokumen ini
di PR yang sama, di kedua repo. Aplikasi menolak `versi_mayor` yang tidak cocok dengan pesan ke
pengguna, jadi ketidakcocokan akan ketahuan sebagai penolakan yang jelas — bukan sebagai byte yang
salah dibaca di pergelangan tangan orang.
