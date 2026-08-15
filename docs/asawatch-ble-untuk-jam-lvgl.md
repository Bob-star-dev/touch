# AsaWatch — protokol BLE dan cara kerjanya

Spesifikasi lengkap lapisan BLE jam tangan AsaWatch, untuk diimplementasikan di firmware jam
berbasis LVGL yang sudah ada.

Berkas ini **berdiri sendiri**: seluruh tabel byte, opcode, dan kode error ada di sini. Tidak ada
rujukan ke berkas lain yang perlu dibuka. Salin ke repo firmware Anda (mis. `docs/`), lalu tunjuk
dari `CLAUDE.md` supaya agen menemukannya.

Berkas ini **normatif**. Sisi aplikasi Flutter sudah selesai dan sudah diuji terhadap tabel-tabel di
bawah; bila kode firmware dan dokumen ini berbeda, kodenya yang bug. Versi kawat: **v1.1**.

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

**4. Seluruh penjadwalan memakai `uptime_s`, tidak pernah wall clock.** Justru karena tidak ada RTC,
jadwal sesi (`t0+1 jam`, `t0+2 jam`) menjadi lebih sederhana dan lebih tahan banting.

**5. Pengiriman at-least-once dengan ack eksplisit.** Duplikat adalah perilaku normal, bukan error.
Aplikasi sudah men-dedup dengan kunci `(sesiId, index)` — **jangan menambahkan anti-duplikat di
firmware.**

**6. Versi protokol dinegosiasikan sejak byte pertama.** App lama + firmware baru harus gagal dengan
pesan jelas, bukan salah membaca byte.

**Batasan perangkat keras yang membentuk seluruh dokumen: jam tidak punya RTC.** Ia tidak punya cara
apa pun mengetahui jam berapa sekarang, dan kehilangan seluruh pengetahuan waktu setiap kali daya
putus.

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

### 2.2 Anchor

Pada **setiap koneksi**, setelah handshake, aplikasi menulis `ANCHOR_WAKTU` (opcode `0x01`) berisi
epoch UTC saat itu dan `boot_id` yang baru saja dibacanya. Jam mencatat ke NVS:

```
anchor = (boot_id, uptime_s saat perintah diterima, epoch_s dari aplikasi)
```

`boot_id` disertakan supaya jam bisa mem-NAK bila ia sempat reboot antara handshake dan write —
tanpa itu, anchor bisa terpasang pada garis waktu yang salah.

**Perintah yang sama juga menyetel jam dinding di layar jam.** Epoch UTC di dalamnya adalah jam HP,
dan HP-lah yang paling sering ada di dekat jam tangan — jauh lebih sering daripada Wi-Fi rumah yang
dibutuhkan NTP. Karena itu firmware LVGL menerapkannya ke RTC PCF85063 (setelah digeser ke zona
waktu perangkat) begitu perintah ini masuk, jadi jam tangan ikut benar tanpa Wi-Fi. Sisi aplikasi
tidak perlu berubah sedikit pun: ia sudah mengirim perintah ini pada setiap koneksi. Kedua peran
anchor tetap terpisah di dalam firmware — pemetaan `uptime_s → epoch` untuk entri buffer tidak
bergantung pada jam dinding, dan sebaliknya.

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
**Complete Local Name** (`AsaWatch <4 hex terakhir serial>`, mis. `AsaWatch 3F1A`) dan
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

Interval iklan: **100 ms selama 60 detik pertama** setelah pairing dipicu, lalu **1000 ms**.

```cpp
NimBLEAdvertisementData data_iklan;
data_iklan.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
data_iklan.setCompleteServices(NimBLEUUID(UUID_SERVICE));   // WAJIB di sini

NimBLEAdvertisementData data_scan;
data_scan.setName(nama);                                    // "AsaWatch 3F1A"
std::vector<uint8_t> md = {0xFF, 0xFF, VERSI_MAYOR};        // 0xFFFF = company id uji
data_scan.setManufacturerData(md);

NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
adv->setAdvertisementData(data_iklan);
adv->setScanResponseData(data_scan);
adv->enableScanResponse(true);
```

Jam **selalu kembali beriklan** setelah putus, dan **tidak pernah memutus koneksi sendiri** saat
idle — setiap koneksi yang terbentuk adalah kesempatan memasang anchor.

### 3.3 Serial perangkat

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
| 1 | 1 | `versi_minor` | `1` |
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

**`UKUR`** — dipakai untuk baseline (index 0) saat shutter kamera ditekan, sebelum `t0` ada. Karena
itu ia membawa `sesiId` yang sama dengan `ARM_SESI`, dan **`ARM_SESI` selalu mendahuluinya**. Jam
hanya melayani `UKUR` dalam status ARMED atau RUNNING; permintaan yang tiba selagi IDLE ditolak
dengan NAK `0x03`.

**`UKUR_SEKARANG`** — pengukuran kalibrasi di luar sesi mana pun. Dijawab **paket Sampel biasa**
dengan `sesiId` **16 byte nol** dan `index` 0. ACK-nya tetap dikirim seperti biasa, mendahului paket
Sampel-nya.

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
**3** di `t0+2 jam`.

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

`A5A70006`, Read + Notify, **8 byte**.

| Offset | Ukuran | Field |
|---|---|---|
| 0 | 1 | `status_sesi`: 0 idle, 1 armed, 2 running |
| 1 | 1 | `sampel_tertunda` — jumlah entri belum di-ack di buffer |
| 2 | 1 | `baterai` % |
| 3 | 1 | `flag`: bit0 sedang mengukur, bit1 kalibrasi tersimpan, bit2 baterai kritis, bit3 boot ini sudah punya anchor |
| 4 | 4 | `uptime_s` uint32 LE |

Dikirim sebagai notifikasi setiap kali salah satu isinya berubah (ARM, tombol, mulai/selesai ukur,
anchor tersimpan, kalibrasi tersimpan, kembali ke IDLE), dan disegarkan juga saat dibaca.

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
NimBLEDevice::setSecurityAuth(true, true, true);              // bonding, MITM, SC
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

### Tulis flash ditunda

Satu sesi menghasilkan belasan perubahan dan tidak ada satu pun yang perlu tersimpan dalam
milidetik yang sama. Tandai "kotor", lalu tulis sekali setelah ~3 detik tenang. Menulis flash sambil
BLE aktif bisa memblokir cukup lama untuk mengganggu jadwal koneksi — dan di firmware LVGL, cukup
lama untuk menjatuhkan frame. Yang perlu dijaga adalah godaan menulis setiap perubahan status.

---

## 12. Mesin status sesi

```
     ┌──────┐  ARM_SESI          ┌───────┐  tombol ditekan   ┌─────────┐
     │ IDLE │ ─────────────────▶ │ ARMED │ ────────────────▶ │ RUNNING │
     └──────┘                    └───────┘                   └─────────┘
        ▲                            │  timeout 4 jam            │
        │                            │  / BATAL_SESI             │ sampel index 3
        └────────────────────────────┴───────────────────────────┘  terkirim

     ANCHOR_WAKTU boleh masuk di status mana pun — ia tidak menyentuh mesin ini.
```

- **IDLE** — tombol "Selesai Makan" tidak berfungsi. `UKUR_SEKARANG` (kalibrasi) tetap boleh.
- **ARMED** — tombol aktif. `UKUR` index 0 (baseline) dilayani. **Timeout 4 jam** `uptime_s`: lewat
  dari itu kirim `SESI_KEDALUWARSA` dan kembali IDLE. Tanpa ini, foto sarapan yang tombolnya tidak
  pernah ditekan akan menyalakan tombol sampai malam.
- **RUNNING** — index 1 diukur segera saat tombol ditekan; jam menjadwalkan sendiri index 2 pada
  `t0.uptime_s + 3600` dan index 3 pada `+ 7200`.
- Sampel index 3 terkirim → IDLE. Sampel yang belum di-ack tetap di buffer.

**Jadwal menyalakan panggilan, bukan sensor.** Firmware LVGL tidak menyalakan MAX30105 sendiri saat
index 2 dan 3 jatuh tempo: ia menandai pengukuran itu "menunggu tombol", menampilkannya di baris
status, dan pengukuran baru berjalan saat pengguna menekan tombol — dengan jarinya sudah menempel di
sensor. Sensor optik tidak menghasilkan apa pun tanpa kulit, jadi pengukuran yang menyala sendiri
saat jam tergeletak di meja hanya menghasilkan `UKUR_GAGAL` setelah 90 detik menunggu kontak.

Yang berubah hanya **pemicunya**; kawatnya tidak. Index, `sesiId`, jenis entri, dan urutan sampel
tetap persis seperti di tabel-tabel di atas, dan `UKUR` dari aplikasi tetap dilayani kapan pun.
Masa tunggunya dibatasi supaya sesi tidak menggantung: index 2 hangus saat index 3 jatuh tempo,
index 3 hangus satu jam setelah jatuh tempo, dan **keduanya dicatat `UKUR_GAGAL` dengan payload
index** — sama seperti pengukuran yang berjalan lalu gagal total, karena bagi aplikasi keduanya
memang peristiwa yang sama: index itu tidak punya sampel dan tidak akan pernah punya. Index 3 yang
hangus menutup sesi persis seperti sampel index 3 yang terkirim.

Dua hal yang wajib dipegang:

**1. Jadwal dihitung dari `uptime_s` absolut milik `t0`, tidak pernah dari "sisa waktu".** Sisa waktu
yang diakumulasikan akan hanyut setiap kali ada penundaan.

```cpp
if (!(index_selesai & (1 << 2)) && sekarang >= t0_uptime + 3600) menunggu_tombol = 2;
if (!(index_selesai & (1 << 3)) && sekarang >= t0_uptime + 7200) menunggu_tombol = 3;
// tombol ditekan -> ukur(menunggu_tombol); sampel/hangusnya index 3 -> ke_idle()
```

**2. Reboot saat RUNNING mengakhiri sesi.** `uptime_s` kembali nol dan `t0` lama tidak bisa
dibandingkan lagi. Kembali ke IDLE; sampel yang terlanjur ada tetap di buffer dengan `boot_id`
lamanya, dan aplikasi menutup sesi itu sebagai tidak lengkap. **Jangan mencoba melanjutkan sesi
lintas boot** — tanpa RTC itu tidak mungkin, dan yang dihasilkan hanya data yang tampak sah tetapi
salah.

**Seluruh siklus harus selesai walau HP tidak pernah tersambung sekali pun.** Jam tidak menunggu
konfirmasi aplikasi untuk berpindah status.

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

Urutannya penting: perintah masuk dulu, jadwal, lalu pengiriman, lalu tulis flash.

```c
void jam_putar() {
  putar_perintah_ble();        // ambil dari antrean, jalankan opcode (§5)
  putar_sesi();                // timeout ARM, jadwal index 2 dan 3 (§12)

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

void jam_tekan_tombol();   // dari lv_event_cb tombol "Selesai Makan"

// Pembaca, semuanya murni RAM dan murah:
uint8_t  jam_status();            // 0 IDLE, 1 ARMED, 2 RUNNING
bool     jam_sedang_mengukur();
uint32_t jam_t0_uptime();         // 0 bila belum RUNNING
uint32_t jam_uptime();
uint8_t  jam_tertunda();          // entri belum di-ack
bool     jam_terhubung();
bool     jam_siap_notifikasi();   // terhubung DAN dilanggani
bool     jam_ada_anchor();
```

Urutan di `jam_mulai()` yang tidak boleh dibalik: NVS dulu → `boot_id` naik → muat ring (**tanpa
dibersihkan**) → status sesi dipaksa IDLE (reboot mengakhiri sesi) → `ble_mulai()` → catat event
`BOOT` ke buffer.

---

## 14. Apa yang ditampilkan UI

Bukan spesifikasi layar — rancangannya milik Anda. Ini daftar keadaan yang **harus punya tampilan**,
karena tanpanya pengguna tidak bisa membedakan jam yang bekerja dari jam yang rusak.

**Tombol "Selesai Makan"**

- **IDLE**: mati/redup. Menekannya tidak menghasilkan apa-apa selain umpan balik "belum disiapkan"
  (getaran pendek + pesan). Jangan "memperbaikinya" menjadi selalu aktif — inilah yang menjamin
  tidak ada sesi tanpa foto makanan. Umpan baliknya wajib ada: pengguna harus tahu **kenapa**
  tombolnya diam.
- **ARMED**: aktif dan menonjol. Ini keadaan yang pengguna tunggu.
- **RUNNING**: ganti dengan tampilan kemajuan sesi.

**Sesi berjalan.** Hitung mundur ke index 2 dan 3 dari `jam_t0_uptime() + 3600 − jam_uptime()` —
selalu dari `uptime_s` absolut, tidak pernah dari sisa waktu yang diakumulasikan sendiri.

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
tidak punya masalah ini.

**MTU dan pemecahan paket.** Sampel 31 byte harus utuh dalam satu notifikasi.

**Membersihkan buffer saat boot.** Entri lintas boot hidup berdampingan.

**Melanjutkan sesi lintas reboot.** Tidak mungkin tanpa RTC.

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
Pembacaan sensor_baca(uint8_t index);   // 0 baseline, 1 t0, 2 +1 jam, 3 +2 jam
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
perintah untuk menyuntik opcode Kontrol persis seperti aplikasi menulisnya. Tambahkan pengali
jadwal (`skala`) supaya siklus 2 jam selesai dalam 2 menit — tanpa itu setiap iterasi pengujian
memakan dua jam. Pengalinya tidak boleh mengubah **cara** jadwal dihitung, hanya angkanya.

### Checklist firmware

- [ ] Service UUID ada di **paket iklan**, nama + versi mayor di scan response
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
- [ ] Tombol mati di IDLE, hidup di ARMED, dengan umpan balik saat ditekan di IDLE
- [ ] ARM timeout 4 jam menghasilkan `SESI_KEDALUWARSA`
- [ ] Reboot saat RUNNING → kembali IDLE, sampel lama tetap di buffer dengan `boot_id` lama
- [ ] Ring buffer bertahan melewati reset dan memuat entri lintas boot
- [ ] Entri kirim-ulang menyalakan flag `dariBuffer`
- [ ] Buffer penuh mengirim `BUFFER_PENUH`, bukan menimpa diam-diam
- [ ] Metrik gagal dikirim sebagai `0`
- [ ] Offset kalibrasi bertahan melewati boot
- [ ] Pairing Just Works + LE Secure Connections, enkripsi di kelima karakteristik
- [ ] Siklus sesi penuh selesai **tanpa HP tersambung sama sekali**
- [ ] LVGL tetap responsif selama pengiriman buffer, penulisan flash, dan pengukuran

### Uji integrasi

- [ ] Satu sesi penuh (baseline → t0 → +1 jam → +2 jam) dengan hardware nyata
- [ ] **Bluetooth HP dimatikan sebelum foto diambil, dinyalakan setelah +2 jam** → seluruh sesi
      diterjemahkan dengan benar dari satu anchor di akhir
- [ ] Jam di-reboot di tengah sesi → sesi berakhir tidak lengkap, sampel sebelum reboot tetap masuk
      dengan waktu yang benar
- [ ] Jam kehabisan daya, boot, dipakai satu sesi penuh, mati lagi, baru tersambung → sesi itu
      ditandai `waktu_tidak_pasti`

Butir kedua adalah yang paling banyak menemukan bug, dan karena itu **jangan ditinggalkan terakhir**:
ia menguji buffer, `uptime_s`, anchor, dan mesin status sekaligus — dan ia kasus pemakaian yang
sebenarnya, bukan kasus tepi. Orang meninggalkan HP-nya.

---

## 18. Yang sengaja tidak ada di v1

Ditulis eksplisit supaya tidak diam-diam masuk lewat UI.

| Ditunda | Alasan |
|---|---|
| OTA firmware | Butuh infrastruktur sendiri. Bit `kemampuan` sudah disediakan |
| Melanjutkan sesi lintas reboot | Tidak mungkin tanpa RTC |
| Pengukuran terjadwal di luar sesi | Keputusan produk yang sudah dikunci: jam **hanya** mengukur saat sesi makan |
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
