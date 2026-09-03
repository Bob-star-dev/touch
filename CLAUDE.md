# AsaWatch — firmware jam tangan (ESP32-C6 + LVGL 8.3)

Sketch Arduino untuk **Waveshare ESP32-C6-Touch-LCD-1.69** (240x280, ST7789V2 +
CST816T). Jam ini adalah **sensor + buffer + penampil**: ia mengukur PPG,
menyimpan hasilnya di ring buffer NVS, dan mengirimkannya ke aplikasi Flutter
lewat BLE. Semua kecerdasan (verdict, tren, penjadwalan sesi) ada di aplikasi,
bukan di sini.

Direktori ini adalah **firmware yang aktif**. `/home/rad/Arduino/jam_ble_check`
adalah peninggalan lama — jangan disunting.

---

## Build & flash

`arduino-cli` ada di `~/.local/bin` (belum ada di PATH shell user), dan
sketchbook-nya **terpisah** dari `~/Arduino` karena `~/Arduino/libraries`
memuat lvgl 9.3.0 yang tidak cocok dengan sketch LVGL 8.3 ini.

```bash
export PATH="$HOME/.local/bin:$PATH"
export ARDUINO_DIRECTORIES_USER="/home/rad/project/Project Enuma/arduino/sketchbook"
FQBN=esp32:esp32:esp32c6:CDCOnBoot=cdc,PartitionScheme=huge_app
BUILD=/tmp/aw-build      # build-path sendiri, JANGAN cache bawaan

arduino-cli compile -b "$FQBN" --build-path "$BUILD" "/home/rad/project/Project Enuma/arduino/touch"
arduino-cli upload  -b "$FQBN" -p /dev/ttyACM0 --input-dir "$BUILD" "/home/rad/project/Project Enuma/arduino/touch"
```

Tiga hal yang sudah memakan waktu dan tidak boleh diulang:

1. **`PartitionScheme=huge_app` WAJIB.** Sketch ~1,89 MB melebihi partisi app
   bawaan 1,31 MB.
2. **Pakai `--build-path` sendiri.** Arduino IDE memakai direktori cache build
   dengan hash yang sama dan menyapu artefaknya di tengah jalan.
3. **Tutup Serial Monitor sebelum upload.** `/dev/ttyACM0` hanya boleh dipegang
   satu proses. Untuk menunggu proses selesai jangan pakai
   `pgrep -f '<pola>'` — polanya cocok dengan baris perintah penunggunya
   sendiri; pakai `lsof` pada device atau cek artefak.

Core: `esp32:esp32@3.3.11` (3.0.2 masih terpasang berdampingan tetapi
BLE-nya Bluedroid — tanpa `PROPERTY_READ_ENC`/`onSubscribe`, jadi tidak bisa
dipakai). Library di sketchbook: lvgl 8.3.11 + `lv_conf.h` (16bpp,
`LV_TICK_CUSTOM=1`, Montserrat 12/26/30), GFX Library for Arduino 1.6.7,
SensorLib 0.4.1, SparkFun MAX3010x 1.1.2, ArduinoJson 7.4.2.

---

## Dokumen protokol itu NORMATIF

`docs/asawatch-ble-untuk-jam-lvgl.md` — **protokol kawat v1.3**. Sisi Flutter
sudah selesai dan sudah diuji terhadap tabel-tabel di dalamnya, jadi **kalau
kode dan dokumen berbeda, kodenya yang bug.** Jangan mengubah satu angka pun di
`aw_proto.h` tanpa mengubah dokumen dan versinya di PR yang sama.

Berkas di akar direktori:

| berkas | isi |
|---|---|
| `asawatch-ble-untuk-jam-lvgl-1.md` | salinan v1.3, identik dengan `docs/` |
| `asawatch-ble-untuk-jam-lvgl.md` | **v1.2, USANG** — jangan dijadikan acuan |

Referensi bergaya "dokumen 12 poin 4" yang bertebaran di komentar kode menunjuk
ke nomor bagian dokumen v1.3 itu.

---

## Struktur

```
touch.ino          UI LVGL satu halaman, pin map, tombol PWR/BOOT, setup/loop,
                   konsol serial penyuntik opcode
config.h           Wi-Fi, API key OWM, zona waktu, interval, kalibrasi baterai

aw_proto.h         definisi kawat: UUID, opcode, event, NAK, ukuran paket,
                   penulis/pembaca little-endian, aw_uptime_s()
aw_ble.{h,cpp}     lapisan BLE NimBLE 2.x — SENGAJA tipis: iklan, GATT, dan
                   memindahkan byte antar-task. Tidak tahu apa-apa soal sesi.
aw_store.{h,cpp}   NVS: boot_id, anchor waktu, daftar boot beranchor, ring
                   buffer 64 entri, offset kalibrasi tensi, titik ter-ARM
aw_jam.{h,cpp}     mesin status sesi, rutin pengukuran, pengirim buffer.
                   "Logika jam", dan sengaja setipis mungkin.

ppg.{h,cpp}        MAX30105/30102 -> BPM, SpO2, glukosa + tensi EKSPERIMENTAL
rtc.{h,cpp}        PCF85063 (I2C 0x51)
time_manager.{h,cpp} pemilik tunggal "jam sekarang" (RTC / NTP / BLE anchor)
net.{h,cpp}        Wi-Fi + task latar yang menjalankan NTP dan cuaca
                   (MATI secara bawaan: AW_PAKAI_WIFI 0 di config.h)
weather.{h,cpp}    OpenWeatherMap
battery.{h,cpp}    ADC1 GPIO0 + median + EMA + kurva Li-Po

ui_assets.h        aset UI, DIHASILKAN genassets.py — jangan disunting tangan
splash_assets.h    logo splash, DIHASILKAN gensplash.py — sda
font_digits_48.c   subset digit montserrat_48 (font_digits_46.c tidak dipakai)
tools/ble-test/index.html  penguji BLE Web Bluetooth, berdiri sendiri
```

---

## Aturan konteks/thread — langgar ini dan bug-nya tidak deterministik

Hanya ada **dua** thread yang penting, dan pembagiannya kaku:

**Konteks loop()** — `lv_timer_handler`, `touch_poll`, `jam_putar`, seluruh
akses I2C (touch, RTC, MAX30105), seluruh akses flash/NVS.

**Task jaringan** (`net.cpp`) — `WiFi.begin`, NTP, HTTP GET. TIDAK menyentuh
I2C, TIDAK menyentuh LVGL. Menitipkan hasil lewat `weather_get()` /
`tm_submit_ntp()`. Satu-satunya yang dibacanya dari luar adalah
`aw_ble_terhubung()` dan `aw_ble_jumlah_tertunda()` — keduanya salinan satu
byte/bool milik `aw_ble`, bukan struktur hidup. **Jangan memanggil
`aw_ring_tertunda()` dari sini**: ia menyusuri 64 slot yang sedang diubah
konteks loop.

Task ini **tidak berjalan sama sekali secara bawaan** — `AW_PAKAI_WIFI 0`.
Wi-Fi hanya melayani NTP dan cuaca, dan NTP sudah kalah berguna dibanding
`ANCHOR_WAKTU` yang datang di setiap koneksi. Yang dibayarnya jauh lebih mahal:
asosiasi yang gagal berulang menyapu 2,4 GHz, dan BLE kehilangan jendela RX
untuk connect request — jam terlihat saat memindai lalu selalu timeout saat
disambungkan. Kalau dinyalakan lagi, gerbang "BLE masih punya pekerjaan" dan
backoff berlipat di `net_task` adalah yang membuatnya aman; jangan dilepas.

**Task host NimBLE** (callback `onWrite`/`onRead`/`onSubscribe`/`onConnect`) —
DILARANG menyentuh LVGL (gejalanya render korup sekali dalam sepuluh menit),
NVS/flash (gejalanya koneksi putus-putus di bawah beban), dan ring buffer. Yang
boleh dilakukan `onWrite` hanyalah menaruh byte mentah ke antrean FreeRTOS;
`aw_jam.cpp` yang mengambilnya di konteks loop. Pengecualian yang eksplisit:
`aw_boot_id()`, `aw_anchor_boot_ini()`, `aw_boot_punya_anchor()` membaca
salinan RAM sehingga aman dari `onRead`.

Konsekuensinya `aw_jam.cpp` berjalan di task yang sama dengan LVGL — itu
disengaja: callback tombol boleh memanggil `jam_tekan_tombol()` langsung dan UI
boleh membaca variabel sesi langsung, tanpa mutex dan tanpa antrean kedua.
Satu-satunya yang menyeberang task adalah antrean perintah BLE.

Bus I2C 100 kHz, **bukan** 400 kHz — CST816T tidak stabil di 400 kHz di board
ini. `ppg.cpp` karena itu tidak boleh memanggil `begin(Wire, I2C_SPEED_FAST)`.

---

## Invarian yang tidak boleh dibalik

- **Urutan init `setup()`**: latch `BAT_EN` HIGH paling awal (tanpa itu board
  mati begitu jari lepas dari tombol PWR) → gerbang tahan-3-detik → touch
  `begin()` **sebelum** `gfx->begin()` (kalau dibalik, CST816T mengunci baseline
  salah dan melaporkan ghost touch permanen, terukur A/B) → LVGL → sensor →
  splash → `jam_mulai()` → `net_begin()` (AsaWatch dulu baru Wi-Fi, demi heap).
- **Urutan `jam_mulai()`**: NVS → boot_id naik → muat ring (tanpa dibersihkan)
  → status sesi dipaksa IDLE → BLE mulai → catat event BOOT.
- **Jam tidak menjadwalkan apa pun** (v1.3). `AW_JADWAL_IDX2_S`/`IDX3_S` dihapus
  dan tidak boleh dikembalikan: jam bertahan ~50 menit menyala sedangkan satu
  sesi lebih dari dua jam, jadi ia PASTI mati sebelum titik berikutnya jatuh
  tempo. Yang menjadwalkan adalah aplikasi, lewat `UKUR` dan `ARM_TITIK`.
- **Satu tombol fisik, dua makna.** Pemilihannya ada di `jam_tekan_tombol()`,
  **bukan** di UI. UI hanya membaca `jam_titik_armed()` untuk menuliskan LABEL —
  `ARM_TITIK` bisa tiba lewat BLE antara UI membaca dan jari diangkat.
- **Tiga keadaan BLE, bukan dua.** Tersambung saja tidak cukup untuk mulai
  mengirim; tunggu `aw_ble_siap_notifikasi()` (tersambung DAN kedua CCCD
  ditulis), kalau tidak notifikasi hilang tanpa jejak sementara entrinya
  terlanjur ditandai terkirim.
- **Ack adalah satu-satunya jalan keluar entri dari ring buffer.**
- **Penulisan flash asimetris**: entri baru (sampel & peristiwa) ditulis
  SEKETIKA; hanya ack/penanda-terkirim/antre-ulang yang boleh ditunda 3 detik.
  Pola pemakaian v1.3 adalah nyalakan-ukur-MATIKAN, jadi daya putus di dalam
  jendela 3 detik itu yang diharapkan terjadi, bukan kasus tepi.
- **`BLESecurity::setAuthenticationMode(true, false, true)`** — MITM sengaja
  false. Dengan `NO_INPUT_OUTPUT` MITM tidak akan pernah tercapai; memintanya
  true hanya menghasilkan "pairing kadang gagal". Jangan dikembalikan ke true.
- **Pengukuran selesai karena DATANYA cukup, bukan karena waktunya habis.**
  Gerbangnya `UKUR_MIN_DETAK` (10) + `UKUR_MIN_MS` (10 dtk); dua batas waktu
  yang ada (`UKUR_TANPA_KONTAK_MS`, `UKUR_BATAS_KERAS_MS`) adalah penjaga daya,
  bukan batas kesabaran.
- `AW_KAPASITAS_BUFFER` (aw_proto.h) dan `AW_RING_KAP` (aw_store.h) harus sama.

---

## Menguji tanpa HP

Konsol serial di `touch.ino` menyuntik opcode **lewat antrean yang sama** dengan
tulisan BLE (`aw_ble_suntik_perintah`) — itu syarat, bukan kenyamanan: memanggil
rutin internal `aw_jam` akan melewati handler-nya sehingga cabang NAK dan
idempotensi tidak pernah teruji.

```
arm [1|2]    ARM_SESI dengan sesiId uji
batal        BATAL_SESI
mulai        MULAI_SESI   (jalankan dua kali untuk menguji idempotensi)
ukur <idx>   UKUR index idx
titik <idx>  ARM_TITIK index idx
now          UKUR_SEKARANG
tombol       tekan tombol fisik (bukan BLE — menguji dispatcher satu tombol)
status       cetak keadaan jam
id <1-99>    atur label uji manual (bukan protokol) — nama BLE jadi
             "AsaWatch NN" alih-alih suffix hex MAC, disimpan di NVS,
             kepakai setelah boot ulang. Berguna kalau banyak jam identik
             disambungkan berurutan dan suffix hex MAC-nya kebetulan sama
             (unit dari batch produksi yang sama — TERBUKTI terjadi pada
             batch 10 unit pengujian, bukan cuma teori: bahkan suffix
             6-hex/3-byte pun masih bisa bertabrakan).
```

Satu sesi utuh: `arm` → `ukur 0` → `tombol` → `titik 2` → `tombol` →
`titik 3` → `tombol`.

Alternatif dari browser: `tools/ble-test/index.html` (Web Bluetooth).

---

## Konvensi

- **Kode, komentar, dan identifier berbahasa Indonesia.** Modul AsaWatch dan UI
  memakai nama Indonesia (`jam_putar`, `kirim_entri`, `mk_kartu`, `cincin_set`);
  modul perangkat keras yang lebih tua (`ppg`, `battery`, `rtc`,
  `time_manager`, `net`, `weather`) memakai nama Inggris. Ikuti gaya berkas yang
  sedang disunting.
- **Komentar menjelaskan KENAPA, bukan APA**, dan sering memuat angka hasil
  ukur beserta gejala kalau keputusannya dibalik. Ini gaya repo yang disengaja —
  pertahankan saat menambah kode, dan **jangan hapus komentar panjang** karena
  terlihat verbose; sebagian besar merekam jebakan yang sudah pernah menggigit.
- Semua field multi-byte di paket kawat **little-endian, ditulis byte demi
  byte**. Sengaja tidak ada struct yang di-`memcpy` ke paket.
- Koordinat UI diukur langsung dari `wajah_jam_modular_240x280.png` pada ukuran
  layar sebenarnya. Kalau mengubah tata letak, ukur dari gambar itu.

---

## Aset yang dihasilkan

`ui_assets.h` dan `splash_assets.h` adalah keluaran `genassets.py` dan
`gensplash.py` (butuh Pillow). **Kedua skrip masih memuat path lama
`/home/harjo/Documents/touch`** — perbaiki `SRC`/`OUT` sebelum menjalankannya.
Sunting skripnya, jangan `.h`-nya.

---

## Peringatan & keamanan

- **Nilai glukosa dan tekanan darah EKSPERIMENTAL** dan tidak punya dasar
  fisiologis tervalidasi — model linear placeholder atas fitur PPG (PI, R, HR).
  Jangan dipakai untuk keputusan medis apa pun. Bit `AW_KEMAMPUAN` untuk
  keduanya tetap menyala karena artinya "jam mengirim metrik ini", bukan
  "angkanya layak klinis". Peringatan lengkap di `ppg.h`.
- Koreksi bias situs wrist untuk SpO2 (`SPO2_WRIST_OFFSET = 16.8`) berasal dari
  **satu titik data** — provisional.
- **`config.h` memuat password Wi-Fi dan API key OpenWeatherMap dalam teks
  polos dan saat ini ikut ter-commit.** Jangan mendorong repo ini ke tempat
  publik apa adanya. `OWM_CITY` harus `"Surakarta,ID"` — `"Solo,ID"` menjawab
  HTTP 200 tetapi menunjuk desa Solo di Flores, ~1300 km meleset.
