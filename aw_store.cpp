#include <Arduino.h>
#include <Preferences.h>
#include <esp_timer.h>
#include <string.h>

#include "aw_store.h"

uint32_t aw_uptime_s(void) {
  return (uint32_t)(esp_timer_get_time() / 1000000LL);
}

/* ---------------- Tata letak NVS ----------------
 * Satu namespace, lima kunci. Ring disimpan sebagai satu blob utuh, bukan satu
 * kunci per entri: 64 kunci terpisah berarti 64 operasi NVS tiap kali buffer
 * berubah, dan NVS memang lebih murah menulis satu blob 2,6 KB sekali daripada
 * puluhan nilai kecil berulang kali. */
#define NVS_NS        "asawatch"
#define K_BOOT_ID     "boot"
#define K_RING        "ring"
#define K_ANCHOR      "anchor"
#define K_ANCHORED    "anchored"
#define K_KALIB       "kalib"

/* Blob ring dibubuhi magic + versi. Kalau tata letak aw_entri_t berubah (mis.
 * field baru ditambahkan), blob lama dibuang alih-alih dibaca sebagai sampah:
 * entri dengan padding yang bergeser akan tampak seperti data sah dan mengirim
 * angka kesehatan yang salah ke aplikasi. */
#define RING_MAGIC    0x41573101UL   /* "AW1" + revisi tata letak */

typedef struct {
  uint32_t   magic;
  uint32_t   next_ord;
  uint8_t    next_seq;
  uint8_t    _pad[3];
  aw_entri_t entri[AW_RING_KAP];
} ring_blob_t;

typedef struct {
  uint16_t boot_id;
  uint32_t uptime_s;
  uint32_t epoch_s;
} anchor_rec_t;

/* Daftar boot yang pernah beranchor. 16 cukup: buffer 64 entri setara ~16 sesi,
 * dan satu boot menampung banyak sesi -- entri lintas 16 boot berbeda di satu
 * buffer praktis tidak mungkin. Ring, jadi boot lama terdorong keluar sendiri
 * tanpa perlu dipangkas. */
#define ANCHORED_KAP  16
typedef struct {
  uint8_t  n;
  uint8_t  tulis;                    /* posisi tulis berikutnya */
  uint16_t id[ANCHORED_KAP];
} anchored_t;

typedef struct {
  uint8_t valid;
  int16_t offset_sis;
  int16_t offset_dia;
} kalib_t;

static Preferences  s_nvs;
static bool         s_nvs_ok = false;
static ring_blob_t  s_ring;
static anchored_t   s_anchored;
static kalib_t      s_kalib;
static uint16_t     s_boot_id = 0;
static bool         s_anchor_boot_ini = false;
static bool         s_flag_penuh = false;

/* Tulis flash ditunda: dokumen 11. Satu sesi menghasilkan belasan perubahan dan
 * tidak ada satu pun yang perlu tersimpan dalam milidetik yang sama. */
#define JEDA_SIMPAN_MS  3000
static bool     s_kotor = false;
static uint32_t s_kotor_ms = 0;

static void tandai_kotor(void) {
  s_kotor = true;
  s_kotor_ms = millis();
}

/* ---------------- Init ---------------- */
void aw_store_begin(void) {
  memset(&s_ring, 0, sizeof(s_ring));
  memset(&s_anchored, 0, sizeof(s_anchored));
  memset(&s_kalib, 0, sizeof(s_kalib));
  s_ring.magic    = RING_MAGIC;
  s_ring.next_seq = 1;
  s_ring.next_ord = 1;

  s_nvs_ok = s_nvs.begin(NVS_NS, false);
  if (!s_nvs_ok) {
    /* Jam tetap jalan: pengukuran dan BLE tidak bergantung pada NVS. Yang
     * hilang cuma ketahanan lintas boot, dan itu lebih baik daripada menolak
     * menyala sama sekali. */
    Serial.println("[store] NVS gagal dibuka -- buffer jadi hanya-RAM");
    s_boot_id = 1;
    return;
  }

  /* boot_id naik tepat satu setiap boot dan tidak pernah direset. uint16 cukup:
   * jam yang di-boot sekali sehari baru berputar setelah ~180 tahun. */
  s_boot_id = s_nvs.getUShort(K_BOOT_ID, 0) + 1;
  s_nvs.putUShort(K_BOOT_ID, s_boot_id);

  /* Ring dimuat APA ADANYA, tidak dibersihkan (dokumen 11 aturan 7).
   *
   * Penyangganya static, bukan di stack: ring_blob_t sekitar 2,6 KB, dan
   * loopTask Arduino cuma punya 8 KB. Menaruhnya di stack berarti sepertiga
   * stack terpakai sekejap di dalam setup() yang di baris berikutnya juga
   * menginisialisasi NimBLE -- persis jenis limpahan yang di dokumen 15 disebut
   * "crash yang terlihat acak". Ia hidup sepanjang fungsi ini saja, tetapi
   * ruangnya diambil dari .bss yang memang sudah menampung s_ring. */
  static ring_blob_t tmp;
  size_t n = s_nvs.getBytes(K_RING, &tmp, sizeof(tmp));
  if (n == sizeof(tmp) && tmp.magic == RING_MAGIC) {
    s_ring = tmp;
    if (s_ring.next_seq == 0) s_ring.next_seq = 1;
  } else if (n > 0) {
    Serial.println("[store] tata letak ring berubah -- buffer lama dibuang");
  }

  size_t na = s_nvs.getBytes(K_ANCHORED, &s_anchored, sizeof(s_anchored));
  if (na != sizeof(s_anchored)) memset(&s_anchored, 0, sizeof(s_anchored));

  size_t nk = s_nvs.getBytes(K_KALIB, &s_kalib, sizeof(s_kalib));
  if (nk != sizeof(s_kalib)) memset(&s_kalib, 0, sizeof(s_kalib));

  /* Disalin ke RAM saat boot justru karena membacanya dari NVS di dalam
   * callback onRead terlarang (dokumen 4 & 13.1). */
  s_anchor_boot_ini = aw_boot_punya_anchor(s_boot_id);

  /* Entri yang selamat langsung diantrekan ulang. */
  aw_ring_antre_ulang_semua();

  int tertunda = aw_ring_tertunda();
  Serial.printf("[store] boot_id=%u, %d entri tersisa di buffer, kalibrasi=%s\n",
                (unsigned)s_boot_id, tertunda, s_kalib.valid ? "ada" : "kosong");
}

/* ---------------- Identitas garis waktu ---------------- */
uint16_t aw_boot_id(void)        { return s_boot_id; }
bool     aw_anchor_boot_ini(void) { return s_anchor_boot_ini; }

bool aw_boot_punya_anchor(uint16_t boot_id) {
  for (uint8_t i = 0; i < s_anchored.n; i++)
    if (s_anchored.id[i] == boot_id) return true;
  return false;
}

void aw_simpan_anchor(uint32_t uptime_s, uint32_t epoch_s) {
  anchor_rec_t rec = { s_boot_id, uptime_s, epoch_s };

  if (!aw_boot_punya_anchor(s_boot_id)) {
    s_anchored.id[s_anchored.tulis] = s_boot_id;
    s_anchored.tulis = (uint8_t)((s_anchored.tulis + 1) % ANCHORED_KAP);
    if (s_anchored.n < ANCHORED_KAP) s_anchored.n++;
  }
  s_anchor_boot_ini = true;

  if (s_nvs_ok) {
    /* Record anchor disimpan TERPISAH dari ring buffer dan tidak pernah
     * ditimpa entri baru (dokumen 11 aturan 8). Ditulis langsung, bukan
     * ditunda: anchor datang sekali per koneksi, jadi tidak ada risiko
     * membanjiri flash, dan kehilangannya karena reset mendadak berarti
     * seluruh boot ini kehilangan waktunya. */
    s_nvs.putBytes(K_ANCHOR, &rec, sizeof(rec));
    s_nvs.putBytes(K_ANCHORED, &s_anchored, sizeof(s_anchored));
  }
  Serial.printf("[store] anchor dipasang: boot=%u uptime=%lu epoch=%lu\n",
                (unsigned)s_boot_id, (unsigned long)uptime_s,
                (unsigned long)epoch_s);
}

/* ---------------- Kalibrasi ---------------- */
bool aw_kalibrasi_ada(void) { return s_kalib.valid != 0; }

void aw_kalibrasi_get(int16_t *offset_sis, int16_t *offset_dia) {
  if (offset_sis) *offset_sis = s_kalib.valid ? s_kalib.offset_sis : 0;
  if (offset_dia) *offset_dia = s_kalib.valid ? s_kalib.offset_dia : 0;
}

void aw_kalibrasi_set(int16_t offset_sis, int16_t offset_dia) {
  s_kalib.valid = 1;
  s_kalib.offset_sis = offset_sis;
  s_kalib.offset_dia = offset_dia;
  if (s_nvs_ok) s_nvs.putBytes(K_KALIB, &s_kalib, sizeof(s_kalib));
  Serial.printf("[store] kalibrasi disimpan: %+d/%+d mmHg\n",
                (int)offset_sis, (int)offset_dia);
}

/* ---------------- Ring buffer ---------------- */
static uint8_t seq_berikutnya(void) {
  /* seq berputar 1..255. 0 TIDAK PERNAH dipakai: aplikasi memakainya sebagai
   * "belum pernah menerima apa pun" di SINKRON, dan ACK/NAK memakainya sebagai
   * "bukan entri buffer" (dokumen 11 aturan 1). */
  uint8_t s = s_ring.next_seq;
  s_ring.next_seq = (uint8_t)(s_ring.next_seq + 1);
  if (s_ring.next_seq == 0) s_ring.next_seq = 1;
  return s;
}

/* Slot kosong, atau slot entri TERTUA kalau buffer penuh. Buang-yang-tertua
 * bukan menimpa diam-diam: flag penuh dinyalakan supaya pemanggil mengirim
 * event BUFFER_PENUH. */
static aw_entri_t *slot_untuk_entri_baru(void) {
  for (int i = 0; i < AW_RING_KAP; i++)
    if (!s_ring.entri[i].dipakai) return &s_ring.entri[i];

  aw_entri_t *tertua = &s_ring.entri[0];
  for (int i = 1; i < AW_RING_KAP; i++)
    if (s_ring.entri[i].ord < tertua->ord) tertua = &s_ring.entri[i];

  s_flag_penuh = true;
  Serial.printf("[ring] penuh, entri tertua (seq %u) dibuang\n",
                (unsigned)tertua->seq);
  return tertua;
}

static uint8_t tambah(uint8_t jenis, const uint8_t *sesi_id, uint8_t index_,
                      uint32_t uptime_s, uint16_t gula, uint8_t bpm,
                      uint8_t sis, uint8_t dia, uint8_t spo2) {
  aw_entri_t *e = slot_untuk_entri_baru();
  memset(e, 0, sizeof(*e));
  e->dipakai  = 1;
  e->seq      = seq_berikutnya();
  e->jenis    = jenis;
  e->index_   = index_;
  if (sesi_id) memcpy(e->sesi_id, sesi_id, 16);
  e->boot_id  = s_boot_id;
  e->uptime_s = uptime_s;
  e->gula = gula; e->bpm = bpm; e->sis = sis; e->dia = dia; e->spo2 = spo2;
  e->perlu_dikirim = 1;
  e->ord = s_ring.next_ord++;
  tandai_kotor();
  return e->seq;
}

uint8_t aw_ring_tambah_sampel(const uint8_t *sesi_id, uint8_t index,
                              uint32_t uptime_s, uint16_t gula, uint8_t bpm,
                              uint8_t sis, uint8_t dia, uint8_t spo2) {
  return tambah(0, sesi_id, index, uptime_s, gula, bpm, sis, dia, spo2);
}

uint8_t aw_ring_tambah_event(uint8_t jenis, const uint8_t *sesi_id,
                             uint8_t payload, uint32_t uptime_s) {
  /* ACK dan NAK TIDAK PERNAH masuk ring buffer -- dokumen 7. Aplikasi tidak
   * pernah meng-ACK_EVENT sebuah balasan, jadi ACK yang menunggu di-ack tidak
   * akan pernah dibersihkan: buffer terisi penuh oleh ACK basi lalu mulai
   * membuang sampel sungguhan, dan gejalanya baru muncul puluhan perintah
   * kemudian, jauh dari penyebabnya. Penjaga ini ada supaya kesalahan itu
   * berhenti di baris log, bukan di data pengguna. */
  if (jenis == AW_EV_ACK || jenis == AW_EV_NAK) {
    Serial.println("[ring] BUG: ACK/NAK tidak boleh masuk ring buffer");
    return 0;
  }
  return tambah(jenis, sesi_id, payload, uptime_s, 0, 0, 0, 0, 0);
}

aw_entri_t *aw_ring_berikutnya(void) {
  aw_entri_t *pilih = NULL;
  for (int i = 0; i < AW_RING_KAP; i++) {
    aw_entri_t *e = &s_ring.entri[i];
    if (!e->dipakai || !e->perlu_dikirim) continue;
    if (!pilih || e->ord < pilih->ord) pilih = e;   /* tertua dulu */
  }
  return pilih;
}

void aw_ring_tandai_terkirim(aw_entri_t *e) {
  if (!e) return;
  e->pernah_dikirim = 1;
  e->perlu_dikirim  = 0;
  tandai_kotor();
}

void aw_ring_ack(uint8_t seq) {
  for (int i = 0; i < AW_RING_KAP; i++) {
    if (s_ring.entri[i].dipakai && s_ring.entri[i].seq == seq) {
      s_ring.entri[i].dipakai = 0;
      tandai_kotor();
      return;
    }
  }
  /* Ack untuk seq yang sudah tidak ada bukan error (dokumen 5): entri itu
   * mungkin sudah di-ack sebelumnya, atau terbuang karena buffer penuh. */
}

void aw_ring_antre_ulang_semua(void) {
  bool ada = false;
  for (int i = 0; i < AW_RING_KAP; i++) {
    if (s_ring.entri[i].dipakai) {
      s_ring.entri[i].perlu_dikirim = 1;
      ada = true;
    }
  }
  if (ada) tandai_kotor();
}

uint8_t aw_ring_tertunda(void) {
  uint8_t n = 0;
  for (int i = 0; i < AW_RING_KAP; i++)
    if (s_ring.entri[i].dipakai) n++;
  return n;
}

bool aw_ring_ambil_flag_penuh(void) {
  bool f = s_flag_penuh;
  s_flag_penuh = false;
  return f;
}

void aw_ring_simpan_jika_perlu(void) {
  if (!s_kotor) return;
  if ((uint32_t)(millis() - s_kotor_ms) < JEDA_SIMPAN_MS) return;
  s_kotor = false;
  if (!s_nvs_ok) return;
  s_nvs.putBytes(K_RING, &s_ring, sizeof(s_ring));
}
