#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "MAX30105.h"
#include "ppg.h"
#include "config.h"

static MAX30105 sensor;
static bool s_present = false;

/* Mulai MATI: LED baru menyala saat pengguna menekan tombol daya di home.
 * Lihat alasan lengkapnya di ppg.h. */
static bool s_enabled = false;

/* ================= Konfigurasi sensor =================
 * Di pergelangan tangan (radial artery) sinyal jauh lebih lemah daripada di
 * ujung jari, sehingga arus LED perlu lebih tinggi dan rata-rata sampel lebih
 * banyak untuk meredam noise.
 *
 * LED_MODE 2 = Red + IR saja. Channel Green TIDAK dipakai: unit yang diuji
 * tidak mengeluarkan cahaya hijau sama sekali (kemungkinan chip sebenarnya
 * MAX30102, yang secara fisik tidak punya LED hijau -- PART_ID keduanya identik
 * dan tidak bisa dibedakan lewat software).
 */
static const byte LED_BRIGHTNESS = 0xFF;   /* sinyal wrist lemah -> arus penuh */
static const byte SAMPLE_AVERAGE = 8;
static const byte LED_MODE       = 2;      /* Red + IR */
static const int  SAMPLE_RATE    = 100;    /* Hz */
static const int  PULSE_WIDTH    = 411;    /* us -> ADC 18-bit */
static const int  ADC_RANGE      = 16384;  /* headroom maksimum: LED_BRIGHTNESS
                                              tinggi mudah saturasi kalau kecil */

/* ---------- Filter DC (EMA) untuk memisahkan komponen AC ---------- */
static const float DC_ALPHA = 0.95f;
static double dcRed = 0, dcIR = 0;
static bool   dcInit = false;

/* ---------- Deteksi kulit menempel ---------- */
static const long IR_PRESENCE_THRESHOLD = 30000;
static bool          wasInContact = false;
static unsigned long contactStartMs = 0;
/* Debounce: tanpa ini, kontak yang goyang sesaat (irVal menyeberang threshold
 * sebentar) langsung dianggap "terlepas" dan me-reset seluruh detektor detak,
 * membuang progres settle yang sudah susah payah tercapai. */
static const unsigned long CONTACT_DEBOUNCE_MS = 250;
static bool          rawContactCandidate = false;
static unsigned long rawContactCandidateSinceMs = 0;

/* Filter DC butuh waktu konvergen dari nilai awal ke nilai sebenarnya. Selama
 * masa ini rasio AC/DC (R, PI) belum representatif -> jangan hitung SpO2 dan
 * glukosa dulu, supaya tidak keluar angka yang mustahil secara fisiologis.
 *
 * 3 detik, turun dari 8. Angkanya dihitung, bukan dikira: FIFO mengeluarkan
 * 100/SAMPLE_AVERAGE = 12,5 sampel per detik, dan EMA dengan DC_ALPHA 0,95 punya
 * konstanta waktu 1/(1-0,95) = 20 sampel = 1,6 detik. Delapan detik berarti lima
 * konstanta waktu -- jauh lebih lama daripada yang dibutuhkan, apalagi karena
 * dcRed/dcIR di-seed dari sampel pertama sehingga transiennya kecil sejak awal.
 * Tiga detik masih hampir dua konstanta waktu (sisa transien <15%), sementara
 * lima detik yang dihemat adalah lima detik pengguna menatap layar kosong. */
static const unsigned long DC_SETTLE_MS = 3000;
static const byte MIN_STABLE_BEATS = 3;
static byte stableBeatCount = 0;

/* Pencacah detak yang TIDAK dibatasi, berbeda dari stableBeatCount di atas.
 * stableBeatCount berhenti di MIN_STABLE_BEATS karena tugasnya cuma satu:
 * menjadi gerbang biner "sudah ada detak yang konsisten atau belum". Yang ini
 * terus mencacah, karena pemakainya butuh tahu SUDAH BERAPA BANYAK -- rutin
 * pengukuran AsaWatch menunggu sejumlah detak sebelum menganggap satu
 * pengukuran layak diakhiri, dan tiga detak jelas terlalu sedikit untuk itu.
 * Direset bersama detektornya setiap kali kontak kulit hilang. */
static uint16_t beatCount = 0;

/* ---------- Batas fisiologis untuk membuang window "noise" ----------
 * Formula Maxim AN6409 hanya valid di rentang sempit R~0.4-1.0 (SpO2 ~100%
 * turun ke ~85%). R sampai 1.15 dipertahankan sebagai toleransi bawah
 * (SpO2~70%, sudah kondisi gawat darurat kalau nyata) -- di atas itu hasilnya
 * cuma noise, bukan estimasi SpO2 yang valid. */
static const double R_MIN_PLAUSIBLE  = 0.3;
static const double R_MAX_PLAUSIBLE  = 1.15;
static const float  PI_MIN_PLAUSIBLE = 0.02f;   /* % */
static const float  PI_MAX_PLAUSIBLE = 15.0f;   /* % di atas ini artefak gerak */
static const float  SPO2_MIN_PLAUSIBLE = 70.0f; /* manusia hidup selalu >=70% */

/* ---------- KOREKSI BIAS SITUS WRIST UNTUK SpO2 (PROVISIONAL, 1 TITIK DATA) --
 * PERINGATAN: ini BUKAN kalibrasi statistik -- cuma koreksi offset dari SATU
 * perbandingan (2026-07-29): sesi wrist menghasilkan SpO2 mentah ~83.1%
 * (mustahil secara fisiologis), sedangkan sesi jari di waktu berdekatan
 * menghasilkan ~99.9% (formula AN6409 divalidasi untuk geometri jari). Offset
 * di bawah menggeser hasil wrist mendekati acuan jari itu, TAPI tidak ada
 * jaminan offset konstan ini benar untuk kondisi lain. WAJIB diganti dengan
 * koreksi dari banyak titik data (idealnya dibanding pulse oximeter medis di
 * wrist yang sama) begitu tersedia. */
static const float SPO2_WRIST_OFFSET = 16.8f;   /* dari (99.9 - 83.1) */

/* ---------- Model glukosa (SANGAT eksperimental, TIDAK PUNYA DASAR
 * FISIOLOGIS TERVALIDASI, WAJIB dikalibrasi) ----------
 * glukosa_mgdl = G0 + G1*PI + G2*(HR-70) + G3*R
 *
 * G0 sudah digeser dari placeholder awal (90.0) jadi 82.1 berdasarkan koreksi
 * PROVISIONAL 1 titik data (2026-07-29): sesi wrist menghasilkan 108.9 mg/dL
 * dibanding tes darah sungguhan (alat GCHb) 101 mg/dL di waktu berdekatan ->
 * delta -7.9. Ini BUKAN kalibrasi statistik (butuh >=15-20 titik lintas level
 * glukosa berbeda, semua diukur di wrist). G1/G2/G3 SAMA SEKALI belum tersentuh
 * data nyata. Bahkan setelah kalibrasi asli, akurasinya TIDAK BISA DIJAMIN
 * karena tidak ada dasar fisiologis tervalidasi yang menghubungkan rasio
 * Red/IR dengan kadar glukosa darah. */
static double G0 = 82.1, G1 = -15.0, G2 = 0.3, G3 = 20.0;

/* ---------- Model tekanan darah (SANGAT eksperimental -- BELUM PUNYA titik
 * koreksi data nyata sama sekali, beda dengan G0 glukosa di atas yang sudah
 * dapat satu titik dari perbandingan alat sungguhan. Peringatan yang sama
 * berlaku, malah lebih keras: tidak ada sensor tekanan langsung di sini --
 * perangkat medis biasa memakai pulse-transit-time (jeda EKG ke puncak nadi)
 * yang butuh dua sensor terpisah, sementara di sini cuma ada satu PPG.
 * Angka di bawah HANYA regresi placeholder atas fitur sinyal yang sama
 * dipakai glukosa (PI, HR, R), memakai rata-rata tekanan darah dewasa
 * (120/80) sebagai titik tolak. WAJIB dikalibrasi terhadap tensimeter
 * sungguhan sebelum dipakai untuk apa pun, termasuk sekadar dipercaya.
 *
 *   sistol_mmHg  = SBP0 + SBP1*(HR-70) + SBP2*PI + SBP3*R
 *   diastol_mmHg = DBP0 + DBP1*(HR-70) + DBP2*PI + DBP3*R
 */
static double SBP0 = 120.0, SBP1 = 0.3, SBP2 = -3.0, SBP3 = 15.0;
static double DBP0 = 80.0,  DBP1 = 0.2, DBP2 = -2.0, DBP3 = 10.0;

/* ---------- Smoothing antar-window ---------- */
static const int SMOOTH_N = 5;
static float spo2History[SMOOTH_N];
static float glucoseHistory[SMOOTH_N];
static float sbpHistory[SMOOTH_N];
static float dbpHistory[SMOOTH_N];
static byte  smoothIndex = 0, smoothCount = 0;

/* Dua gerbang, bukan satu.
 *
 *   readingsAwal   - window terakhir menghasilkan angka yang masuk akal.
 *                    Cukup untuk DITAMPILKAN.
 *   readingsStable - angka itu juga lahir setelah MIN_STABLE_BEATS detak
 *                    konsisten. Hanya ini yang boleh DIKIRIM ke aplikasi.
 *
 * Dulu keduanya satu bendera, dan akibatnya layar ikut menunggu syarat yang
 * sebenarnya milik protokol. Lihat ppg.h pada field `awal`. */
static bool  readingsAwal = false;
static bool  readingsStable = false;

/* ---------- Deteksi puncak untuk BPM ----------
 * Filter DC (highpass ~0.8Hz) saja tidak cukup membuang modulasi napas/gerakan
 * (~0.2-0.4Hz) yang amplitudonya sering JAUH lebih besar daripada detak jantung
 * (~1-2Hz) -- kalau lolos, threshold adaptif "terkunci" ke ayunan napas dan
 * detak asli tidak akan pernah melewatinya. slowAC = EMA dari sinyal AC itu
 * sendiri (cutoff ~0.5Hz); menguranginya membuat band-pass kasar yang fokus ke
 * pita detak jantung. */
static const double SLOW_ALPHA = 0.97;
static double acIRSmoothPrev = 0;
static double slowAC = 0;
static double cardiacPrev = 0;
static double peakEnvelope = 100;
static unsigned long lastBeatMs = 0;
static const unsigned long MIN_IBI_MS = 300;   /* refractory -> maks 200 bpm */
static double ibiHistory[4] = { 600, 600, 600, 600 };
static byte   ibiIndex = 0;
static float  currentBPM = 0;
static bool   bpmValid = false;

/* ---------- Window AC-RMS untuk SpO2 ----------
 * Panjang window ditentukan WAKTU, bukan jumlah sampel, dan itu memperbaiki
 * kekeliruan yang tersembunyi lama: versi sebelumnya memakai 100 sampel dengan
 * keterangan "~1 detik pada 100 Hz", padahal SAMPLE_AVERAGE = 8 membuat FIFO
 * mengeluarkan 100/8 = 12,5 sampel per detik. Seratus sampel karena itu bukan 1
 * detik melainkan DELAPAN. Ditambah settle 8 detik, angka SpO2/glukosa/tensi
 * pertama baru mungkin muncul di detik ke-16 -- dan sepanjang itu layar
 * menampilkan "--" tanpa alasan yang bisa dilihat pengguna.
 *
 * Satu detik per window berarti tiap window cuma memuat ~1 detak, jadi R-nya
 * memang lebih berisik dari sebelumnya. Itu justru yang diminta: angka yang
 * bergerak sambil diukur. Kestabilan tidak diambil dari panjang window lagi,
 * melainkan dari dua lapis di atasnya yang memang sudah ada -- rata-rata
 * SMOOTH_N window (push_smoothed) untuk yang tampil, dan rata-rata seluruh sesi
 * (update_session_stats) untuk yang akhirnya dikirim ke aplikasi.
 *
 * MIN masih dijaga: window sependek 1-2 sampel akan menghasilkan RMS yang tidak
 * berarti apa-apa kalau satu poll kebetulan terlambat. */
static const unsigned long SPO2_WINDOW_MS  = 1000;
static const int           SPO2_WINDOW_MIN = 8;
static double sumSqRed = 0, sumSqIR = 0;
static double sumDcRed = 0, sumDcIR = 0;
static int    windowCount = 0;
static unsigned long windowStartMs = 0;
static float  currentSpO2 = 0;
static float  currentPI = 0;
static float  currentR = 0;
static float  currentGlucose = 0;
static float  currentSBP = 0;
static float  currentDBP = 0;

/* ---------- Statistik sesi ---------- */
static long  statBpmSum = 0, statSpo2Sum = 0;
static long  statSbpSum = 0, statDbpSum = 0;
static int   statN = 0;
static int   statBpmMin = 0, statBpmMax = 0;
static int   statSpo2Min = 0;
static int   statGluMin = 0, statGluMax = 0;
static int   statSbpMin = 0, statSbpMax = 0;
static int   statDbpMin = 0, statDbpMax = 0;

static ppg_state_t s_state = PPG_ABSENT;

/* Diagnostik mentah. Tanpa ini, status "tidak menempel" tidak bisa dibedakan
 * dari tiga sebab yang sangat berbeda penanganannya:
 *   ir = 0 dan sampel tidak bertambah -> FIFO tidak jalan / sensor tak dicatu
 *   ir kecil tapi sampel bertambah    -> sensor jalan, ambang terlalu tinggi
 *   ir >= ambang                      -> memang tidak ada jari saja
 * LED MAX30105 butuh puluhan mA; chip bisa ACK di I2C hanya dari pull-up
 * sementara LED-nya tetap gelap, jadi "0x57 menjawab" bukan bukti sensor hidup. */
static long     s_last_ir = 0, s_last_red = 0;
static uint32_t s_samples = 0;
/* Berapa kali FIFO benar-benar ditanyai. Memisahkan "sensor tidak mengirim data"
 * dari "ppg_update() tidak pernah jalan" -- keduanya membuat sampel=0. */
static uint32_t s_polls = 0;

/* ================= tahan hasil terakhir =================
 * Sensor ini hanya menghasilkan angka selama kulit menempel, dan reset DSP saat
 * kontak lepas memang WAJIB (lihat alasan di process_sample). Jadi nilai
 * terakhir tidak bisa "dibiarkan saja" -- harus disalin ke tempat lain sebelum
 * reset menghapusnya.
 *
 * Salinan ini yang dipakai UI setelah sensor diangkat, supaya angkanya tetap
 * terbaca. Dihapus hanya pada dua kejadian:
 *   - pengukuran baru dimulai (kulit menempel lagi)
 *   - ppg_clear_hold() dipanggil dari luar (mis. layar dimatikan)
 */
static ppg_data_t s_hold;
static bool       s_hold_valid = false;
static uint32_t   s_hold_ms    = 0;   /* millis() saat snapshot diambil */
static bool       s_new_result = false; /* window baru menghasilkan angka sah */

static void reset_hold(void) {
  memset(&s_hold, 0, sizeof(s_hold));
  s_hold_valid = false;
  s_hold_ms    = 0;
  /* Ikut dinolkan: process_sample() keluar lebih awal saat kontak lepas, jadi
   * tanda "ada hasil baru" bisa tertinggal true dan memicu salinan palsu pada
   * sampel pertama sesi berikutnya. */
  s_new_result = false;
}

/* Definisinya di bawah, setelah fill_live(). Dideklarasikan di sini karena
 * pemanggilnya (accumulate_spo2) ada lebih dulu dan ini file .cpp biasa --
 * tidak ada penyisipan prototipe otomatis seperti pada .ino. */
static void capture_hold(void);

/* ================= util ================= */
static void reset_beat_detector(void) {
  acIRSmoothPrev = 0;
  slowAC = 0;
  cardiacPrev = 0;
  peakEnvelope = 100;
  lastBeatMs = 0;
  for (int i = 0; i < 4; i++) ibiHistory[i] = 600;
  ibiIndex = 0;
  currentBPM = 0;
  bpmValid = false;
  stableBeatCount = 0;
  beatCount = 0;
  smoothCount = 0;
  smoothIndex = 0;
  readingsAwal = false;
  readingsStable = false;
}

static void reset_session_stats(void) {
  statBpmSum = statSpo2Sum = 0;
  statSbpSum = statDbpSum = 0;
  statN = 0;
  statBpmMin = statBpmMax = 0;
  statSpo2Min = 0;
  statGluMin = statGluMax = 0;
  statSbpMin = statSbpMax = 0;
  statDbpMin = statDbpMax = 0;
}

static void detect_beat(double acIR) {
  /* Smoothing ringan supaya threshold crossing tidak terpicu noise 1 sampel. */
  double acIRSmooth = acIRSmoothPrev * 0.7 + acIR * 0.3;

  slowAC = slowAC * SLOW_ALPHA + acIRSmooth * (1.0 - SLOW_ALPHA);
  double cardiac = acIRSmooth - slowAC;

  /* Threshold dihitung dari envelope SEBELUM update sampel ini: kalau envelope
   * ikut naik memakai puncak sampel yang sedang dievaluasi, threshold akan
   * "mengejar" persis di puncak yang sama dan menggagalkan crossing. */
  double threshold = peakEnvelope * 0.5;

  unsigned long nowMs = millis();
  bool rising = cardiac > threshold && cardiacPrev <= threshold;

  if (rising && (nowMs - lastBeatMs) > MIN_IBI_MS) {
    unsigned long ibi = nowMs - lastBeatMs;
    if (lastBeatMs != 0 && ibi < 2000) {   /* buang IBI pertama & <30bpm */
      ibiHistory[ibiIndex] = ibi;
      ibiIndex = (ibiIndex + 1) % 4;

      double avgIbi = 0;
      for (int i = 0; i < 4; i++) avgIbi += ibiHistory[i];
      avgIbi /= 4.0;

      currentBPM = 60000.0 / avgIbi;
      bpmValid = true;
      if (stableBeatCount < MIN_STABLE_BEATS) stableBeatCount++;
      if (beatCount < 0xFFFF) beatCount++;
    }
    lastBeatMs = nowMs;
  }

  double absVal = fabs(cardiac);
  peakEnvelope = peakEnvelope * 0.98 + absVal * 0.02;

  acIRSmoothPrev = acIRSmooth;
  cardiacPrev = cardiac;
}

static void estimate_glucose(void) {
  currentGlucose = (float)(G0
                         + G1 * currentPI
                         + G2 * (currentBPM - 70.0)
                         + G3 * currentR);
}

static void estimate_bp(void) {
  currentSBP = (float)(SBP0
                     + SBP1 * (currentBPM - 70.0)
                     + SBP2 * currentPI
                     + SBP3 * currentR);
  currentDBP = (float)(DBP0
                     + DBP1 * (currentBPM - 70.0)
                     + DBP2 * currentPI
                     + DBP3 * currentR);
  /* Diastol tidak boleh melebihi sistol -- bisa terjadi kalau regresi
   * placeholder ini kebetulan diberi input di ujung rentang plausible-nya.
   * Bukan koreksi fisiologis, cuma jaring pengaman supaya angkanya tidak
   * terang-terangan mustahil ("140/150") di layar. */
  if (currentDBP > currentSBP - 10.0f) currentDBP = currentSBP - 10.0f;
}

static void push_smoothed(float spo2Raw, float glucoseRaw,
                          float sbpRaw, float dbpRaw) {
  spo2History[smoothIndex] = spo2Raw;
  glucoseHistory[smoothIndex] = glucoseRaw;
  sbpHistory[smoothIndex] = sbpRaw;
  dbpHistory[smoothIndex] = dbpRaw;
  smoothIndex = (smoothIndex + 1) % SMOOTH_N;
  if (smoothCount < SMOOTH_N) smoothCount++;

  float sumSpo2 = 0, sumGlucose = 0, sumSbp = 0, sumDbp = 0;
  for (byte i = 0; i < smoothCount; i++) {
    sumSpo2 += spo2History[i];
    sumGlucose += glucoseHistory[i];
    sumSbp += sbpHistory[i];
    sumDbp += dbpHistory[i];
  }
  currentSpO2 = sumSpo2 / smoothCount;
  currentGlucose = sumGlucose / smoothCount;
  currentSBP = sumSbp / smoothCount;
  currentDBP = sumDbp / smoothCount;
}

static void update_session_stats(void) {
  int bpm = (int)lroundf(currentBPM);
  int sp  = (int)lroundf(currentSpO2);
  int gl  = (int)lroundf(currentGlucose);
  int sb  = (int)lroundf(currentSBP);
  int db  = (int)lroundf(currentDBP);

  if (statN == 0) {
    statBpmMin = statBpmMax = bpm;
    statSpo2Min = sp;
    statGluMin = statGluMax = gl;
    statSbpMin = statSbpMax = sb;
    statDbpMin = statDbpMax = db;
  } else {
    if (bpm < statBpmMin) statBpmMin = bpm;
    if (bpm > statBpmMax) statBpmMax = bpm;
    if (sp  < statSpo2Min) statSpo2Min = sp;
    if (gl  < statGluMin) statGluMin = gl;
    if (gl  > statGluMax) statGluMax = gl;
    if (sb  < statSbpMin) statSbpMin = sb;
    if (sb  > statSbpMax) statSbpMax = sb;
    if (db  < statDbpMin) statDbpMin = db;
    if (db  > statDbpMax) statDbpMax = db;
  }
  statBpmSum += bpm;
  statSpo2Sum += sp;
  statSbpSum += sb;
  statDbpSum += db;
  statN++;
}

static void accumulate_spo2(double acRed, double acIR, double dRed, double dIR) {
  if (windowCount == 0) windowStartMs = millis();
  sumSqRed += acRed * acRed;
  sumSqIR  += acIR * acIR;
  sumDcRed += dRed;
  sumDcIR  += dIR;
  windowCount++;
  if (windowCount < SPO2_WINDOW_MIN) return;
  if ((millis() - windowStartMs) < SPO2_WINDOW_MS) return;

  double acRmsRed = sqrt(sumSqRed / windowCount);
  double acRmsIR  = sqrt(sumSqIR / windowCount);
  double meanDcRed = sumDcRed / windowCount;
  double meanDcIR  = sumDcIR / windowCount;

  /* Gerbang untuk MENGHITUNG. Sengaja tidak lagi memuat stableBeatCount:
   * syarat itu milik pengiriman, bukan perhitungan, dan dipasang di
   * readingsStable beberapa baris di bawah.
   *
   * bpmValid tetap disyaratkan, dan itu bukan sisa lama: model glukosa dan
   * tekanan darah memakai currentBPM sebagai variabel: dengan BPM masih 0,
   * estimate_glucose() menghitung suku G2*(0-70) dan menghasilkan angka yang
   * bukan "sementara" melainkan salah. SpO2 sendiri memang tidak butuh BPM,
   * tetapi jaraknya cuma satu-dua detak -- tidak sepadan dengan layar yang
   * memunculkan satu kartu lebih dulu lalu tiga lainnya menyusul. */
  bool windowGate = (millis() - contactStartMs) >= DC_SETTLE_MS && bpmValid;

  if (windowGate && meanDcRed > 0 && meanDcIR > 0 && acRmsIR > 0) {
    double R  = (acRmsRed / meanDcRed) / (acRmsIR / meanDcIR);
    double pi = (acRmsIR / meanDcIR) * 100.0;

    /* Buang window dengan R atau PI di luar rentang wajar -- itu artefak
     * gerak/kontak longgar, bukan nadi. Kalau dipaksa lewat formula kalibrasi
     * hasilnya angka mustahil (SpO2 mendekati 0%, glukosa ribuan mg/dL). */
    bool plausible = R >= R_MIN_PLAUSIBLE && R <= R_MAX_PLAUSIBLE
                     && pi >= PI_MIN_PLAUSIBLE && pi <= PI_MAX_PLAUSIBLE;

    if (plausible) {
      /* Formula empiris Maxim AN6409. Akurasi sebenarnya sangat bergantung
       * kalibrasi per-perangkat terhadap pulse oximeter medis. */
      float spo2 = -45.060f * (float)R * (float)R + 30.354f * (float)R + 94.845f;
      spo2 += SPO2_WRIST_OFFSET;
      if (spo2 > 100) spo2 = 100;

      if (spo2 >= SPO2_MIN_PLAUSIBLE) {
        currentPI = (float)pi;
        currentR  = (float)R;
        estimate_glucose();
        estimate_bp();
        push_smoothed(spo2, currentGlucose, currentSBP, currentDBP);
        readingsAwal = true;
        readingsStable = (stableBeatCount >= MIN_STABLE_BEATS);

        /* Statistik sesi hanya menghimpun window yang sudah stabil. Ia adalah
         * sumber angka final yang dikirim ke aplikasi (lihat ukur_putar di
         * aw_jam.cpp), jadi memasukkan window sementara ke sana berarti angka
         * yang terkirim ikut tercemar oleh detik-detik pertama yang justru
         * ditandai "belum bisa dipercaya" di layar.
         *
         * Yang tidak bisa dipisahkan sepenuhnya: push_smoothed() di atas
         * merata-ratakan SMOOTH_N window terakhir, jadi window stabil pertama
         * masih membawa jejak beberapa window sementara sebelumnya. Itu memang
         * dibiarkan -- keduanya bacaan sungguhan dari sensor yang sama, dan
         * membuang jejaknya berarti membuang pula peredam yang membuat angka
         * pertama tidak melonjak. */
        if (readingsStable) {
          update_session_stats();
          /* Jangan panggil capture_hold() di sini: s_state baru ditetapkan
           * setelah accumulate_spo2() selesai, jadi salinan akan lahir dengan
           * state basi (PPG_SETTLING pada window pertama) dan bpm_valid-nya
           * false. Cukup tandai bahwa ada hasil baru; salinannya diambil di
           * process_sample.
           *
           * Salinan juga hanya diambil dari window stabil: yang ditahan setelah
           * jari diangkat adalah HASIL, dan angka sementara yang membeku di
           * layar tanpa penanda apa pun tidak bisa dibedakan darinya. */
          s_new_result = true;
        }

#if NET_DEBUG
        /* PI & R dicetak supaya bisa dicatat berpasangan dengan hasil
         * glukometer sungguhan untuk kalibrasi model glukosa nanti.
         * Dibatasi 1x/2 detik: window selesai tiap ~1 detik dan board ini
         * masih harus melayani LVGL. */
        static uint32_t lastLog = 0;
        if ((uint32_t)(millis() - lastLog) >= 2000) {
          lastLog = millis();
          Serial.printf("[ppg] %sBPM %.1f  SpO2 %.1f%%  Glukosa* %.1f mg/dL  "
                        "TD* %.0f/%.0f mmHg  [PI %.3f  R %.4f]\n",
                        readingsStable ? "" : "(awal) ",
                        currentBPM, currentSpO2, currentGlucose,
                        currentSBP, currentDBP, currentPI, currentR);
        }
#endif
      }
    }
  }

  sumSqRed = 0; sumSqIR = 0;
  sumDcRed = 0; sumDcIR = 0;
  windowCount = 0;
}

/* Proses satu sampel Red/IR mentah. */
static void process_sample(long redVal, long irVal) {
  s_last_ir  = irVal;
  s_last_red = redVal;
  s_samples++;

  bool rawInContact = irVal >= IR_PRESENCE_THRESHOLD;
  if (rawInContact != rawContactCandidate) {
    rawContactCandidate = rawInContact;
    rawContactCandidateSinceMs = millis();
  }
  bool inContact = wasInContact;
  if (rawContactCandidate != inContact
      && (millis() - rawContactCandidateSinceMs) >= CONTACT_DEBOUNCE_MS) {
    inContact = rawContactCandidate;
  }

  if (inContact != wasInContact) {
    if (inContact) {
      contactStartMs = millis();
      /* Pengukuran baru: hasil lama dibuang supaya layar tidak mencampur angka
       * sesi sebelumnya dengan sesi yang sedang berjalan. Selama menstabilkan,
       * layar menampilkan "--". */
      reset_hold();
      Serial.println("[ppg] kontak kulit terdeteksi, menstabilkan sinyal...");
    } else {
      /* Sensor diangkat. Salinan TIDAK dihapus di sini -- justru inilah saat
       * salinan itu mulai dipakai UI. Dihapus hanya oleh pengukuran baru atau
       * ppg_clear_hold(). */
      Serial.println("[ppg] sensor terlepas dari kulit, menahan hasil terakhir");
    }
    /* Reset tiap sesi kontak berganti. Tanpa ini, lonjakan transien saat filter
     * DC "mengejar" baseline baru ikut termakan peakEnvelope dan menaikkan
     * threshold-nya secara permanen (envelope hanya naik, tidak turun sendiri)
     * -- akibatnya detak asli yang jauh lebih kecil tidak akan pernah lolos. */
    reset_beat_detector();
    reset_session_stats();
    wasInContact = inContact;
  }

  if (!inContact) {
    dcInit = false;
    windowCount = 0;
    s_state = PPG_NO_CONTACT;
    return;
  }

  if (!dcInit) {
    dcRed = redVal;
    dcIR  = irVal;
    dcInit = true;
  }

  dcRed = dcRed * DC_ALPHA + redVal * (1 - DC_ALPHA);
  dcIR  = dcIR  * DC_ALPHA + irVal  * (1 - DC_ALPHA);

  double acRed = redVal - dcRed;
  double acIR  = irVal  - dcIR;

  bool dcSettled = (millis() - contactStartMs) >= DC_SETTLE_MS;

  /* Selama settle, acIR masih didominasi transien "DC mengejar baseline",
   * bukan detak asli -- jangan diumpankan ke detektor supaya peakEnvelope
   * tidak terkontaminasi. */
  if (dcSettled) detect_beat(acIR);
  accumulate_spo2(acRed, acIR, dcRed, dcIR);

  if (!dcSettled)                              s_state = PPG_SETTLING;
  else if (!readingsStable)                    s_state = PPG_ACQUIRING;
  else                                         s_state = PPG_STABLE;

  /* Salinan hasil diambil DI SINI, setelah s_state di atas ditetapkan, karena
   * fill_live() menurunkan bpm_valid dari s_state. Hanya saat ada hasil window
   * baru, bukan tiap sampel -- 100 Hz x memcpy struct itu pemborosan. */
  if (s_new_result) {
    s_new_result = false;
    capture_hold();
  }
}

/* ================= API ================= */
bool ppg_begin(void) {
  /* I2C_SPEED_STANDARD (100 kHz), BUKAN I2C_SPEED_FAST: begin() memanggil
   * setClock(), dan 400 kHz membuat touch CST816T tidak stabil di board ini. */
  s_present = sensor.begin(Wire, I2C_SPEED_STANDARD, MAX30105_ADDRESS);
  if (!s_present) {
    Serial.println("[ppg] MAX30105/30102 tidak terdeteksi di 0x57 "
                   "(BPM/SpO2/glukosa dilewati, UI tetap jalan)");
    s_state = PPG_ABSENT;
    return false;
  }

  sensor.setup(LED_BRIGHTNESS, SAMPLE_AVERAGE, LED_MODE,
               SAMPLE_RATE, PULSE_WIDTH, ADC_RANGE);

  /* Dipaksa ulang: kalau versi library apa pun sempat mengubah clock bus,
   * touch akan rusak. Lebih murah menegaskan daripada mendiagnosanya nanti. */
  Wire.setClock(100000);

  /* setup() di atas WAJIB dijalankan -- ia yang menulis seluruh register mode,
   * arus LED, sample rate, dan pulse width. Tapi ia juga langsung menyalakan
   * kedua LED. Jadi begitu konfigurasinya tertanam, chip segera ditidurkan:
   * boot berakhir dalam keadaan hemat daya, dan ppg_set_enabled(true) nanti
   * cukup membalik satu bit tanpa mengonfigurasi ulang apa pun. */
  sensor.shutDown();
  s_enabled = false;
  s_state = PPG_OFF;
  Serial.println("[ppg] MAX30105/30102 OK (Red+IR, 100 Hz) -- mulai dalam keadaan MATI");
  Serial.println("[ppg] Glukosa* = estimasi EKSPERIMENTAL belum terkalibrasi, "
                 "bukan alat medis");
  return true;
}

void ppg_update(void) {
  /* Saat mati: tidak ada satu pun transaksi I2C. Selain hemat daya, ini juga
   * mengembalikan seluruh jatah bus ke touch CST816T yang datanya hilang
   * hampir seketika setelah IRQ (lihat catatan di touch.ino). */
  if (!s_present || !s_enabled) return;

  /* Gate waktu supaya transaksi I2C tidak dilakukan setiap iterasi loop (loop
   * berputar tiap ~2 ms). FIFO 32 sampel pada 100 Hz = 320 ms sebelum
   * overflow, jadi 20 ms sangat aman. */
  static uint32_t lastPoll = 0;
  if ((uint32_t)(millis() - lastPoll) < 20) return;
  lastPoll = millis();

  s_polls++;
  sensor.check();                   /* non-blocking: isi FIFO lokal */

  int guard = 0;
  while (sensor.available() && guard++ < 40) {
    long red = (long)sensor.getFIFORed();
    long ir  = (long)sensor.getFIFOIR();
    sensor.nextSample();
    process_sample(red, ir);
  }
}

/* Isi struct dari keadaan DSP saat ini (bacaan langsung). Dipakai bersama oleh
 * ppg_get() dan capture_hold() supaya salinan tidak pernah beda isi dari
 * bacaan langsungnya. */
static void fill_live(ppg_data_t *out) {
  memset(out, 0, sizeof(*out));

  out->state = s_state;

  out->bpm_valid = bpmValid && s_state >= PPG_ACQUIRING;
  out->bpm       = currentBPM;

  out->spo2_valid = readingsStable;
  out->spo2       = currentSpO2;

  out->glu_valid = readingsStable;
  out->glucose   = currentGlucose;

  out->bp_valid = readingsStable;
  out->sbp      = currentSBP;
  out->dbp      = currentDBP;

  /* Ada angka, tapi belum lolos gerbang kirim. UI menampilkannya (dengan warna
   * berbeda); aw_jam tidak pernah memanennya. */
  out->awal = readingsAwal && !readingsStable;

  out->pi = currentPI;
  out->r  = currentR;

  out->beats = beatCount;

  if (statN > 0) {
    out->stats_valid = true;
    out->bpm_min = statBpmMin;
    out->bpm_max = statBpmMax;
    out->bpm_avg = (int)(statBpmSum / statN);
    out->spo2_min = statSpo2Min;
    out->spo2_avg = (int)(statSpo2Sum / statN);
    out->glu_min = statGluMin;
    out->glu_max = statGluMax;
    out->sbp_min = statSbpMin;
    out->sbp_max = statSbpMax;
    out->sbp_avg = (int)(statSbpSum / statN);
    out->dbp_min = statDbpMin;
    out->dbp_max = statDbpMax;
    out->dbp_avg = (int)(statDbpSum / statN);
  }
}

/* Ambil salinan hasil terakhir yang sah. Dipanggil setiap kali sebuah window
 * menghasilkan angka stabil, jadi salinannya selalu yang terbaru -- tidak
 * bergantung pada berhasilnya menangkap momen sensor diangkat. */
static void capture_hold(void) {
  fill_live(&s_hold);
  s_hold.held  = true;
  s_hold_valid = true;
  s_hold_ms    = millis();
}

void ppg_clear_hold(void) {
  reset_hold();
}

void ppg_get(ppg_data_t *out) {
  if (!out) return;
  fill_live(out);

  /* Selama masih ada bacaan langsung, itu yang dipakai. Begitu sensor diangkat,
   * reset DSP membuat semuanya 0/invalid -- di titik itu salinan terakhir yang
   * ditampilkan, supaya angka di layar tidak hilang.
   *
   * state sengaja TIDAK diambil dari salinan: ia harus tetap menggambarkan
   * kondisi sensor yang sebenarnya, sehingga titik "live" di layar berhenti
   * berkedip walaupun angkanya masih terbaca. */
  /* out->awal ikut menahan salinan: angka sementara ADALAH bacaan langsung, dan
   * menggantinya dengan hasil pengukuran sebelumnya di tengah pengukuran baru
   * berarti layar mundur ke angka lama justru saat pengguna sedang menatapnya. */
  if (!out->bpm_valid && !out->spo2_valid && !out->awal && s_hold_valid) {
    ppg_state_t live_state = out->state;
    *out = s_hold;
    out->state       = live_state;
    out->held        = true;
    out->hold_age_ms = (uint32_t)(millis() - s_hold_ms);
  }
}

bool ppg_present(void) {
  return s_present;
}

bool ppg_enabled(void) {
  return s_enabled;
}

void ppg_set_enabled(bool on) {
  if (on == s_enabled) return;
  s_enabled = on;

  /* Chip tidak terpasang: niat pengguna tetap dicatat supaya tombol di UI ikut
   * berpindah keadaan dan halaman menu tetap bisa dibuka. Tanpa ini, board
   * tanpa MAX30105 akan terkunci di halaman home -- tombolnya satu-satunya
   * jalan ke menu. s_state dibiarkan PPG_ABSENT karena itu memang kondisinya. */
  if (!s_present) {
    Serial.printf("[ppg] tombol daya %s (chip tidak terdeteksi, tanpa efek)\n",
                  on ? "ON" : "OFF");
    return;
  }

  if (on) {
    sensor.wakeUp();

    /* FIFO masih memuat sampel dari sebelum chip ditidurkan. Kalau tidak
     * dibuang, sampel basi itu masuk ke filter DC sebagai lompatan besar dan
     * mencemari peakEnvelope -- envelope hanya naik, tidak pernah turun
     * sendiri, jadi detak asli setelahnya tidak akan pernah melewatinya. */
    sensor.clearFIFO();

    /* Sesi baru: seluruh DSP dimulai dari nol, sama seperti saat kulit baru
     * menempel. Akumulator window ikut dinolkan -- windowCount saja tidak
     * cukup, sisa sumSq* akan ikut terbagi di window pertama berikutnya. */
    dcInit = false;
    windowCount = 0;
    sumSqRed = sumSqIR = 0;
    sumDcRed = sumDcIR = 0;
    wasInContact = false;
    rawContactCandidate = false;
    reset_beat_detector();
    reset_session_stats();
    reset_hold();

    s_state = PPG_NO_CONTACT;
    Serial.println("[ppg] sensor DINYALAKAN, menunggu kulit menempel");
  } else {
    sensor.shutDown();

    /* Salinan hasil terakhir ikut dihapus. Membiarkannya berarti layar
     * kesehatan tetap memajang angka sementara pengukuran sudah dimatikan --
     * pada layar kesehatan, angka basi yang tampak nyata lebih berbahaya
     * daripada tanda hubung. */
    reset_hold();

    /* Statistik sesi ikut dinolkan. reset_hold() saja tidak cukup: fill_live()
     * menurunkan stats_valid dari statN, yang tidak tersentuh salinan hasil --
     * tanpa baris ini chip "Avg/Min/Max" tetap memajang angka sesi lama
     * walaupun angka utama di atasnya sudah "--". */
    reset_session_stats();

    bpmValid = false;
    readingsAwal = false;
    readingsStable = false;
    s_state = PPG_OFF;
    Serial.println("[ppg] sensor DIMATIKAN (LED padam, hemat daya)");
  }
}

void ppg_diag(long *ir, long *red, uint32_t *samples, long *threshold,
              uint32_t *polls) {
  if (ir)        *ir        = s_last_ir;
  if (red)       *red       = s_last_red;
  if (samples)   *samples   = s_samples;
  if (threshold) *threshold = IR_PRESENCE_THRESHOLD;
  if (polls)     *polls     = s_polls;
}

const char *ppg_state_text(void) {
  switch (s_state) {
    case PPG_ABSENT:     return "sensor tidak ada";
    case PPG_OFF:        return "dimatikan";
    case PPG_NO_CONTACT: return "tidak menempel";
    case PPG_SETTLING:   return "menstabilkan";
    case PPG_ACQUIRING:  return "mencari detak";
    case PPG_STABLE:     return "stabil";
  }
  return "?";
}
