// ===========================================================================
// CONTROLLER_UDP — panel kendali hardware ATG-6.0
// ---------------------------------------------------------------------------
// Perangkat CADANGAN. Kalau desktop app ATG tidak bisa dipakai (komputer rusak,
// mati di tengah pengujian), pengujian tetap bisa dijalankan lewat alat ini:
// Arduino Nano + W5500 + display 7-segmen TM1637 + 3 tombol + LED status.
//
// Perangkat ini TIDAK menggantikan desktop app dan tidak mengubahnya — ia
// berbicara ke papan motor (Basic_Test_UDP) lewat UDP protokol yang SAMA PERSIS.
// Aplikasi dan alat ini adalah dua klien ke papan yang sama; keduanya boleh
// terpasang di jaringan, tapi JANGAN dikirim perintahnya bersamaan.
//
//   Terminal app / controller
//     192.168.1.104 : 8889   →  192.168.1.103 : 8888   (papan motor)
//
// ALUR — ada DUA KEADAAN (tombol: TAHAN BTN3 ±2 detik untuk berpindah)
//
// SENYAP (default) — panel BISU TOTAL: tidak ada satu paket UDP pun keluar.
//   App desktop leluasa; panel tidak mungkin merebut balasan papan. Layar "OFF",
//   LED merah. Satu-satunya aksi: TAHAN BTN3 ±2 dtk.
//
// AKTIF — panel mulai bicara ke papan (ping "y" lalu poll "i" tiap 1 dtk):
//   1. Baru AKTIF → "----", ping sampai papan menjawab (LED hijau).
//   2. Papan ZERO=0 → tampil "0" (WAJIB ZERO); ENTER mengirim "-".
//   3. Papan ZERO=1 & diam → tampil POSISI motor (cm) + titik penanda.
//      Tekan BTN1/BTN2 → buka DAFTAR; pilih 500…1700 cm; ENTER menjalankan.
//      Selesai gerak → kembali ke tampilan POSISI.
//   4. TAHAN BTN3 ±2 dtk → kembali SENYAP. Panel juga otomatis SENYAP setelah
//      ±2 menit tanpa aktivitas tombol, ATAU begitu melihat papan digerakkan
//      pihak lain (app desktop) — supaya tidak mencuri balasan app.
//
// Tidak ada HOME. Kenapa AKTIF memakai poll "i": app desktop & panel adalah dua
// klien, dan panel TIDAK bisa menyadap lalu lintas app (switch tidak mem-
// forward unicast asing; W5500 juga tak bisa). Perintah "i" adalah jalur
// baca-saja yang disediakan firmware (tidak menggerakkan motor) dan tetap
// dijawab saat motor bergerak — panel hanya membaca POS/ZERO/MOVING. Balasan
// dikirim unicast ke panel.
//
// ATURAN AMAN: panel TIDAK mengirim perintah gerak ("@","-","+") selama papan
// melaporkan MOVING=1 atau status STAT-nya basi — handlePositionValue() di
// firmware TIDAK punya guard posisiBergerak, jadi "@" saat motor jalan akan
// merusak hitungan posisi papan. Aturan operasi: aktifkan panel HANYA saat app
// desktop tidak sedang menjalankan gerakan.
//
// ---------------------------------------------------------------------------
// KABEL (Arduino Nano)
//   D10 / D11 / D12 / D13  →  W5500 (CS / MOSI / MISO / SCK)  JANGAN dipakai lain
//   D2  → TM1637 CLK            D3  → TM1637 DIO
//   D4  → LED hijau (resistor 220Ω–1k)   D5  → LED merah (resistor)
//   D6  → BTN1 (kursor naik)   D7  → BTN2 (kursor turun)   D8  → BTN3 (ENTER)
//   D9  → W5500 RST (opsional; ditahan HIGH oleh firmware)
//   Tombol: INPUT_PULLUP, satu kaki ke pin, kaki lain ke GND (tanpa resistor)
//
// Build:  arduino-cli compile --fqbn arduino:avr:nano Controller_UDP/
// Lib:    arduino-cli lib install TM1637      (Ethernet sudah termasuk)
// ===========================================================================
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <TM1637Display.h>
#include <limits.h>   // LONG_MIN — penanda "field tidak ada" saat membaca STAT

// --- Jaringan -------------------------------------------------------------
// MAC HARUS berbeda dari papan motor (DE:AD:BE:EF:EE:D1). Dua perangkat dengan
// MAC sama di satu LAN membuat paket hilang arah dan LED jadi tidak masuk akal.
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xEE, 0xD2 };
IPAddress ipKontroller(192, 168, 1, 104);   // alamat alat ini
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress ipMotor(192, 168, 1, 103);        // HARUS sama dengan Basic_Test_UDP.ino
const unsigned int PORT_MOTOR = 8888;       // HARUS sama dengan Basic_Test_UDP.ino
const unsigned int PORT_LOKAL = 8889;       // port sendiri; balasan board ke sini
EthernetUDP Udp;

// --- Pin ------------------------------------------------------------------
#define PIN_CS     10   // W5500 (default Ethernet.init)
#define PIN_CLK     2   // TM1637
#define PIN_DIO     3   // TM1637
#define PIN_BTN1    8   // kursor naik
#define PIN_BTN2    6   // kursor turun
#define PIN_BTN3    7   // ENTER — jalankan pilihan
#define PIN_LED_HIJAU 4
#define PIN_LED_MERAH 5
#define PIN_RST_W5500 9   // W5500 RST (opsional; aman walau tidak tersambung)

TM1637Display disp(PIN_CLK, PIN_DIO);

// --- Batas titik uji dalam centimeter (cm) ---------------------------------
// Satuan cm mengikuti versi ESP32-S3 (v3.0). Nilai ini SKALA-LABEL rel, sama
// seperti desktop app: 500 = titik 5 m, 1700 = titik 17 m, langkah 1 m.
const int TITIK_MIN_CM = 500;       // 500 cm (5 meter)
const int TITIK_MAX_CM = 1700;      // 1700 cm (17 meter)
const int LANGKAH_TITIK_CM = 100;   // 100 cm (1 meter per klik)

// --- Waktu ----------------------------------------------------------------
const unsigned long JEDA_PING_MS = 2000;         // interval ping saat idle
const unsigned long JEDA_POLL_MS = 1000;         // interval baca status "i" saat idle
const unsigned long HIDUP_LINK_MS = 5000;        // balasan terakhir masih dianggap sehat
const unsigned long TIMEOUT_KECIL_MS = 3000;     // timeout ping / "i"
const unsigned long STAT_BASI_MS = 1500;         // STAT lebih lama dari ini = basi
const unsigned long TAHAN_GANTI_MODE_MS = 2000;  // tahan BTN3 untuk pindah mode
const unsigned long JEDA_SENYAP_MS = 120000;     // AKTIF → SENYAP bila menganggur ±2 menit
const unsigned long JEDA_DAFTAR_MS = 10000;      // tampilan DAFTAR → kembali POSISI bila menganggur
const unsigned long TIMEOUT_ZERO_MS = 180000;    // timeout ZERO: motor bisa jalan ~12 m (±1,5 menit)
const unsigned long TIMEOUT_GERAK_MS = 180000;   // timeout gerak titik (12 m bisa lama)
const unsigned long DEBOUNCE_MS = 250;
const unsigned long KEDIP_MS = 400;
const unsigned long TAMPIL_GALAT_MS = 3000;
const unsigned long TAMPIL_PERINGATAN_MS = 3000;

// --- Glyph 7-segmen ------------------------------------------------------
// bit 0 = segmen A … bit 6 = G, bit 7 = titik desimal.
// SEG_A..SEG_G SUDAH didefinisikan TM1637Display.h dengan nilai identik —
// jangan didefinisikan ulang (warning "SEG_x redefined").
// Hanya SEG_DP yang tidak disediakan library, jadi satu-satunya yang kita buat.
#define SEG_DP 0x80

const uint8_t DIGIT[10] = {
  0x3F, 0x06, 0x5B, 0x4F, 0x66,   // 0 1 2 3 4
  0x6D, 0x7D, 0x07, 0x7F, 0x6F    // 5 6 7 8 9
};
const uint8_t GLYP_MINUS  = SEG_G;                                    // "----"
const uint8_t GLYP_E      = SEG_A | SEG_D | SEG_E | SEG_F | SEG_G;   // "Er"
const uint8_t GLYP_r      = SEG_E | SEG_G;
const uint8_t GLYP_o      = SEG_C | SEG_D | SEG_E | SEG_G;            // "oP"
const uint8_t GLYP_P      = SEG_A | SEG_B | SEG_E | SEG_F | SEG_G;
const uint8_t GLYP_O      = SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;  // "OFF"
const uint8_t GLYP_F      = SEG_A | SEG_E | SEG_F | SEG_G;                  // "OFF"

// --- State ----------------------------------------------------------------
// Dua KEADAAN (tombol: tahan BTN3 ±2 dtk):
//   MODE_SENYAP — default. Panel BISU TOTAL (tidak mengirim apa pun).
//   MODE_AKTIF  — panel bicara ke papan. Di dalamnya ada dua fase:
//                 (1) WAJIB ZERO (bila papan belum ZERO) → prompt "0";
//                 (2) sudah ZERO → tampilan POSISI, dan DAFTAR hanya saat
//                     tombol arah ditekan. Tidak ada HOME.
enum Menunggu { TIDAK, M_PING, M_INFO, M_ZERO, M_GERAK };
enum Layar { L_PUTUS, L_POSISI, L_WAJIB_ZERO, L_PILIH, L_GERAK, L_PERINGATAN, L_GALAT };
enum Mode  { MODE_SENYAP, MODE_AKTIF };

Menunggu menunggu = TIDAK;
Layar layar = L_PUTUS;
Mode  mode  = MODE_SENYAP;        // default: BISU, app desktop leluasa
bool  daftarTerbuka = false;      // AKTIF: false = tampil POSISI, true = tampil DAFTAR
unsigned long aktivitasTerakhirMs = 0;  // pijakan auto-SENYAP & auto-tutup daftar

// Daftar: kursor 0 = ZERO ("0"), 1..13 = titik 500..1700 cm.
const int KURSOR_ZERO          = 0;
const int KURSOR_TITIK_PERTAMA = 1;
const int JUMLAH_TITIK = (TITIK_MAX_CM - TITIK_MIN_CM) / LANGKAH_TITIK_CM + 1; // 13
const int KURSOR_MAKS  = KURSOR_TITIK_PERTAMA + JUMLAH_TITIK - 1;              // 13

int kursor = KURSOR_ZERO;            // dipaksa 0 tiap koneksi baru (lihat masukWajibZero)

// Nilai cm untuk kursor >= KURSOR_TITIK_PERTAMA.
int titikCm(int k) {
  return TITIK_MIN_CM + (k - KURSOR_TITIK_PERTAMA) * LANGKAH_TITIK_CM;
}

bool jaringanSiap = true;
bool gagalHardware = false;
bool linkHidup = false;
unsigned long linkTerakhirOk = 0;
unsigned long waktuKirim = 0;
unsigned long timeoutMs = 0;
unsigned long waktuKirimTerakhir = 0;
unsigned long waktuLayar = 0;
unsigned long waktuKedip = 0;
bool kedip = false;

long posisiMm = 0;                  // POS terakhir dari papan (mm)
bool papanMoving = false;           // MOVING= terakhir dari papan
bool papanZero   = false;           // ZERO= terakhir dari papan
unsigned long statTerakhirMs = 0;   // kapan STAT valid terakhir diterima
bool zeroSesi = false;              // (KONTROL) ZERO sudah dikonfirmasi sesi ini
bool peringatanOffPoint = false;
bool tungguMotorDiam = false;       // gerak timeout → wajib lihat MOVING=0 dulu

// --- Tombol ---------------------------------------------------------------
struct Tombol {
  uint8_t pin;
  bool mentah;
  bool stabil;
  unsigned long waktuUbah;
};
Tombol tombol[3];
bool tertunda[3] = { false, false, false };   // tekanan yang ditahan sementara

void tombolInit() {
  uint8_t pin[3] = { PIN_BTN1, PIN_BTN2, PIN_BTN3 };
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(pin[i], INPUT_PULLUP);
    tombol[i].pin = pin[i];
    tombol[i].mentah = (digitalRead(pin[i]) == LOW);
    tombol[i].stabil = tombol[i].mentah;
    tombol[i].waktuUbah = millis();
  }
}

// True SEKALI per tekanan (bukan saat tombol ditahan), sudah lewat debounce.
bool tombolTekan(uint8_t i) {
  bool mentah = (digitalRead(tombol[i].pin) == LOW);
  if (mentah != tombol[i].mentah) {
    tombol[i].mentah = mentah;
    tombol[i].waktuUbah = millis();
    return false;
  }
  if (millis() - tombol[i].waktuUbah >= DEBOUNCE_MS && tombol[i].stabil != mentah) {
    tombol[i].stabil = mentah;
    if (tombol[i].stabil) return true;   // baru saja ditekan
  }
  return false;
}

// BTN3 diputuskan saat DILEPAS (bukan saat ditekan): supaya "tahan 2 detik"
// untuk ganti mode tidak keburu mengirim ENTER lebih dulu. Pakai struct tombol
// yang sama (indeks 2) agar debounce-nya konsisten.
enum Aksi3 { A3_TIDAK, A3_PENDEK, A3_PANJANG };
Aksi3 aksi3 = A3_TIDAK;
bool  btn3Ditekan = false;
unsigned long btn3Mulai = 0;

void bacaTombol3() {
  bool mentah = (digitalRead(tombol[2].pin) == LOW);
  if (mentah != tombol[2].mentah) {
    tombol[2].mentah = mentah;
    tombol[2].waktuUbah = millis();
    return;
  }
  if (millis() - tombol[2].waktuUbah < DEBOUNCE_MS) return;
  if (tombol[2].stabil == mentah) return;      // tidak ada perubahan stabil
  tombol[2].stabil = mentah;
  if (mentah) {
    btn3Mulai = millis();
    btn3Ditekan = true;
  } else if (btn3Ditekan) {
    btn3Ditekan = false;
    aksi3 = (millis() - btn3Mulai >= TAHAN_GANTI_MODE_MS) ? A3_PANJANG : A3_PENDEK;
  }
}

// --- Display --------------------------------------------------------------
uint8_t layarTerakhir[4];
bool layarBelumDigambar = true;

// Tulis bilangan bulat (0..9999) ke 4 sel; digit depan kosong. `padam` true →
// digit terakhir tidak ditulis (untuk berkedip "sedang bergerak").
void gambarAngka(uint8_t s[4], int nilai, bool padam) {
  if (nilai < 0) nilai = 0;
  int ribuan  = nilai / 1000;
  int ratusan = (nilai / 100) % 10;
  int puluhan = (nilai / 10) % 10;
  int satuan  = nilai % 10;

  if (nilai >= 1000) {                                // "1700"
    s[0] = DIGIT[ribuan];
    s[1] = DIGIT[ratusan];
    s[2] = DIGIT[puluhan];
    s[3] = padam ? 0 : DIGIT[satuan];
  } else if (nilai >= 100) {                          // " 500"
    s[1] = DIGIT[ratusan];
    s[2] = DIGIT[puluhan];
    s[3] = padam ? 0 : DIGIT[satuan];
  } else if (nilai >= 10) {                           // "  50"
    s[2] = DIGIT[puluhan];
    s[3] = padam ? 0 : DIGIT[satuan];
  } else {                                            // "   5"
    s[3] = padam ? 0 : DIGIT[satuan];
  }
}

// Tulis item kursor ke 4 sel. `pakaiKedip` true → sel terakhir berkedip, dipakai
// saat motor bergerak. Layar siap (L_WAJIB_ZERO / L_PILIH) memakai false:
// "menyala penuh = siap ditekan ENTER", "berkedip = sedang berjalan".
void gambarKursor(uint8_t s[4], bool pakaiKedip) {
  bool padam = pakaiKedip && kedip;

  if (kursor == KURSOR_ZERO) {                          // "0"
    if (!padam) s[0] = DIGIT[0];
    return;
  }
  gambarAngka(s, titikCm(kursor), padam);
}

// Isi 4 sel sesuai keadaan. Tidak menyentuh hardware — hanya menghitung.
void susunLayar(uint8_t s[4]) {
  s[0] = 0; s[1] = 0; s[2] = 0; s[3] = 0;

  if (gagalHardware) {
    s[0] = GLYP_E; s[1] = GLYP_r;                 // gagal: hardware tidak ada
    return;
  }

  // --- MODE SENYAP: panel mati → "OFF" apa pun keadaan link-nya ---
  if (mode == MODE_SENYAP) {
    s[0] = GLYP_O; s[1] = GLYP_F; s[2] = GLYP_F;  // "OFF"
    return;
  }

  switch (layar) {
    case L_PUTUS:
      s[0] = GLYP_MINUS; s[1] = GLYP_MINUS;
      s[2] = GLYP_MINUS; s[3] = GLYP_MINUS;       // "----"
      return;

    // --- tampilan default AKTIF: POSISI motor (cm) apa adanya dari "i" ---
    case L_POSISI:
      if (!linkHidup || !papanZero) {             // link putus / belum terkalibrasi
        s[0] = GLYP_MINUS; s[1] = GLYP_MINUS;
        s[2] = GLYP_MINUS; s[3] = GLYP_MINUS;     // "----"
        return;
      }
      gambarAngka(s, (int)(posisiMm / 10), papanMoving && kedip);
      s[1] |= SEG_DP;                             // titik = penanda tampilan POSISI
      return;

    case L_WAJIB_ZERO:
      gambarKursor(s, false);                     // "0" menyala penuh (prompt)
      return;

    case L_PILIH:
      gambarKursor(s, false);                     // selalu menyala penuh
      return;

    case L_GERAK:
      gambarKursor(s, true);                      // sel terakhir berkedip
      return;

    case L_PERINGATAN:
      if (kedip) return;                            // "oP" berkedip
      s[0] = GLYP_o; s[1] = GLYP_P;
      return;

    case L_GALAT:
      s[0] = GLYP_E; s[1] = GLYP_r;                 // "Er"
      return;
  }
}

void gambarLayar() {
  if (layar == L_GALAT && millis() - waktuLayar >= TAMPIL_GALAT_MS) {
    // Setelah "Er": kembali ke fase yang benar. Kalau ZERO belum dikonfirmasi
    // pada sesi ini (mis. baru kena NOT_ZEROED), kembali ke "0" — bukan daftar.
    if (!linkHidup) layar = L_PUTUS;
    else if (!zeroSesi) { kursor = KURSOR_ZERO; layar = L_WAJIB_ZERO; }
    else layar = daftarTerbuka ? L_PILIH : L_POSISI;
    layarBelumDigambar = true;
  }
  if (layar == L_PERINGATAN && millis() - waktuLayar >= TAMPIL_PERINGATAN_MS) {
    peringatanOffPoint = false;
    if (!linkHidup) layar = L_PUTUS;
    else if (!zeroSesi) layar = L_WAJIB_ZERO;
    else layar = daftarTerbuka ? L_PILIH : L_POSISI;
    layarBelumDigambar = true;
  }

  uint8_t s[4];
  susunLayar(s);
  if (!layarBelumDigambar &&
      s[0] == layarTerakhir[0] && s[1] == layarTerakhir[1] &&
      s[2] == layarTerakhir[2] && s[3] == layarTerakhir[3]) {
    return;   // tidak berubah → jangan buang 4 ms bit-banging
  }
  for (uint8_t i = 0; i < 4; i++) layarTerakhir[i] = s[i];
  layarBelumDigambar = false;
  disp.setSegments(layarTerakhir, 4, 0);
}

void setLayar(Layar l) {
  if (layar != l) { layar = l; layarBelumDigambar = true; }
  waktuLayar = millis();
}

void tampilkanGalat() { setLayar(L_GALAT); }

// Kembalikan panel ke fase WAJIB ZERO. Dipanggil saat koneksi baru, saat papan
// ke-reset (STAT ZERO=0), dan saat papan menolak NOT_ZEROED. Kursor dikunci di
// "0" supaya tombol arah tidak bisa lompat ke titik sebelum ZERO.
void masukWajibZero() {
  zeroSesi = false;
  daftarTerbuka = false;
  kursor = KURSOR_ZERO;
  setLayar(L_WAJIB_ZERO);
}

// STAT masih mewakili keadaan papan sekarang? Kalau link hidup tapi sudah lama
// tak ada STAT, kita TIDAK tahu motor sedang apa — anggap belum aman.
bool statSegar() {
  return linkHidup && statTerakhirMs != 0 && (millis() - statTerakhirMs <= STAT_BASI_MS);
}

// Boleh kirim perintah gerak HANYA kalau papan jelas sedang diam dan statusnya
// segar. Ini mencegah "@"/"-" menimpa gerakan yang sedang berjalan (mis. dari
// app desktop): handlePositionValue() di firmware tidak punya guard posisiBergerak.
bool bolehGerak() {
  return statSegar() && !papanMoving;
}

// --- Perpindahan SENYAP <-> AKTIF (dipicu tahan BTN3) ----------------------
// AKTIF: panel mulai bicara. linkHidup dipaksa false supaya jalankanLaju()
// langsung mengirim ping jabat tangan "y"; tampilan POSISI baru dipakai setelah
// STAT pertama tiba (lihat koneksiBaru di prosesBalasan).
void masukAktif() {
  mode = MODE_AKTIF;
  daftarTerbuka = false;
  zeroSesi = false;
  tungguMotorDiam = false;
  menunggu = TIDAK;
  linkHidup = false;              // paksa jalankanLaju() mengirim ping
  statTerakhirMs = 0;             // STAT lama jangan dianggap masih segar
  aktivitasTerakhirMs = millis();
  setLayar(L_PUTUS);
  Serial.println(F("AKTIF — panel mulai bicara ke papan"));
}

// SENYAP: panel berhenti mengirim apa pun. Balasan yang sedang ditunggu
// dibatalkan supaya tidak sisa-sisa ketika panel diaktifkan lagi nanti.
void masukSenyap() {
  mode = MODE_SENYAP;
  daftarTerbuka = false;
  menunggu = TIDAK;
  tungguMotorDiam = false;
  layarBelumDigambar = true;
  Serial.println(F("SENYAP — panel tidak mengirim paket"));
}

// Tampilan siap di AKTIF: POSISI, atau DAFTAR kalau sedang dibuka tombol arah.
void tampilSiap() {
  setLayar(daftarTerbuka ? L_PILIH : L_POSISI);
}

// Titik terdekat dengan posisi sekarang — supaya membuka daftar tidak membuat
// kursor "melompat" dari posisi yang jauh.
int titikTerdekat() {
  int cm = (int)(posisiMm / 10);
  int k = (cm - TITIK_MIN_CM + LANGKAH_TITIK_CM / 2) / LANGKAH_TITIK_CM
          + KURSOR_TITIK_PERTAMA;
  if (k < KURSOR_TITIK_PERTAMA) k = KURSOR_TITIK_PERTAMA;
  if (k > KURSOR_MAKS) k = KURSOR_MAKS;
  return k;
}

// --- LED ------------------------------------------------------------------
void ledUpdate() {
  if (mode == MODE_SENYAP) {                  // panel bisu → merah
    digitalWrite(PIN_LED_HIJAU, LOW);
    digitalWrite(PIN_LED_MERAH, HIGH);
    return;
  }
  bool sehat = linkHidup && (millis() - linkTerakhirOk < HIDUP_LINK_MS);
  digitalWrite(PIN_LED_HIJAU, sehat ? HIGH : LOW);
  digitalWrite(PIN_LED_MERAH, sehat ? LOW : HIGH);
}

// --- Kirim / terima -------------------------------------------------------
void kirim(const String &isi, Menunggu m, unsigned long tmo) {
  Udp.beginPacket(ipMotor, PORT_MOTOR);
  Udp.print(isi);
  Udp.endPacket();
  menunggu = m;
  waktuKirim = millis();
  waktuKirimTerakhir = millis();
  timeoutMs = tmo;
}

// Ambil nilai setelah "KEY=". Mengembalikan LONG_MIN kalau tidak ada.
long bacaField(const String &p, const char *key) {
  String k = String(";") + key + String("=");
  int at = p.indexOf(k);
  if (at < 0) return LONG_MIN;
  at += k.length();
  int end = p.indexOf(';', at);
  String v = (end < 0) ? p.substring(at) : p.substring(at, end);
  if (v.length() == 0) return LONG_MIN;
  return v.toInt();
}

void prosesBalasan(String p) {
  // Koneksi BARU = sebelumnya tidak sehat (boot, kabel lepas-nyambung, papan
  // baru jawab lagi). Saat itu wajib ZERO dulu. linkHidup hanya dibuat turun di
  // jalankanLaju() waktu idle, jadi balasan di tengah gerakan tidak salah
  // dianggap koneksi baru.
  bool koneksiBaru = !linkHidup;
  linkHidup = true;
  linkTerakhirOk = millis();
  if (koneksiBaru && !gagalHardware) {
    if (mode == MODE_AKTIF) {
      if (papanZero) {                 // papan sudah terkalibrasi → langsung POSISI
        zeroSesi = true;
        tampilSiap();
      } else {
        masukWajibZero();
      }
      Serial.println(F("KONEK (AKTIF)"));
    }
  }

  // --- jabat tangan kapabilitas: "OK;FW=6.1;KOREKSI=1;STEP=1;MAX=100"
  if (p.startsWith("OK;")) {
    if (menunggu == M_PING) menunggu = TIDAK;
    return;
  }

  // --- status: "STAT;HOME=..;ZERO=..;POS=..;MOVING=..;ESTOP=.."
  // MOVING dipakai tahu motor benar-benar berhenti setelah timeout. ZERO dipakai
  // mendeteksi papan yang ke-reset diam-diam (heartbeat tetap jalan, tapi
  // zeroDone hilang) → panel balik wajib ZERO.
  if (p.startsWith("STAT;")) {
    long o = bacaField(p, "POS");
    long mv = bacaField(p, "MOVING");
    long z = bacaField(p, "ZERO");
    if (o != LONG_MIN) posisiMm = o;
    if (mv != LONG_MIN) papanMoving = (mv == 1);
    if (z != LONG_MIN) papanZero = (z == 1);
    statTerakhirMs = millis();
    if (mv == 0) tungguMotorDiam = false;   // gerak timeout ternyata sudah selesai
    if (menunggu == M_INFO) menunggu = TIDAK;
    Serial.print(F("STAT pos="));
    Serial.print(posisiMm);
    Serial.print(F(" moving="));
    Serial.print(papanMoving ? 1 : 0);
    Serial.print(F(" zero="));
    Serial.println(papanZero ? 1 : 0);
    if (mode == MODE_AKTIF) {
      // Papan digerakkan PIHAK LAIN (app desktop) sementara panel tidak sedang
      // menunggu gerakan sendiri → panel menyingkir. Ini jaring pengaman supaya
      // panel berhenti mempoll "i" di tengah gerakan app, yang bisa merampas
      // balasan akhir gerakan (POS_DONE/ZERO_DONE) milik app.
      if (mv == 1 && !tungguMotorDiam && menunggu != M_GERAK && menunggu != M_ZERO) {
        Serial.println(F("Papan digerakkan pihak lain — panel kembali SENYAP"));
        masukSenyap();
        return;
      }
      // Papan sudah terkalibrasi (mis. baru dikalibrasi app) → tampilkan siap.
      if (!zeroSesi && papanZero) {
        zeroSesi = true;
        tampilSiap();
      // Papan ke-reset diam-diam (ZERO hilang) → wajib ZERO ulang.
      } else if (zeroSesi && z == 0) {
        Serial.println(F("Papan kehilangan ZERO (di-reset) — wajib ZERO ulang"));
        masukWajibZero();
      }
    }
    return;
  }

  // --- ZERO selesai → buka daftar titik, kursor langsung di 500 cm
  if (p.startsWith("ZERO_DONE")) {
    papanZero = true;
    papanMoving = false;
    statTerakhirMs = millis();
    if (menunggu == M_ZERO) {
      menunggu = TIDAK;
      zeroSesi = true;
      daftarTerbuka = false;          // tampil POSISI dulu (angka "470")
      setLayar(L_POSISI);
      Serial.println(F("ZERO selesai — tampil posisi"));
    }
    return;
  }

  // --- gerakan titik uji: "ARRIVE:…" lebih dulu, lalu "POS_DONE:<mm>"
  if (p.startsWith("ARRIVE:")) {
    if (p.startsWith("ARRIVE:OFF_POINT")) peringatanOffPoint = true;
    return;
  }
  if (p.startsWith("POS_DONE:")) {
    posisiMm = p.substring(9).toInt();
    papanMoving = false;
    statTerakhirMs = millis();
    if (menunggu == M_GERAK) {
      menunggu = TIDAK;
      daftarTerbuka = false;          // gerak selesai → kembali tampilan POSISI
      Serial.print(F("tiba di target, posisi="));
      Serial.println(posisiMm);
      if (peringatanOffPoint) setLayar(L_PERINGATAN);
      else setLayar(L_POSISI);
    }
    return;
  }

  // --- papan menolak karena belum ZERO → balik wajib ZERO + "Er" sebentar.
  if (p.startsWith("NOT_ZEROED") || p.startsWith("NOT_HOMED")) {
    menunggu = TIDAK;
    papanZero = false;
    Serial.print(F("DITOLAK papan: "));
    Serial.println(p);
    masukWajibZero();
    tampilkanGalat();
    return;
  }
  if (p.startsWith("ESTOP")) {
    menunggu = TIDAK;
    Serial.println(F("E-STOP aktif di papan"));
    tampilkanGalat();
    return;
  }

  // Balasan lain (MAJU / MUNDUR / SENSOR:… / ACK) tidak mengubah apa pun —
  // ia hanya tanda bahwa papan hidup, yang sudah dicatat di baris pertama.
  if (menunggu == M_PING) menunggu = TIDAK;
}

void bacaPaket() {
  while (true) {
    int n = Udp.parsePacket();
    if (n <= 0) return;
    char buf[96];
    int len = Udp.read(buf, sizeof(buf) - 1);
    if (len <= 0) return;
    buf[len] = '\0';
    String p = String(buf);
    p.trim();
    if (p.length() > 0) prosesBalasan(p);
  }
}

// --- Laju pengiriman ------------------------------------------------------
// ATURAN PALING PENTING DI FILE INI.
//
// serviceEvents() -> udpEvent() di papan dipanggil DI DALAM loop gerak (tiap
// 64 step), sedangkan handlePositionValue() tidak punya guard "posisiBergerak"
// (guard itu hanya ada di handleKoreksi). Kirim "@" di tengah gerakan akan
// menimpa moveTargetMm/moveSteps dan merusak hitungan posisi selamanya.
// Karena itu: selama menunggu balasan, TIDAK BOLEH ada paket keluar — termasuk
// ping, termasuk "i". Tombol juga diabaikan supaya tidak ada perintah beruntun.
void jalankanLaju() {
  if (!jaringanSiap) return;      // hardware Ethernet tidak ada — sudah "Er" permanen

  // --- SENYAP: panel tidak mengirim APA PUN, termasuk ping "y" dan poll "i". ---
  // Inilah inti perlindungan terhadap app desktop: kalau tidak ada paket keluar,
  // target balasan papan tidak pernah berpindah ke panel. Juga berarti link
  // tidak pernah diperbarui — makanya saat AKTIF linkHidup dipaksa false agar
  // jalankanLaju() mengirim ping jabat tangan lebih dulu.
  if (mode == MODE_SENYAP) return;

  // Link dianggap putus HANYA saat idle. Saat menunggu balasan, ZERO_DONE bisa
  // bermenit-menit sehingga linkTerakhirOk memang lama — itu bukan putus, dan
  // kalau salah ditandai putus, balasan ZERO_DONE-nya justru memicu wajib-ZERO
  // ulang. Saat idle, tak ada balasan dalam >5 dtk memang benar-benar putus.
  if (menunggu == TIDAK && linkHidup && millis() - linkTerakhirOk >= HIDUP_LINK_MS) {
    linkHidup = false;
    if (layar == L_POSISI || layar == L_WAJIB_ZERO ||
        layar == L_PILIH || layar == L_GERAK) setLayar(L_PUTUS);
  }

  if (menunggu != TIDAK) {
    if (millis() - waktuKirim > timeoutMs) {
      Menunggu yangKirim = menunggu;
      menunggu = TIDAK;
      tampilkanGalat();
      Serial.print(F("TIMEOUT menunggu balasan #"));
      Serial.println((int)yangKirim);
      // Gerak/ZERO tidak dapat balasan → posisi papan TIDAK diketahui. Minta STAT
      // (baca-saja, aman) sampai papan bilang MOVING=0, baru tombol hidup lagi.
      if (yangKirim == M_GERAK || yangKirim == M_ZERO) tungguMotorDiam = true;
      // Sengaja TIDAK kirim ulang gerakan otomatis. Operator yang memutuskan.
    }
    return;
  }

  if (!jaringanSiap || !linkHidup || millis() - linkTerakhirOk >= HIDUP_LINK_MS) {
    // Link putus → hanya ping. Jangan sentuh apa pun selain itu.
    if (millis() - waktuKirimTerakhir >= JEDA_PING_MS) {
      kirim(String("y"), M_PING, TIMEOUT_KECIL_MS);
    }
    return;
  }

  // Heartbeat saat idle: poll "i" tiap 1 dtk (JEDA_POLL_MS). Ini baca-saja dan
  // aman walau motor masih jalan — satu-satunya cara mengetahui POSISI motor
  // (mode PANTAU) dan MOVING sudah 0 setelah gerak timeout (tungguMotorDiam).
  if (millis() - waktuKirimTerakhir >= JEDA_POLL_MS) {
    kirim(String("i"), M_INFO, TIMEOUT_KECIL_MS);
  }
}

// --- Tombol ---------------------------------------------------------------
// Hanya keadaan AKTIF yang bisa mengirim perintah gerak. Di SENYAP semua tombol
// diabaikan selain tahan BTN3 untuk kembali AKTIF (ditangani di loop()).
//
// Fase WAJIB ZERO: hanya BTN3 (ENTER) yang aktif, kirim "-". BTN1/BTN2 sengaja
// diabaikan supaya tidak bisa lompat ke titik sebelum papan menerima ZERO.
//
// Setelah papan ZERO=1, tampilan default adalah POSISI. BTN1/BTN2 berfungsi
// ganda: tekan PERTAMA kali → buka DAFTAR (kursor langsung di titik terdekat
// posisi sekarang), tekan berikutnya → pindah kursor. BTN3 (ENTER) baru berlaku
// setelah daftar terbuka — supaya tekanan ragu-ragu di tampilan POSISI tidak
// ikut menggerakkan motor.
//
// SATU ATURAN AMAN: perintah gerak hanya keluar kalau bolehGerak() — papan
// jelas diam & STAT segar. Tanpa ini, "@"/"-" saat papan sedang digerakkan app
// desktop akan menimpa gerakan yang berjalan.
void tanganiTombol(bool t1, bool t2, bool t3) {
  if (mode != MODE_AKTIF) return;           // SENYAP: tidak ada perintah gerak
  if (menunggu != TIDAK) return;            // masih menunggu balasan → abaikan
  if (!linkHidup || millis() - linkTerakhirOk >= HIDUP_LINK_MS) return;
  if (tungguMotorDiam) return;              // belum pasti motor berhenti

  if (!zeroSesi) {                          // --- fase WAJIB ZERO
    if (t3) {
      if (!bolehGerak()) {
        Serial.println(F("ENTER diabaikan: motor belum jelas diam"));
        return;
      }
      peringatanOffPoint = false;
      Serial.println(F("ZERO diminta (wajib)"));
      kirim(String("-"), M_ZERO, TIMEOUT_ZERO_MS);
      setLayar(L_GERAK);
    }
    return;                                 // BTN1/BTN2 diabaikan di fase ini
  }

  // --- fase POSISI: belum memilih apa-apa ---------------------------------
  if (!daftarTerbuka) {
    if (t1 || t2) {                         // tekan arah → buka daftar
      daftarTerbuka = true;
      kursor = titikTerdekat();
      setLayar(L_PILIH);
      Serial.print(F("daftar dibuka di titik "));
      Serial.print(titikCm(kursor));
      Serial.println(F(" cm"));
    }
    return;                                 // ENTER sengaja TIDAK berlaku di sini
  }

  // --- fase DAFTAR TITIK --------------------------------------------------
  if (t1 && kursor > KURSOR_ZERO) { kursor--; setLayar(L_PILIH); }
  if (t2 && kursor < KURSOR_MAKS)  { kursor++; setLayar(L_PILIH); }

  if (t3) {
    if (!bolehGerak()) {
      Serial.println(F("ENTER diabaikan: motor belum jelas diam"));
      return;
    }
    peringatanOffPoint = false;
    if (kursor == KURSOR_ZERO) {
      // Re-ZERO manual (opsional): item "0" tetap ada di ujung atas daftar.
      Serial.println(F("ZERO diminta (ulang)"));
      kirim(String("-"), M_ZERO, TIMEOUT_ZERO_MS);
    } else {
      // Firmware menerima target dalam milimeter: 500 cm -> "@5000".
      kirim(String("@") + String(titikCm(kursor) * 10), M_GERAK, TIMEOUT_GERAK_MS);
    }
    setLayar(L_GERAK);
  }
}

// --- Program --------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial.println(F("PANEL ATG-6.0 — default SENYAP; tahan BTN3 2 dtk untuk AKTIF"));

  pinMode(PIN_LED_HIJAU, OUTPUT);
  pinMode(PIN_LED_MERAH, OUTPUT);
  digitalWrite(PIN_LED_HIJAU, LOW);
  digitalWrite(PIN_LED_MERAH, HIGH);   // default: MERAH (belum ada koneksi)

  tombolInit();

  disp.setBrightness(2, true);
  layarBelumDigambar = true;
  gambarLayar();

  // W5500 RST (opsional, D9). Kalau tersambung: pulsa LOW lalu HIGH = reset
  // bersih saat boot. Kalau D9 tidak tersambung, tidak ada efek samping.
  pinMode(PIN_RST_W5500, OUTPUT);
  digitalWrite(PIN_RST_W5500, LOW);
  delay(10);
  digitalWrite(PIN_RST_W5500, HIGH);
  delay(50);                          // W5500 butuh waktu keluar dari reset

  Ethernet.init(PIN_CS);
  Ethernet.begin(mac, ipKontroller, gateway, subnet);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    gagalHardware = true;              // "Er" menyala terus + LED merah
    jaringanSiap = false;
    return;
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    linkHidup = false;                 // kabel belum colok → "----" + merah
  }
  Udp.begin(PORT_LOKAL);
}

void loop() {
  bacaPaket();
  jalankanLaju();

  bool t1 = tombolTekan(0);
  bool t2 = tombolTekan(1);
  bacaTombol3();                            // isi aksi3 (diputuskan saat DILEPAS)

  Aksi3 a3 = aksi3;
  aksi3 = A3_TIDAK;

  // Tiap tekanan menaikkan "aktivitas" — pijakan dua jaring pengaman di bawah.
  if (t1 || t2 || a3 != A3_TIDAK) aktivitasTerakhirMs = millis();

  // Ganti SENYAP <-> AKTIF (tahan BTN3). Boleh saat idle ATAU saat menunggu
  // balasan baca-saja (ping "y" / status "i") — keduanya tidak menggerakkan
  // motor. Diblokir hanya saat panel sedang menunggu ZERO/gerak sendiri, supaya
  // tahanan tidak bertumpuk dengan gerakan yang sedang berjalan.
  if (a3 == A3_PANJANG && menunggu != M_ZERO && menunggu != M_GERAK) {
    if (mode == MODE_SENYAP) masukAktif();
    else                    masukSenyap();
  }

  // ATURAN 1 — SENYAP berarti panel benar-benar pasif: jangan ada tekanan yang
  // masuk ke antrean, apalagi sampai ke tanganiTombol(). ENTER kini kejadian
  // "dilepas pendek" (lihat bacaTombol3); jendela M_PING/M_INFO cuma ±1 ms,
  // tapi kalau tombol jatuh tepat di situ tekanannya harus ditahan. Saat koneksi
  // putus / motor jalan, tekanan HARUS dibuang.
  bool bolehAntre = (mode == MODE_AKTIF)
                    && (menunggu == TIDAK || menunggu == M_PING || menunggu == M_INFO)
                    && linkHidup;
  if (bolehAntre) {
    tertunda[0] = tertunda[0] || t1;
    tertunda[1] = tertunda[1] || t2;
    if (a3 == A3_PENDEK) tertunda[2] = true;
  } else {
    tertunda[0] = tertunda[1] = tertunda[2] = false;
  }

  if (menunggu == TIDAK) {
    bool a = tertunda[0], b = tertunda[1], c = tertunda[2];
    tertunda[0] = tertunda[1] = tertunda[2] = false;
    tanganiTombol(a, b, c);
  }

  // ATURAN 2 — daftar terbuka tapi tidak disentuh → kembali ke tampilan POSISI.
  if (mode == MODE_AKTIF && zeroSesi && daftarTerbuka && menunggu == TIDAK
      && millis() - aktivitasTerakhirMs >= JEDA_DAFTAR_MS) {
    daftarTerbuka = false;
    setLayar(L_POSISI);
  }

  // ATURAN 3 — keadaan default panel: menganggur ±2 menit → SENYAP lagi, supaya
  // panel tidak diam-diam mempoll "i" saat app desktop mulai dipakai kembali.
  if (mode == MODE_AKTIF && menunggu == TIDAK && !papanMoving
      && millis() - aktivitasTerakhirMs >= JEDA_SENYAP_MS) {
    Serial.println(F("Tidak ada aktivitas ±2 menit — panel kembali SENYAP"));
    masukSenyap();
  }

  if (millis() - waktuKedip >= KEDIP_MS) {
    waktuKedip = millis();
    kedip = !kedip;
  }

  gambarLayar();
  ledUpdate();
}
