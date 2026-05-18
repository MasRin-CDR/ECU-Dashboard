#pragma once
// ╔══════════════════════════════════════════════════════════════╗
// ║          VARIO DASHBOARD — config.h                         ║
// ║  Semua konstanta, pin, dan struktur data global             ║
// ╚══════════════════════════════════════════════════════════════╝

// ─── MODE DEBUG ──────────────────────────────────────────────────
// Set 1 untuk cetak data ke Serial Monitor, 0 untuk produksi
#define DEBUG_MODE        1
#define DEBUG_BAUD        115200

// ─── MODE SIMULASI ───────────────────────────────────────────────
// Set 1 jika belum punya sensor fisik — dashboard tetap berjalan
#define SIMULATION_MODE   0

// ─── PIN ESP32 ───────────────────────────────────────────────────
// K-Line UART
#define PIN_KLINE_TX      17
#define PIN_KLINE_RX      16

// RPM — interrupt dari pickup coil
#define PIN_RPM           34      // input-only GPIO, pasang pull-up 10kΩ ke 3.3V

// Speed — interrupt dari sensor Hall roda
#define PIN_SPEED_HALL    35      // input-only GPIO

// AFR — wideband O2 sensor (0–5V → voltage divider → 0–3.3V)
#define PIN_AFR_ADC       36      // VP

// Suhu mesin — NTC atau output ECU analog
#define PIN_TEMP_ADC      39      // VN

// Voltase baterai — voltage divider R1=100kΩ, R2=22kΩ
#define PIN_BATT_ADC      32

// Sensor pelampung bensin — analog resistif
#define PIN_FUEL_ADC      33

// TFT SPI (sesuaikan di User_Setup.h TFT_eSPI)
#define PIN_TFT_CS        15
#define PIN_TFT_DC         2
#define PIN_TFT_RST        4
#define PIN_TFT_MOSI      23
#define PIN_TFT_SCLK      18
#define PIN_TFT_MISO      19

// ─── SPESIFIKASI RODA / MESIN ────────────────────────────────────
#define WHEEL_CIRCUMFERENCE_M   1.720f   // keliling roda dalam meter (C = π × diameter)
                                         // Vario 125: ban 80/90-14 → diameter ≈ 0.548m
#define SPEED_PULSES_PER_ROT    1        // jumlah magnet/pulsa per putaran roda

#define RPM_PULSES_PER_ROT      1        // pickup coil: 1 pulsa per TDC

// ─── KALIBRASI SENSOR ────────────────────────────────────────────
// AFR — Innovate LC-2 / AEM wideband: 0–5V → AFR 7.35–22.39
// (voltage divider ÷2.2 → ESP32 menerima 0–2.27V)
#define AFR_VOLT_MIN      0.5f    // volt saat AFR = 10.0
#define AFR_VOLT_MAX      2.0f    // volt saat AFR = 20.0
#define AFR_MIN           10.0f
#define AFR_MAX           20.0f

// Suhu mesin — NTC 10kΩ (Beta = 3950)
// atau ganti dengan rumus linear jika pakai sensor ECU
#define TEMP_R_FIXED      10000.0f   // resistor seri
#define TEMP_R25          10000.0f   // NTC resistance pada 25°C
#define TEMP_BETA         3950.0f    // koefisien Beta NTC
#define TEMP_ADC_REF      3.3f

// Voltase baterai — voltage divider R1=100k, R2=22k
// Vbatt = Vadc × (R1+R2)/R2
#define BATT_R1           100000.0f
#define BATT_R2            22000.0f
#define BATT_ADC_REF        3.3f

// Pelampung bensin — resistansi 10Ω (penuh) – 180Ω (kosong)
// Dengan pull-up 220Ω: ADC turun saat penuh, naik saat kosong
#define FUEL_ADC_EMPTY    3800      // ADC saat tangki kosong
#define FUEL_ADC_FULL      400      // ADC saat tangki penuh

// ─── INTERVAL TUGAS (ms) ─────────────────────────────────────────
#define INTERVAL_SENSOR_ANALOG   50    // baca ADC
#define INTERVAL_KLINE          100    // request ECU
#define INTERVAL_CALC           200    // hitung turunan (health, konsumsi)
#define INTERVAL_DISPLAY        150    // refresh TFT

// ─── SMOOTHING ───────────────────────────────────────────────────
#define SMOOTH_SAMPLES     8     // jumlah sampel moving average
#define RPM_TIMEOUT_MS   500     // jika tidak ada pulsa, RPM = 0
#define SPEED_TIMEOUT_MS 800     // jika tidak ada pulsa, speed = 0

// ─── ENGINE HEALTH — bobot tiap indikator (total = 1.0) ──────────
#define HEALTH_W_TEMP     0.30f
#define HEALTH_W_VOLT     0.20f
#define HEALTH_W_AFR      0.25f
#define HEALTH_W_RPM      0.15f
#define HEALTH_W_FUEL_CONS 0.10f

// ─── WARNA TEMA TFT ──────────────────────────────────────────────
#define COL_BG        0x0000
#define COL_WHITE     0xFFFF
#define COL_GREY      0x4208
#define COL_DARKGREY  0x2104
#define COL_GREEN     0x07E0
#define COL_LIME      0x37E0
#define COL_YELLOW    0xFFE0
#define COL_ORANGE    0xFD20
#define COL_RED       0xF800
#define COL_BLUE      0x001F
#define COL_CYAN      0x07FF
#define COL_AMBER     0xFC60
#define COL_PURPLE    0x780F
#define COL_NAVY      0x000F
#define COL_TEAL      0x0410

// ─── STRUCT DATA SENSOR ──────────────────────────────────────────
struct SensorData {
  // Speed
  float     speedKmh        = 0.0f;

  // RPM
  uint16_t  rpm             = 0;

  // AFR
  float     afr             = 14.7f;

  // Suhu mesin °C
  float     engineTempC     = 25.0f;

  // Voltase baterai
  float     battVolt        = 12.6f;

  // Persentase bensin 0–100%
  float     fuelPct         = 100.0f;

  // Konsumsi BBM rata-rata (km/L)
  float     fuelConsKmL     = 0.0f;

  // Kesehatan mesin rata-rata 0–100%
  float     engineHealthPct = 100.0f;

  // Data tambahan / internal
  float     tps             = 0.0f;    // throttle %
  float     mapKpa          = 101.3f;  // MAP kPa
  uint32_t  distanceM       = 0;       // jarak tempuh meter
  float     fuelUsedL       = 0.0f;    // estimasi BBM terpakai liter

  // Flag valid
  bool      speedValid      = false;
  bool      rpmValid        = false;
  bool      afrValid        = false;
  bool      tempValid       = false;
  bool      battValid       = false;
  bool      fuelValid       = false;
  bool      ecuOnline       = false;
};

// ─── STRUCT KALIBRASI SENSOR ─────────────────────────────────────
struct CalibData {
  float speedFactor   = 1.0f;   // koreksi speed (misalnya 1.02 jika odometer error 2%)
  float afrOffset     = 0.0f;   // offset kalibrasi AFR
  float tempOffset    = 0.0f;   // offset kalibrasi suhu
  float battFactor    = 1.0f;   // koreksi voltase
  float fuelEmpty     = 0.0f;   // ADC bensin kosong (dikalibrasi manual)
  float fuelFull      = 1.0f;   // ADC bensin penuh
};
