// ╔══════════════════════════════════════════════════════════════╗
// ║  VARIO 125 — Digital Dashboard                              ║
// ║  ESP32 + TFT 5" + K-Line ECU Honda Keihin                  ║
// ║                                                             ║
// ║  Data: Speed, RPM, AFR, Suhu, Voltase, Bensin,             ║
// ║        Engine Health, Avg Konsumsi BBM                      ║
// ║                                                             ║
// ║  Library yang WAJIB diinstall:                              ║
// ║    - TFT_eSPI oleh Bodmer (Library Manager)                 ║
// ║                                                             ║
// ║  Konfigurasi User_Setup.h TFT_eSPI:                         ║
// ║    #define ILI9488_DRIVER  (atau ILI9341_DRIVER)            ║
// ║    #define TFT_CS   15                                      ║
// ║    #define TFT_DC    2                                       ║
// ║    #define TFT_RST   4                                       ║
// ║    #define TFT_MOSI 23                                       ║
// ║    #define TFT_SCLK 18                                       ║
// ║    #define TFT_MISO 19                                       ║
// ║    #define SPI_FREQUENCY 27000000                            ║
// ║                                                             ║
// ║  Untuk test tanpa sensor: set SIMULATION_MODE 1 di config.h ║
// ╚══════════════════════════════════════════════════════════════╝

#include "include/config.h"
#include "include/sensors.h"
#include "include/kline.h"
#include "include/display.h"

// ─── Data sensor global ──────────────────────────────────────────
SensorData vehicle;

// ─── Timer millis() tiap task ────────────────────────────────────
uint32_t tSensor  = 0;
uint32_t tKline   = 0;
uint32_t tCalc    = 0;
uint32_t tDisplay = 0;
uint32_t tDebug   = 0;

// ════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════
void setup() {
  if (DEBUG_MODE) {
    Serial.begin(DEBUG_BAUD);
    delay(200);
    Serial.println("\n╔══════════════════════════════╗");
    Serial.println("║  VARIO 125 Dashboard v1.0   ║");
    Serial.println("╚══════════════════════════════╝");
  }

  // Init TFT dulu — tampilkan splash
  displayInit();
  displaySplash("Inisialisasi...", COL_YELLOW);

  // Init sensor & interrupt
  sensorsInit();
  displaySplash("Sensor OK...", COL_GREEN);
  delay(300);

  // Init K-Line ECU
  displaySplash("Menghubungkan ECU...", COL_YELLOW);

#if !SIMULATION_MODE
  klineBegin();
  if (vehicle.ecuOnline) {
    displaySplash("ECU Terhubung!", COL_GREEN);
  } else {
    displaySplash("ECU Tidak Ditemukan — mode offline", COL_ORANGE);
  }
  delay(600);
#else
  displaySplash("Mode Simulasi Aktif", COL_CYAN);
  delay(800);
#endif

  // Reset semua timer ke sekarang
  uint32_t now = millis();
  tSensor = tKline = tCalc = tDisplay = tDebug = now;

  if (DEBUG_MODE) Serial.println("[MAIN] Setup selesai, masuk loop");
}

// ════════════════════════════════════════════
//  LOOP UTAMA
//  Tidak ada delay() — semua pakai millis()
// ════════════════════════════════════════════
void loop() {
  uint32_t now = millis();

  // ── Task 1: Baca sensor ADC & interrupt (50ms) ──────────────
  if (now - tSensor >= INTERVAL_SENSOR_ANALOG) {
    tSensor = now;
    sensorsUpdate(vehicle);
  }

  // ── Task 2: Update K-Line ECU (100ms) ───────────────────────
#if !SIMULATION_MODE
  if (now - tKline >= INTERVAL_KLINE) {
    tKline = now;
    klineUpdate(vehicle);
  }
#endif

  // ── Task 3: Hitung derivatif (health, konsumsi) (200ms) ─────
  if (now - tCalc >= INTERVAL_CALC) {
    tCalc = now;
    vehicle.engineHealthPct = calcEngineHealth(vehicle);

    // Jika ECU tidak online, estimasi konsumsi dari speed/rpm
    if (!vehicle.ecuOnline) {
      vehicle.fuelConsKmL = estimateFuelByLoad(
        vehicle.speedKmh, vehicle.rpm, vehicle.tps);
    }
  }

  // ── Task 4: Refresh display (150ms) ─────────────────────────
  if (now - tDisplay >= INTERVAL_DISPLAY) {
    tDisplay = now;
    displayUpdate(vehicle);
  }

  // ── Task 5: Debug Serial (1000ms) ───────────────────────────
  if (DEBUG_MODE && now - tDebug >= 1000) {
    tDebug = now;
    sensorsPrint(vehicle);
  }
}
