// ============================================================
// ESP32 Motor ECU Dashboard - TFT 800x480
// Platform  : ESP32 / ESP32-S3
// Framework : Arduino
// Display   : TFT 5" 800x480 via TFT_eSPI
// Author    : Generated for Motor ECU Project
// ============================================================
// WIRING RINGKASAN (detail di bawah fungsi setup):
//   RPM       -> GPIO 34 (interrupt, via optocoupler)
//   Speed     -> GPIO 35 (interrupt, via optocoupler)
//   AFR       -> GPIO 36 (ADC1_CH0, wideband O2 controller 0-5V via divider)
//   Eng Temp  -> GPIO 39 (ADC1_CH3, NTC/DS18B20)
//   Battery   -> GPIO 32 (ADC1_CH4, voltage divider)
//   Fuel Lvl  -> GPIO 33 (ADC1_CH5, fuel float sensor)
//   TPS       -> GPIO 25 (ADC2_CH8)
//   MAP       -> GPIO 26 (ADC2_CH9)
//   IAT       -> GPIO 27 (ADC2_CH7)
//   Mode Btn  -> GPIO 0  (BOOT button atau tombol fisik)
//   TFT       -> SPI pins sesuai konfigurasi TFT_eSPI
// ============================================================

#include <TFT_eSPI.h>
#include <SPI.h>

// ============================================================
// KONFIGURASI - ubah sesuai hardware
// ============================================================
#define SIMULATION_MODE  true   // true = gunakan data simulasi, false = sensor asli
#define SCREEN_W         800
#define SCREEN_H         480
#define SERIAL_DEBUG     true

// ============================================================
// PIN DEFINITIONS
// ============================================================
#define PIN_RPM           34    // Interrupt dari pulser/coil (via optocoupler)
#define PIN_SPEED         35    // Interrupt dari sensor speed roda
#define PIN_AFR           36    // ADC - wideband O2 controller output (0-5V -> divider ke 0-3.3V)
#define PIN_ENG_TEMP      39    // ADC - NTC thermistor atau DS18B20
#define PIN_BATTERY       32    // ADC - voltage divider dari aki 12V
#define PIN_FUEL          33    // ADC - sensor pelampung tangki
#define PIN_TPS           25    // ADC - Throttle Position Sensor 0-5V -> divider
#define PIN_MAP_SENSOR    26    // ADC - MAP sensor tekanan intake
#define PIN_IAT           27    // ADC - Intake Air Temperature
#define PIN_MODE_BTN      0     // Tombol ganti mode (pullup)
#define PIN_FUEL_PUMP     18    // Digital input - status fuel pump
#define PIN_FAN           19    // Digital input - status kipas radiator

// ============================================================
// KONSTANTA KALIBRASI
// ============================================================
// RPM: pulses per revolution (pilih sesuai motor: 1 atau 2)
#define RPM_PULSES_PER_REV   1
// Speed: pulses per km (kalibrasi dari speedometer asli)
#define SPEED_PULSES_PER_KM  1400
// Battery voltage divider: R1=10kΩ, R2=3.3kΩ (faktor = (R1+R2)/R2 = 4.03)
#define VBAT_DIVIDER_RATIO   4.03f
// ADC reference voltage ESP32
#define ADC_VREF             3.3f
#define ADC_RESOLUTION       4095.0f
// Fuel sensor: ADC min dan max dari sensor pelampung
#define FUEL_ADC_EMPTY       400
#define FUEL_ADC_FULL        3600
// AFR wideband: typical output 0-5V mapped ke 10.0-20.0 AFR (via divider ke 0-3.3V)
#define AFR_VOLTAGE_MIN      0.5f
#define AFR_VOLTAGE_MAX      4.5f
#define AFR_VALUE_MIN        10.0f
#define AFR_VALUE_MAX        20.0f
// Engine temp NTC: gunakan lookup table atau linearisasi (placeholder linear)
#define TEMP_ADC_MIN         200
#define TEMP_ADC_MAX         3800
#define TEMP_C_MIN           -10.0f
#define TEMP_C_MAX           150.0f

// ============================================================
// INTERVAL TIMING (non-blocking dengan millis)
// ============================================================
#define INTERVAL_SENSOR_FAST   50    // ms - RPM, TPS, AFR
#define INTERVAL_SENSOR_SLOW   500   // ms - suhu, baterai, fuel
#define INTERVAL_DISPLAY       100   // ms - refresh UI
#define INTERVAL_GRAPH         200   // ms - update grafik mini
#define INTERVAL_HEALTH        2000  // ms - hitung engine health
#define INTERVAL_DEBUG         1000  // ms - Serial output
#define RPM_TIMEOUT_US         500000UL // 0.5 detik = RPM = 0 (idle/mati)

// ============================================================
// JUMLAH SAMPLE UNTUK MOVING AVERAGE
// ============================================================
#define MA_SIZE   8

// ============================================================
// MINI GRAPH BUFFER
// ============================================================
#define GRAPH_POINTS  60  // titik data per grafik

// ============================================================
// ENUMS
// ============================================================
enum DisplayMode {
  MODE_DASHBOARD = 0,
  MODE_ECU_MAPPING,
  MODE_DIAGNOSTIC,
  MODE_SENSOR_MONITOR,
  MODE_COUNT
};

enum SensorStatus {
  STATUS_OK = 0,
  STATUS_WARNING,
  STATUS_ERROR,
  STATUS_OFFLINE
};

enum AFRStatus {
  AFR_LEAN = 0,
  AFR_NORMAL,
  AFR_RICH
};

enum TempStatus {
  TEMP_COLD = 0,
  TEMP_NORMAL,
  TEMP_HOT,
  TEMP_OVERHEAT
};

enum BattStatus {
  BATT_LOW = 0,
  BATT_NORMAL,
  BATT_HIGH
};

enum RPMStatus {
  RPM_IDLE = 0,
  RPM_NORMAL,
  RPM_HIGH,
  RPM_OVERREV
};

enum HealthStatus {
  HEALTH_EXCELLENT = 0,
  HEALTH_NORMAL,
  HEALTH_WARNING,
  HEALTH_CRITICAL
};

// ============================================================
// STRUKTUR DATA MOVING AVERAGE
// ============================================================
struct MovingAvg {
  float   buffer[MA_SIZE];
  int     index;
  float   sum;
  bool    filled;

  MovingAvg() : index(0), sum(0), filled(false) {
    memset(buffer, 0, sizeof(buffer));
  }

  float update(float val) {
    sum -= buffer[index];
    buffer[index] = val;
    sum += val;
    index = (index + 1) % MA_SIZE;
    if (index == 0) filled = true;
    return sum / (filled ? MA_SIZE : (index == 0 ? MA_SIZE : index));
  }
};

// ============================================================
// STRUKTUR DATA SENSOR TUNGGAL
// ============================================================
struct SensorData {
  float         value;          // Nilai setelah konversi
  float         rawADC;         // Raw ADC value
  float         minVal;         // Min sejak start
  float         maxVal;         // Max sejak start
  float         avgVal;         // Running average
  uint32_t      lastUpdate;     // millis() terakhir update
  SensorStatus  status;         // OK / WARNING / ERROR / OFFLINE
  MovingAvg     ma;             // Moving average filter
  long          avgCount;
  float         avgAccum;

  SensorData() : value(0), rawADC(0), minVal(9999), maxVal(-9999),
                 avgVal(0), lastUpdate(0), status(STATUS_OFFLINE),
                 avgCount(0), avgAccum(0) {}

  void updateStats(float v) {
    value = v;
    lastUpdate = millis();
    if (v < minVal) minVal = v;
    if (v > maxVal) maxVal = v;
    avgCount++;
    avgAccum += v;
    avgVal = avgAccum / avgCount;
  }
};

// ============================================================
// STRUKTUR DATA SEMUA SENSOR
// ============================================================
struct AllSensors {
  SensorData speed;         // km/h
  SensorData rpm;           // RPM
  SensorData afr;           // Air Fuel Ratio
  SensorData engTemp;       // °C
  SensorData battery;       // Volt
  SensorData fuelLevel;     // %
  SensorData tps;           // % throttle
  SensorData mapSensor;     // kPa
  SensorData iat;           // °C Intake Air Temp
  SensorData oilTemp;       // °C (placeholder)
  SensorData oilPressure;   // kPa (placeholder)
  SensorData injPulse;      // ms injector pulse width
  SensorData ignTiming;     // derajat advance
  SensorData knockLevel;    // level knock sensor
  SensorData ambientTemp;   // °C suhu udara luar
  SensorData humidity;      // % RH
  SensorData barometric;    // hPa tekanan udara
} sensors;

// ============================================================
// VARIABEL STATUS ANALISIS
// ============================================================
AFRStatus    afrStatus    = AFR_NORMAL;
TempStatus   tempStatus   = TEMP_COLD;
BattStatus   battStatus   = BATT_NORMAL;
RPMStatus    rpmStatus    = RPM_IDLE;
HealthStatus healthStatus = HEALTH_NORMAL;
float        engineHealth = 100.0f;
float        fuelConsumptionL100 = 0.0f;

// ============================================================
// INTERRUPT VARIABLES (volatile)
// ============================================================
volatile uint32_t rpmPulseCount    = 0;
volatile uint32_t rpmLastTime      = 0;
volatile uint32_t rpmPulseInterval = 0;  // microseconds antar pulse
volatile uint32_t speedPulseCount  = 0;
volatile uint32_t speedLastTime    = 0;

// ============================================================
// TIMING VARIABEL
// ============================================================
uint32_t lastFastSensor   = 0;
uint32_t lastSlowSensor   = 0;
uint32_t lastDisplay      = 0;
uint32_t lastGraph        = 0;
uint32_t lastHealth       = 0;
uint32_t lastDebug        = 0;
uint32_t lastModeBtn      = 0;

// ============================================================
// MODE & UI STATE
// ============================================================
DisplayMode currentMode    = MODE_DASHBOARD;
bool        warningVisible = false;
bool        prevModBtn     = HIGH;
bool        needFullRedraw = true;

// ============================================================
// MINI GRAPH BUFFERS
// ============================================================
float graphRPM[GRAPH_POINTS];
float graphAFR[GRAPH_POINTS];
float graphTemp[GRAPH_POINTS];
float graphBatt[GRAPH_POINTS];
int   graphIdx = 0;

// ============================================================
// SIMULASI STATE
// ============================================================
float simTime = 0;

// ============================================================
// TFT OBJECT
// ============================================================
TFT_eSPI tft = TFT_eSPI(SCREEN_W, SCREEN_H);
TFT_eSprite spr = TFT_eSprite(&tft);  // Sprite untuk double-buffer

// ============================================================
// INTERRUPT SERVICE ROUTINES
// ============================================================

/**
 * ISR_RPM - Dipanggil setiap ada pulse dari sensor RPM (pulser/coil)
 * Hitung interval antar pulse untuk kalkulasi RPM
 * Proteksi: pasang optocoupler di depan input untuk isolasi
 */
void IRAM_ATTR ISR_RPM() {
  uint32_t now = micros();
  rpmPulseInterval = now - rpmLastTime;
  rpmLastTime = now;
  rpmPulseCount++;
}

/**
 * ISR_Speed - Dipanggil setiap ada pulse dari sensor speed roda
 */
void IRAM_ATTR ISR_Speed() {
  uint32_t now = micros();
  speedLastTime = now;
  speedPulseCount++;
}

// ============================================================
// FUNGSI PEMBACAAN SENSOR
// ============================================================

/**
 * readRPM()
 * Rumus: RPM = 60,000,000 / (interval_us * pulses_per_rev)
 * interval_us = waktu antar pulse dalam microseconds
 * Jika timeout (tidak ada pulse > RPM_TIMEOUT_US), RPM = 0
 */
float readRPM() {
  if (SIMULATION_MODE) {
    // Simulasi RPM: idle -> akselerasi -> cruise
    float t = simTime;
    float rpmSim = 1200 + 2500 * sin(t * 0.3) + 800 * sin(t * 0.7);
    if (rpmSim < 800) rpmSim = 800;
    if (rpmSim > 9500) rpmSim = 9500;
    return rpmSim;
  }
  uint32_t interval = rpmPulseInterval;
  uint32_t elapsed  = micros() - rpmLastTime;
  if (elapsed > RPM_TIMEOUT_US || interval == 0) return 0.0f;
  float rpm = 60000000.0f / ((float)interval * RPM_PULSES_PER_REV);
  return rpm;
}

/**
 * readSpeed()
 * Rumus: Speed (km/h) = (pulseCount / PULSES_PER_KM) * (3600 / elapsed_seconds)
 * Atau dari GPS NMEA $GPVTG sentence
 */
float readSpeed() {
  if (SIMULATION_MODE) {
    float spd = 60 + 30 * sin(simTime * 0.25);
    if (spd < 0) spd = 0;
    return spd;
  }
  static uint32_t lastSpeedCalc = 0;
  static uint32_t lastPulseSnap = 0;
  uint32_t now = millis();
  float dt = (now - lastSpeedCalc) / 1000.0f;
  if (dt < 0.1f) return sensors.speed.value;  // terlalu cepat, pakai nilai lama
  noInterrupts();
  uint32_t pulses = speedPulseCount - lastPulseSnap;
  lastPulseSnap = speedPulseCount;
  interrupts();
  lastSpeedCalc = now;
  float km = (float)pulses / SPEED_PULSES_PER_KM;
  float speedKmh = km / dt * 3600.0f;
  return speedKmh;
}

/**
 * readAFR()
 * Wideband O2 controller output 0-5V -> voltage divider ke 0-3.3V
 * Rumus: V_adc = adcRaw * (ADC_VREF / ADC_RESOLUTION)
 *        V_sensor = V_adc * DIVIDER_RATIO (misal: 5/3.3 = 1.515)
 *        AFR = map(V_sensor, AFR_VOLTAGE_MIN, AFR_VOLTAGE_MAX, AFR_MIN, AFR_MAX)
 */
float readAFR() {
  if (SIMULATION_MODE) {
    return 14.2 + 1.5 * sin(simTime * 0.8) + 0.3 * sin(simTime * 3.1);
  }
  int raw = analogRead(PIN_AFR);
  sensors.afr.rawADC = raw;
  float vADC    = raw * (ADC_VREF / ADC_RESOLUTION);
  float vSensor = vADC * (5.0f / 3.3f);  // scale kembali ke 0-5V
  // Map voltage ke AFR
  float afr = AFR_VALUE_MIN + (vSensor - AFR_VOLTAGE_MIN) *
              (AFR_VALUE_MAX - AFR_VALUE_MIN) / (AFR_VOLTAGE_MAX - AFR_VOLTAGE_MIN);
  afr = constrain(afr, 8.0f, 22.0f);
  return afr;
}

/**
 * readEngineTemp()
 * Sensor NTC: Gunakan lookup table atau rumus Steinhart-Hart
 * Rumus linear (placeholder): Temp = map(adcRaw, ADC_MIN, ADC_MAX, TEMP_MIN, TEMP_MAX)
 * Untuk NTC real, gunakan: R = (ADC_MAX * R_FIXED) / (ADC_MAX - adcRaw) - R_FIXED
 * Lalu masukkan ke Steinhart-Hart: 1/T = A + B*ln(R) + C*ln(R)^3
 */
float readEngineTemp() {
  if (SIMULATION_MODE) {
    // Simulasi: mulai dingin, naik ke suhu normal
    float t = simTime;
    return 45 + 45 * (1 - exp(-t / 60)) + 5 * sin(t * 0.1);
  }
  int raw = analogRead(PIN_ENG_TEMP);
  sensors.engTemp.rawADC = raw;
  // Linearisasi sederhana (HARUS dikalibrasi ulang dengan sensor asli)
  float temp = TEMP_C_MIN + (float)(raw - TEMP_ADC_MIN) *
               (TEMP_C_MAX - TEMP_C_MIN) / (TEMP_ADC_MAX - TEMP_ADC_MIN);
  return constrain(temp, -20.0f, 200.0f);
}

/**
 * readBatteryVoltage()
 * Voltage divider: R1=10kΩ, R2=3.3kΩ
 * V_bat = V_adc * VBAT_DIVIDER_RATIO
 * PENTING: Tambahkan dioda zener 3.3V sebagai proteksi tambahan
 */
float readBatteryVoltage() {
  if (SIMULATION_MODE) {
    return 12.6 + 0.3 * sin(simTime * 0.15) + 0.1 * sin(simTime * 2.0);
  }
  int raw = analogRead(PIN_BATTERY);
  sensors.battery.rawADC = raw;
  float vADC = raw * (ADC_VREF / ADC_RESOLUTION);
  float vBat = vADC * VBAT_DIVIDER_RATIO;
  return vBat;
}

/**
 * readFuelLevel()
 * Sensor pelampung tangki: output resistif -> voltage divider -> ADC
 * Rumus: fuel% = map(adcRaw, FUEL_ADC_EMPTY, FUEL_ADC_FULL, 0, 100)
 * Kalibrasi: ukur ADC saat tangki kosong dan penuh
 */
float readFuelLevel() {
  if (SIMULATION_MODE) {
    // Perlahan turun sesuai waktu simulasi
    float fuel = 75.0f - simTime * 0.05f;
    return constrain(fuel, 0, 100);
  }
  int raw = analogRead(PIN_FUEL);
  sensors.fuelLevel.rawADC = raw;
  float fuel = (float)(raw - FUEL_ADC_EMPTY) * 100.0f /
               (FUEL_ADC_FULL - FUEL_ADC_EMPTY);
  return constrain(fuel, 0.0f, 100.0f);
}

/**
 * readTPS()
 * Throttle Position Sensor: 0.5V (tertutup) - 4.5V (penuh)
 * TPS% = (V_sensor - 0.5) / (4.5 - 0.5) * 100
 */
float readTPS() {
  if (SIMULATION_MODE) {
    float tps = 20 + 40 * abs(sin(simTime * 0.4));
    return constrain(tps, 0, 100);
  }
  int raw = analogRead(PIN_TPS);
  sensors.tps.rawADC = raw;
  float vADC    = raw * (ADC_VREF / ADC_RESOLUTION);
  float vSensor = vADC * (5.0f / 3.3f);
  float tps = (vSensor - 0.5f) / 4.0f * 100.0f;
  return constrain(tps, 0.0f, 100.0f);
}

/**
 * readMAP()
 * MAP sensor: tekanan intake manifold dalam kPa
 * Tipikal: 1Bar = 100kPa, output 0.5-4.5V -> 0-250kPa
 */
float readMAP() {
  if (SIMULATION_MODE) {
    return 40 + 40 * abs(sin(simTime * 0.4));
  }
  int raw = analogRead(PIN_MAP_SENSOR);
  sensors.mapSensor.rawADC = raw;
  float vADC = raw * (ADC_VREF / ADC_RESOLUTION);
  float vSensor = vADC * (5.0f / 3.3f);
  // Map 0.5-4.5V ke 10-110 kPa (tergantung sensor, sesuaikan)
  float kPa = 10.0f + (vSensor - 0.5f) / 4.0f * 100.0f;
  return constrain(kPa, 0.0f, 300.0f);
}

/**
 * readIAT()
 * Intake Air Temperature: sensor NTC di intake manifold
 */
float readIAT() {
  if (SIMULATION_MODE) {
    return 32 + 5 * sin(simTime * 0.05);
  }
  int raw = analogRead(PIN_IAT);
  sensors.iat.rawADC = raw;
  // Linearisasi sederhana (kalibrasi ulang sesuai sensor)
  float temp = -10.0f + (float)raw / ADC_RESOLUTION * 100.0f;
  return constrain(temp, -20.0f, 100.0f);
}

/**
 * readInjectorPulse()
 * Injector pulse width dalam ms
 * HARDWARE: Butuh optocoupler isolasi + timer capture
 * Placeholder: dihitung estimasi dari MAP dan RPM
 */
float readInjectorPulse() {
  if (SIMULATION_MODE) {
    // Estimasi dari MAP dan RPM
    float map_kPa = sensors.mapSensor.value;
    float rpm     = sensors.rpm.value;
    if (rpm < 100) return 0;
    float baseInj  = 2.5f;  // ms base pulse
    float loadCorr = map_kPa / 100.0f;
    return baseInj * loadCorr;
  }
  // TODO: Implementasi timer capture interrupt dari sinyal injector
  // Sinyal injector harus diisolasi optocoupler sebelum masuk ESP32
  return 0.0f;
}

// ============================================================
// FUNGSI FILTERING
// ============================================================

/**
 * movingAverage() - sudah built-in di struct MovingAvg
 * Cara pakai: sensors.rpm.value = sensors.rpm.ma.update(rawRPM);
 */

/**
 * lowPassFilter()
 * Output = alpha * newValue + (1 - alpha) * prevValue
 * alpha kecil = smooth lambat, alpha besar = lebih responsif
 */
float lowPassFilter(float newVal, float prevVal, float alpha = 0.2f) {
  return alpha * newVal + (1.0f - alpha) * prevVal;
}

// ============================================================
// FUNGSI ANALISIS
// ============================================================

/**
 * analyzeAFR()
 * Stoichiometric AFR bensin: 14.7
 * Lean  : AFR > 15.4
 * Rich  : AFR < 14.0
 * Normal: 14.0 - 15.4
 */
AFRStatus analyzeAFR(float afr) {
  if (afr > 15.5f) return AFR_LEAN;
  if (afr < 13.5f) return AFR_RICH;
  return AFR_NORMAL;
}

/**
 * analyzeTemperature()
 * Cold     : < 50°C
 * Normal   : 50 - 100°C
 * Hot      : 100 - 115°C
 * Overheat : > 115°C
 */
TempStatus analyzeTemperature(float temp) {
  if (temp < 50.0f)  return TEMP_COLD;
  if (temp < 100.0f) return TEMP_NORMAL;
  if (temp < 115.0f) return TEMP_HOT;
  return TEMP_OVERHEAT;
}

/**
 * analyzeBattery()
 * Low     : < 11.8V
 * Normal  : 11.8 - 14.8V
 * High    : > 14.8V (kemungkinan overcharging)
 */
BattStatus analyzeBattery(float volt) {
  if (volt < 11.8f) return BATT_LOW;
  if (volt < 14.8f) return BATT_NORMAL;
  return BATT_HIGH;
}

/**
 * analyzeRPM()
 * Idle     : < 1500 RPM
 * Normal   : 1500 - 7500 RPM
 * High     : 7500 - 9000 RPM
 * Over-rev : > 9000 RPM
 */
RPMStatus analyzeRPM(float rpm) {
  if (rpm < 1500.0f) return RPM_IDLE;
  if (rpm < 7500.0f) return RPM_NORMAL;
  if (rpm < 9000.0f) return RPM_HIGH;
  return RPM_OVERREV;
}

/**
 * calculateEngineHealth()
 * Skor 0-100% dari kombinasi beberapa faktor:
 * - AFR score    : 30% bobot
 * - Temp score   : 25% bobot
 * - Battery score: 20% bobot
 * - RPM stability: 15% bobot
 * - Sensor errors: 10% penalti
 */
float calculateEngineHealth() {
  float score = 100.0f;

  // AFR score (30%)
  float afrVal = sensors.afr.value;
  float afrDev = abs(afrVal - 14.7f);
  float afrScore = max(0.0f, 100.0f - afrDev * 15.0f);
  score = score * 0.7f + afrScore * 0.3f;

  // Temp score (25%)
  float tempScore = 0;
  float t = sensors.engTemp.value;
  if (t >= 70 && t <= 95)      tempScore = 100;
  else if (t >= 50 && t < 70)  tempScore = 70 + (t - 50) * 1.5f;
  else if (t > 95 && t <= 110) tempScore = 100 - (t - 95) * 8.0f;
  else if (t > 110)             tempScore = max(0.0f, 20.0f - (t - 110) * 4.0f);
  else                          tempScore = max(0.0f, (t + 10) * 2.0f);
  score = score * 0.75f + tempScore * 0.25f;

  // Battery score (20%)
  float battScore = 0;
  float v = sensors.battery.value;
  if (v >= 12.4f && v <= 14.4f) battScore = 100;
  else if (v >= 11.8f)           battScore = 60 + (v - 11.8f) * 100.0f;
  else                           battScore = max(0.0f, v * 5.0f);
  score = score * 0.80f + battScore * 0.20f;

  // RPM stability (15%) - variance dari moving average
  static float lastRPM = 0;
  float rpmDelta = abs(sensors.rpm.value - lastRPM);
  lastRPM = sensors.rpm.value;
  float rpmScore = max(0.0f, 100.0f - rpmDelta / 50.0f);
  score = score * 0.85f + rpmScore * 0.15f;

  // Penalti sensor error (10%)
  int errorCount = 0;
  if (sensors.afr.status == STATUS_ERROR)     errorCount++;
  if (sensors.engTemp.status == STATUS_ERROR) errorCount++;
  if (sensors.battery.status == STATUS_ERROR) errorCount++;
  if (sensors.rpm.status == STATUS_ERROR)     errorCount++;
  score -= errorCount * 10.0f;

  return constrain(score, 0.0f, 100.0f);
}

/**
 * calculateFuelConsumption()
 * Estimasi konsumsi BBM L/100km
 * Rumus: FC = (injPulse_ms * RPM/60 * fuelDensity * injectorFlow) / speed
 * Injector flow tipikal: 230cc/min
 * Densitas bensin: ~740 g/L
 * Ini estimasi kasar - sensor flow rate real lebih akurat
 */
float calculateFuelConsumption() {
  float rpm   = sensors.rpm.value;
  float speed = sensors.speed.value;
  float injPW = sensors.injPulse.value;  // ms
  if (rpm < 100 || speed < 1) return 0.0f;

  float injFlowCC_per_min = 230.0f;  // cc/min injector rating
  // Durasi injeksi per detik: (injPW_ms/1000) * (RPM/60) * jumlah_silinder
  float injDutyPerSec = (injPW / 1000.0f) * (rpm / 60.0f) * 1;  // asumsi 1 silinder
  float fuelCC_per_sec = injFlowCC_per_min / 60.0f * injDutyPerSec;
  float fuelL_per_sec  = fuelCC_per_sec / 1000.0f;
  float fuelL_per_hour = fuelL_per_sec * 3600.0f;
  float speedKmh       = speed;
  float fcL100km       = (fuelL_per_hour / speedKmh) * 100.0f;
  return constrain(fcL100km, 0.0f, 30.0f);
}

// ============================================================
// WARNA TEMA (TFT color 16-bit)
// ============================================================
#define CLR_BG          0x0841   // Dark background
#define CLR_PANEL       0x10A2   // Panel background
#define CLR_ACCENT      0x07FF   // Cyan accent
#define CLR_GREEN       0x07E0   // Green
#define CLR_YELLOW      0xFFE0   // Yellow
#define CLR_ORANGE      0xFD20   // Orange
#define CLR_RED         0xF800   // Red
#define CLR_WHITE       0xFFFF
#define CLR_GRAY        0x7BEF
#define CLR_DARKGRAY    0x39E7
#define CLR_TEXT        0xDEDB   // Light gray text
#define CLR_LEAN        0x07FF   // Cyan = lean
#define CLR_RICH        0xF81F   // Magenta = rich
#define CLR_NORMAL_AFR  0x07E0   // Green = normal AFR

// Helper: warna berdasarkan status
uint16_t statusColor(SensorStatus s) {
  switch(s) {
    case STATUS_OK:      return CLR_GREEN;
    case STATUS_WARNING: return CLR_YELLOW;
    case STATUS_ERROR:   return CLR_RED;
    default:             return CLR_GRAY;
  }
}

uint16_t healthColor(float pct) {
  if (pct >= 80) return CLR_GREEN;
  if (pct >= 60) return CLR_YELLOW;
  if (pct >= 40) return CLR_ORANGE;
  return CLR_RED;
}

// ============================================================
// FUNGSI UI - KOMPONEN DASAR
// ============================================================

/**
 * drawMiniGraph()
 * Gambar grafik mini di area yang ditentukan
 * data[]  : array float titik data
 * n       : jumlah titik
 * x,y,w,h : posisi dan ukuran area grafik
 * minV, maxV : range nilai
 * color   : warna garis
 */
void drawMiniGraph(float* data, int n, int x, int y, int w, int h,
                   float minV, float maxV, uint16_t color, const char* label) {
  // Background panel
  tft.fillRect(x, y, w, h, CLR_PANEL);
  tft.drawRect(x, y, w, h, CLR_DARKGRAY);

  // Label
  tft.setTextColor(CLR_GRAY, CLR_PANEL);
  tft.setTextSize(1);
  tft.setCursor(x + 3, y + 2);
  tft.print(label);

  if (n < 2) return;
  int plotH = h - 14;
  int plotY = y + 10;

  float range = maxV - minV;
  if (range <= 0) range = 1;

  int prevPx = x;
  int prevPy = plotY + plotH - (int)((data[0] - minV) / range * plotH);
  prevPy = constrain(prevPy, plotY, plotY + plotH);

  for (int i = 1; i < n; i++) {
    int px = x + (int)((float)i / (n - 1) * w);
    float val = data[i];
    int py = plotY + plotH - (int)((val - minV) / range * plotH);
    py = constrain(py, plotY, plotY + plotH);
    tft.drawLine(prevPx, prevPy, px, py, color);
    prevPx = px;
    prevPy = py;
  }
  // Nilai terakhir
  tft.setTextColor(color, CLR_PANEL);
  tft.setCursor(x + w - 32, y + h - 10);
  if (data[n-1] >= 1000)
    tft.printf("%.0f", data[n-1]);
  else
    tft.printf("%.1f", data[n-1]);
}

/**
 * drawGauge()
 * Gauge bar horizontal sederhana
 */
void drawGauge(int x, int y, int w, int h, float val, float minV, float maxV,
               uint16_t color, const char* label, const char* unit) {
  tft.fillRect(x, y, w, h, CLR_PANEL);
  tft.drawRect(x, y, w, h, CLR_DARKGRAY);

  float pct = constrain((val - minV) / (maxV - minV), 0.0f, 1.0f);
  int fillW = (int)(pct * (w - 4));
  tft.fillRect(x + 2, y + 2, fillW, h - 4, color);

  tft.setTextColor(CLR_WHITE, CLR_PANEL);
  tft.setTextSize(1);
  tft.setCursor(x + 4, y + (h - 7) / 2);
  tft.printf("%s: %.1f%s", label, val, unit);
}

/**
 * drawRPMBar()
 * RPM bar visual dengan zona warning
 */
void drawRPMBar(int x, int y, int w, int h, float rpm, float maxRPM) {
  tft.fillRect(x, y, w, h, CLR_PANEL);
  tft.drawRect(x, y, w, h, CLR_DARKGRAY);
  float pct = constrain(rpm / maxRPM, 0.0f, 1.0f);
  int fillW = (int)(pct * (w - 4));

  uint16_t barColor;
  if (pct < 0.6f)      barColor = CLR_GREEN;
  else if (pct < 0.8f) barColor = CLR_YELLOW;
  else if (pct < 0.9f) barColor = CLR_ORANGE;
  else                 barColor = CLR_RED;

  tft.fillRect(x + 2, y + 2, fillW, h - 4, barColor);

  // Zone markers
  tft.drawLine(x + (int)(0.6f * (w-4)) + 2, y+1, x + (int)(0.6f * (w-4)) + 2, y+h-1, CLR_YELLOW);
  tft.drawLine(x + (int)(0.8f * (w-4)) + 2, y+1, x + (int)(0.8f * (w-4)) + 2, y+h-1, CLR_ORANGE);
  tft.drawLine(x + (int)(0.9f * (w-4)) + 2, y+1, x + (int)(0.9f * (w-4)) + 2, y+h-1, CLR_RED);
}

/**
 * drawWarningPanel()
 * Panel peringatan di bagian bawah layar
 */
void drawWarningPanel() {
  bool hasWarning = false;
  char warnMsg[64] = "";

  if (afrStatus == AFR_LEAN)          { strcpy(warnMsg, "! AFR LEAN - CAMPURAN TERLALU KURUS"); hasWarning = true; }
  else if (afrStatus == AFR_RICH)     { strcpy(warnMsg, "! AFR RICH - CAMPURAN TERLALU GEMUK"); hasWarning = true; }
  if (tempStatus == TEMP_OVERHEAT)    { strcpy(warnMsg, "!!! MESIN OVERHEAT - SEGERA MATIKAN"); hasWarning = true; }
  else if (tempStatus == TEMP_HOT)    { strcpy(warnMsg, "! SUHU MESIN TINGGI"); hasWarning = true; }
  if (battStatus == BATT_LOW)         { strcpy(warnMsg, "! TEGANGAN BATERAI RENDAH"); hasWarning = true; }
  if (rpmStatus == RPM_OVERREV)       { strcpy(warnMsg, "!!! OVER-REV - LEPAS GAS SEGERA"); hasWarning = true; }

  int wy = SCREEN_H - 22;
  if (hasWarning) {
    // Blink effect menggunakan millis
    bool blink = (millis() / 400) % 2 == 0;
    tft.fillRect(0, wy, SCREEN_W, 22, blink ? CLR_RED : CLR_ORANGE);
    tft.setTextColor(CLR_WHITE, blink ? CLR_RED : CLR_ORANGE);
    tft.setTextSize(1);
    tft.setCursor(8, wy + 7);
    tft.print(warnMsg);
    warningVisible = true;
  } else {
    if (warningVisible) {
      tft.fillRect(0, wy, SCREEN_W, 22, CLR_BG);
      warningVisible = false;
    }
  }
}

// ============================================================
// FUNGSI UI - MODE DASHBOARD
// ============================================================
void drawDashboardMode() {
  if (needFullRedraw) {
    tft.fillScreen(CLR_BG);
    // Header bar
    tft.fillRect(0, 0, SCREEN_W, 28, CLR_PANEL);
    tft.setTextColor(CLR_ACCENT, CLR_PANEL);
    tft.setTextSize(2);
    tft.setCursor(8, 6);
    tft.print("MOTO ECU DASHBOARD");
    tft.setTextSize(1);
    tft.setTextColor(CLR_GRAY, CLR_PANEL);
    tft.setCursor(SCREEN_W - 100, 10);
    tft.print("[DASHBOARD]");
    needFullRedraw = false;
  }

  int y0 = 32;

  // --- SPEED (Besar di kiri atas) ---
  tft.fillRect(2, y0, 180, 80, CLR_PANEL);
  tft.drawRect(2, y0, 180, 80, CLR_DARKGRAY);
  tft.setTextColor(CLR_GRAY, CLR_PANEL);
  tft.setTextSize(1);
  tft.setCursor(10, y0 + 5);
  tft.print("SPEED");
  tft.setTextColor(CLR_ACCENT, CLR_PANEL);
  tft.setTextSize(4);
  tft.setCursor(10, y0 + 18);
  tft.printf("%.0f", sensors.speed.value);
  tft.setTextSize(1);
  tft.setTextColor(CLR_GRAY, CLR_PANEL);
  tft.setCursor(10, y0 + 66);
  tft.print("km/h");

  // --- RPM ---
  tft.fillRect(186, y0, 180, 80, CLR_PANEL);
  tft.drawRect(186, y0, 180, 80, CLR_DARKGRAY);
  tft.setTextColor(CLR_GRAY, CLR_PANEL);
  tft.setTextSize(1);
  tft.setCursor(194, y0 + 5);
  tft.print("RPM");
  uint16_t rpmColor = (rpmStatus == RPM_OVERREV) ? CLR_RED :
                      (rpmStatus == RPM_HIGH) ? CLR_ORANGE : CLR_GREEN;
  tft.setTextColor(rpmColor, CLR_PANEL);
  tft.setTextSize(3);
  tft.setCursor(194, y0 + 20);
  tft.printf("%.0f", sensors.rpm.value);
  // RPM bar
  drawRPMBar(188, y0 + 62, 176, 14, sensors.rpm.value, 10000);

  // --- AFR ---
  tft.fillRect(370, y0, 140, 80, CLR_PANEL);
  tft.drawRect(370, y0, 140, 80, CLR_DARKGRAY);
  tft.setTextColor(CLR_GRAY, CLR_PANEL);
  tft.setTextSize(1);
  tft.setCursor(378, y0 + 5);
  tft.print("AFR");
  uint16_t afrColor = (afrStatus == AFR_LEAN) ? CLR_LEAN :
                      (afrStatus == AFR_RICH) ? CLR_RICH : CLR_GREEN;
  tft.setTextColor(afrColor, CLR_PANEL);
  tft.setTextSize(3);
  tft.setCursor(378, y0 + 20);
  tft.printf("%.1f", sensors.afr.value);
  tft.setTextSize(1);
  tft.setTextColor(CLR_GRAY, CLR_PANEL);
  tft.setCursor(378, y0 + 52);
  const char* afrStr = (afrStatus == AFR_LEAN) ? "LEAN" :
                       (afrStatus == AFR_RICH) ? "RICH" : "NORMAL";
  tft.print(afrStr);

  // --- ENGINE TEMP ---
  tft.fillRect(514, y0, 140, 80, CLR_PANEL);
  tft.drawRect(514, y0, 140, 80, CLR_DARKGRAY);
  tft.setTextColor(CLR_GRAY, CLR_PANEL);
  tft.setTextSize(1);
  tft.setCursor(522, y0 + 5);
  tft.print("ENG TEMP");
  uint16_t tempColor = (tempStatus == TEMP_OVERHEAT) ? CLR_RED :
                       (tempStatus == TEMP_HOT) ? CLR_ORANGE :
                       (tempStatus == TEMP_COLD) ? CLR_ACCENT : CLR_GREEN;
  tft.setTextColor(tempColor, CLR_PANEL);
  tft.setTextSize(3);
  tft.setCursor(522, y0 + 20);
  tft.printf("%.0f", sensors.engTemp.value);
  tft.setTextSize(1);
  tft.setCursor(522, y0 + 52);
  tft.print("\xB0""C");

  // --- BATTERY ---
  tft.fillRect(658, y0, 138, 80, CLR_PANEL);
  tft.drawRect(658, y0, 138, 80, CLR_DARKGRAY);
  tft.setTextColor(CLR_GRAY, CLR_PANEL);
  tft.setTextSize(1);
  tft.setCursor(666, y0 + 5);
  tft.print("BATTERY");
  uint16_t battColor = (battStatus == BATT_LOW) ? CLR_RED :
                       (battStatus == BATT_HIGH) ? CLR_ORANGE : CLR_GREEN;
  tft.setTextColor(battColor, CLR_PANEL);
  tft.setTextSize(3);
  tft.setCursor(666, y0 + 20);
  tft.printf("%.1f", sensors.battery.value);
  tft.setTextSize(1);
  tft.setCursor(666, y0 + 52);
  tft.print("Volt");

  // --- BARIS KEDUA: Fuel, Engine Health, Fuel Consumption, TPS ---
  int y1 = y0 + 84;
  // Fuel level
  drawGauge(2, y1, 180, 24, sensors.fuelLevel.value, 0, 100,
            sensors.fuelLevel.value < 15 ? CLR_RED : CLR_GREEN, "FUEL", "%");
  // Engine Health
  tft.fillRect(186, y1, 240, 24, CLR_PANEL);
  tft.drawRect(186, y1, 240, 24, CLR_DARKGRAY);
  float hp = engineHealth;
  int fillW2 = (int)((hp / 100.0f) * 236);
  tft.fillRect(188, y1 + 2, fillW2, 20, healthColor(hp));
  tft.setTextColor(CLR_WHITE, CLR_PANEL);
  tft.setTextSize(1);
  tft.setCursor(194, y1 + 8);
  const char* healthStr = (healthStatus == HEALTH_EXCELLENT) ? "EXCELLENT" :
                          (healthStatus == HEALTH_NORMAL)    ? "NORMAL" :
                          (healthStatus == HEALTH_WARNING)   ? "WARNING" : "CRITICAL";
  tft.printf("ENGINE HEALTH: %.0f%% - %s", hp, healthStr);
  // Fuel consumption
  tft.fillRect(430, y1, 200, 24, CLR_PANEL);
  tft.drawRect(430, y1, 200, 24, CLR_DARKGRAY);
  tft.setTextColor(CLR_TEXT, CLR_PANEL);
  tft.setTextSize(1);
  tft.setCursor(438, y1 + 8);
  tft.printf("KONSUMSI: %.1f L/100km", fuelConsumptionL100);
  // TPS
  drawGauge(634, y1, 162, 24, sensors.tps.value, 0, 100, CLR_ACCENT, "TPS", "%");

  // --- MINI GRAPHS ---
  int gy = y1 + 28;
  int gw = (SCREEN_W / 4) - 4;
  int gh = 90;
  drawMiniGraph(graphRPM,  GRAPH_POINTS, 2,           gy, gw, gh, 0, 10000, CLR_GREEN,  "RPM");
  drawMiniGraph(graphAFR,  GRAPH_POINTS, gw + 6,      gy, gw, gh, 10, 20,  CLR_ACCENT, "AFR");
  drawMiniGraph(graphTemp, GRAPH_POINTS, (gw+4)*2 + 2, gy, gw, gh, 0, 150, CLR_ORANGE, "TEMP");
  drawMiniGraph(graphBatt, GRAPH_POINTS, (gw+4)*3 + 2, gy, gw, gh, 10, 16, CLR_YELLOW, "BATT");

  // --- STATUS SENSOR BARIS ---
  int sy = gy + gh + 4;
  tft.fillRect(0, sy, SCREEN_W, 20, CLR_PANEL);

  struct { const char* name; SensorStatus st; } statusList[] = {
    {"RPM", sensors.rpm.status},
    {"SPD", sensors.speed.status},
    {"AFR", sensors.afr.status},
    {"TMP", sensors.engTemp.status},
    {"BAT", sensors.battery.status},
    {"FUL", sensors.fuelLevel.status},
    {"TPS", sensors.tps.status},
    {"MAP", sensors.mapSensor.status},
    {"IAT", sensors.iat.status},
  };

  int sx = 4;
  for (auto& s : statusList) {
    uint16_t sc = statusColor(s.st);
    tft.fillRect(sx, sy + 2, 6, 16, sc);
    tft.setTextColor(CLR_TEXT, CLR_PANEL);
    tft.setTextSize(1);
    tft.setCursor(sx + 8, sy + 6);
    tft.print(s.name);
    sx += 72;
  }

  drawWarningPanel();
}

// ============================================================
// FUNGSI UI - ECU MAPPING MODE
// ============================================================
void drawECUMappingMode() {
  if (needFullRedraw) {
    tft.fillScreen(CLR_BG);
    tft.fillRect(0, 0, SCREEN_W, 28, CLR_PANEL);
    tft.setTextColor(CLR_ACCENT, CLR_PANEL);
    tft.setTextSize(2);
    tft.setCursor(8, 6);
    tft.print("ECU FUEL MAP");
    tft.setTextSize(1);
    tft.setTextColor(CLR_GRAY, CLR_PANEL);
    tft.setCursor(SCREEN_W - 130, 10);
    tft.print("[ECU MAPPING]");
    needFullRedraw = false;
  }

  // ECU Map definition
  const int RPM_COLS = 8;
  const int TPS_ROWS = 5;
  const int rpmPoints[RPM_COLS] = {1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000};
  const int tpsPoints[TPS_ROWS] = {0, 25, 50, 75, 100};

  // Dummy AFR target map (kolom=RPM, baris=TPS)
  float afrMap[TPS_ROWS][RPM_COLS] = {
    {14.7, 14.7, 14.7, 14.7, 14.7, 14.7, 14.7, 14.7},  // TPS 0%
    {14.7, 14.5, 14.3, 14.2, 14.0, 13.8, 13.7, 13.6},  // TPS 25%
    {14.5, 14.2, 13.9, 13.7, 13.5, 13.3, 13.2, 13.1},  // TPS 50%
    {14.3, 14.0, 13.7, 13.4, 13.2, 13.0, 12.9, 12.8},  // TPS 75%
    {14.0, 13.8, 13.5, 13.2, 13.0, 12.8, 12.7, 12.6},  // TPS 100%
  };

  // Temukan cell aktif berdasarkan RPM dan TPS real-time
  float curRPM = sensors.rpm.value;
  float curTPS = sensors.tps.value;
  int activeCol = 0, activeRow = 0;
  for (int c = 1; c < RPM_COLS; c++)
    if (curRPM > rpmPoints[c]) activeCol = c;
  for (int r = 1; r < TPS_ROWS; r++)
    if (curTPS > tpsPoints[r]) activeRow = r;

  // Gambar tabel
  int startX = 60;
  int startY = 40;
  int cellW  = 86;
  int cellH  = 48;

  // Header RPM
  for (int c = 0; c < RPM_COLS; c++) {
    int cx = startX + c * cellW;
    tft.setTextColor(CLR_ACCENT, CLR_BG);
    tft.setTextSize(1);
    tft.setCursor(cx + 4, startY - 14);
    tft.printf("%d", rpmPoints[c]);
  }
  tft.setCursor(startX + RPM_COLS * cellW / 2 - 20, 32);
  tft.setTextColor(CLR_GRAY, CLR_BG);
  tft.print("RPM -->");

  // Header TPS (sumbu Y)
  for (int r = 0; r < TPS_ROWS; r++) {
    int ry = startY + r * cellH;
    tft.setTextColor(CLR_ACCENT, CLR_BG);
    tft.setTextSize(1);
    tft.setCursor(2, ry + cellH / 2 - 4);
    tft.printf("%3d%%", tpsPoints[r]);
  }

  // Gambar cells
  for (int r = 0; r < TPS_ROWS; r++) {
    for (int c = 0; c < RPM_COLS; c++) {
      int cx = startX + c * cellW;
      int ry = startY + r * cellH;
      float afr = afrMap[r][c];

      // Warna berdasarkan AFR
      uint16_t cellColor;
      if (afr > 15.0f)     cellColor = 0x001F;  // Blue = lean
      else if (afr < 13.5f) cellColor = 0x4800; // Dark red = rich
      else                  cellColor = 0x0240;  // Dark green = normal

      // Cell aktif
      bool isActive = (r == activeRow && c == activeCol);
      if (isActive) {
        tft.fillRect(cx + 1, ry + 1, cellW - 2, cellH - 2, CLR_WHITE);
        tft.setTextColor(CLR_BG, CLR_WHITE);
      } else {
        tft.fillRect(cx + 1, ry + 1, cellW - 2, cellH - 2, cellColor);
        tft.setTextColor(CLR_WHITE, cellColor);
      }
      tft.drawRect(cx, ry, cellW, cellH, CLR_DARKGRAY);
      tft.setTextSize(1);
      tft.setCursor(cx + 6, ry + cellH / 2 - 4);
      tft.printf("%.1f", afr);
    }
  }

  // Info aktif
  int infoY = startY + TPS_ROWS * cellH + 8;
  tft.fillRect(0, infoY, SCREEN_W, 30, CLR_PANEL);
  tft.setTextColor(CLR_ACCENT, CLR_PANEL);
  tft.setTextSize(1);
  tft.setCursor(8, infoY + 4);
  tft.printf("AKTIF: RPM=%.0f | TPS=%.0f%% | AFR Target=%.1f | AFR Actual=%.1f",
             curRPM, curTPS, afrMap[activeRow][activeCol], sensors.afr.value);
  tft.setCursor(8, infoY + 16);
  tft.setTextColor(CLR_GRAY, CLR_PANEL);
  tft.print("Legend: [BIRU]=Lean [HIJAU]=Normal [MERAH]=Rich | Putih=Cell Aktif");

  drawWarningPanel();
}

// ============================================================
// FUNGSI UI - DIAGNOSTIC MODE
// ============================================================
void drawDiagnosticMode() {
  if (needFullRedraw) {
    tft.fillScreen(CLR_BG);
    tft.fillRect(0, 0, SCREEN_W, 28, CLR_PANEL);
    tft.setTextColor(CLR_ACCENT, CLR_PANEL);
    tft.setTextSize(2);
    tft.setCursor(8, 6);
    tft.print("DIAGNOSTIC MODE");
    tft.setTextColor(CLR_GRAY, CLR_PANEL);
    tft.setTextSize(1);
    tft.setCursor(SCREEN_W - 130, 10);
    tft.print("[DIAGNOSTIC]");
    // Header tabel
    tft.setTextColor(CLR_ACCENT, CLR_BG);
    tft.setCursor(4,   32); tft.print("SENSOR");
    tft.setCursor(160, 32); tft.print("VALUE");
    tft.setCursor(280, 32); tft.print("STATUS");
    tft.setCursor(380, 32); tft.print("LAST UPDATE");
    tft.setCursor(520, 32); tft.print("ANALYSIS");
    tft.drawLine(0, 42, SCREEN_W, 42, CLR_DARKGRAY);
    needFullRedraw = false;
  }

  struct DiagEntry {
    const char* name;
    float       value;
    const char* unit;
    SensorStatus status;
    uint32_t    lastUpdate;
    const char* analysis;
  };

  // Analisis string
  const char* afrAn = (afrStatus == AFR_LEAN) ? "LEAN" :
                      (afrStatus == AFR_RICH)  ? "RICH" : "NORMAL";
  const char* tmpAn = (tempStatus == TEMP_OVERHEAT) ? "OVERHEAT" :
                      (tempStatus == TEMP_HOT)  ? "HOT" :
                      (tempStatus == TEMP_COLD) ? "COLD" : "NORMAL";
  const char* batAn = (battStatus == BATT_LOW)  ? "LOW" :
                      (battStatus == BATT_HIGH) ? "HIGH" : "NORMAL";
  const char* rpmAn = (rpmStatus == RPM_OVERREV) ? "OVER-REV" :
                      (rpmStatus == RPM_HIGH)  ? "HIGH" :
                      (rpmStatus == RPM_IDLE)  ? "IDLE" : "NORMAL";

  DiagEntry entries[] = {
    {"RPM",            sensors.rpm.value,        "rpm", sensors.rpm.status,        sensors.rpm.lastUpdate,        rpmAn},
    {"SPEED",          sensors.speed.value,       "kmh", sensors.speed.status,      sensors.speed.lastUpdate,      "-"},
    {"AFR",            sensors.afr.value,         "",    sensors.afr.status,         sensors.afr.lastUpdate,        afrAn},
    {"ENGINE TEMP",    sensors.engTemp.value,     "C",   sensors.engTemp.status,    sensors.engTemp.lastUpdate,    tmpAn},
    {"BATTERY",        sensors.battery.value,     "V",   sensors.battery.status,    sensors.battery.lastUpdate,    batAn},
    {"FUEL LEVEL",     sensors.fuelLevel.value,   "%",   sensors.fuelLevel.status,  sensors.fuelLevel.lastUpdate,  sensors.fuelLevel.value < 15 ? "LOW" : "OK"},
    {"TPS",            sensors.tps.value,         "%",   sensors.tps.status,        sensors.tps.lastUpdate,        "-"},
    {"MAP",            sensors.mapSensor.value,   "kPa", sensors.mapSensor.status,  sensors.mapSensor.lastUpdate,  "-"},
    {"IAT",            sensors.iat.value,         "C",   sensors.iat.status,        sensors.iat.lastUpdate,        "-"},
    {"INJ PULSE",      sensors.injPulse.value,    "ms",  sensors.injPulse.status,   sensors.injPulse.lastUpdate,   "-"},
    {"ENGINE HEALTH",  engineHealth,              "%",   STATUS_OK,                  millis(),                      (healthStatus==HEALTH_EXCELLENT)?"EXCELLENT":(healthStatus==HEALTH_NORMAL)?"NORMAL":(healthStatus==HEALTH_WARNING)?"WARNING":"CRITICAL"},
  };

  int rowH = 36;
  int startY = 46;
  for (int i = 0; i < (int)(sizeof(entries)/sizeof(entries[0])); i++) {
    int ry = startY + i * rowH;
    uint16_t rowBg = (i % 2 == 0) ? CLR_PANEL : CLR_BG;
    tft.fillRect(0, ry, SCREEN_W, rowH - 2, rowBg);

    tft.setTextColor(CLR_TEXT, rowBg);
    tft.setTextSize(1);
    tft.setCursor(4,   ry + 13);  tft.print(entries[i].name);
    tft.setCursor(160, ry + 13);  tft.printf("%.2f %s", entries[i].value, entries[i].unit);

    // Status dot + text
    uint16_t sc = statusColor(entries[i].status);
    tft.fillCircle(284, ry + 14, 5, sc);
    tft.setTextColor(sc, rowBg);
    tft.setCursor(294, ry + 10);
    tft.setTextSize(1);
    const char* stStr = (entries[i].status == STATUS_OK)      ? "OK" :
                        (entries[i].status == STATUS_WARNING)  ? "WARN" :
                        (entries[i].status == STATUS_ERROR)    ? "ERROR" : "OFFLINE";
    tft.print(stStr);

    // Last update
    uint32_t age = millis() - entries[i].lastUpdate;
    tft.setTextColor(CLR_GRAY, rowBg);
    tft.setCursor(380, ry + 13);
    tft.printf("%lu ms ago", age);

    // Analysis
    tft.setTextColor(sc, rowBg);
    tft.setCursor(520, ry + 13);
    tft.print(entries[i].analysis);
  }
}

// ============================================================
// FUNGSI UI - SENSOR MONITOR MODE
// ============================================================
void drawSensorMonitorMode() {
  if (needFullRedraw) {
    tft.fillScreen(CLR_BG);
    tft.fillRect(0, 0, SCREEN_W, 28, CLR_PANEL);
    tft.setTextColor(CLR_ACCENT, CLR_PANEL);
    tft.setTextSize(2);
    tft.setCursor(8, 6);
    tft.print("SENSOR MONITOR");
    tft.setTextColor(CLR_GRAY, CLR_PANEL);
    tft.setTextSize(1);
    tft.setCursor(SCREEN_W - 160, 10);
    tft.print("[SENSOR MONITOR]");
    tft.setTextColor(CLR_ACCENT, CLR_BG);
    tft.setCursor(4,   32); tft.print("SENSOR");
    tft.setCursor(120, 32); tft.print("RAW ADC");
    tft.setCursor(200, 32); tft.print("VALUE");
    tft.setCursor(290, 32); tft.print("MIN");
    tft.setCursor(360, 32); tft.print("MAX");
    tft.setCursor(430, 32); tft.print("AVG");
    tft.setCursor(510, 32); tft.print("UNIT");
    tft.drawLine(0, 42, SCREEN_W, 42, CLR_DARKGRAY);
    needFullRedraw = false;
  }

  struct MonEntry {
    const char* name;
    float raw;
    float val;
    float mn;
    float mx;
    float avg;
    const char* unit;
  };

  MonEntry entries[] = {
    {"RPM",         0,                    sensors.rpm.value,      sensors.rpm.minVal,      sensors.rpm.maxVal,      sensors.rpm.avgVal,      "rpm"},
    {"SPEED",       0,                    sensors.speed.value,    sensors.speed.minVal,    sensors.speed.maxVal,    sensors.speed.avgVal,    "km/h"},
    {"AFR",         sensors.afr.rawADC,   sensors.afr.value,      sensors.afr.minVal,      sensors.afr.maxVal,      sensors.afr.avgVal,      ""},
    {"ENG TEMP",    sensors.engTemp.rawADC, sensors.engTemp.value, sensors.engTemp.minVal, sensors.engTemp.maxVal,  sensors.engTemp.avgVal,  "C"},
    {"BATTERY",     sensors.battery.rawADC, sensors.battery.value, sensors.battery.minVal, sensors.battery.maxVal, sensors.battery.avgVal,  "V"},
    {"FUEL LEVEL",  sensors.fuelLevel.rawADC, sensors.fuelLevel.value, sensors.fuelLevel.minVal, sensors.fuelLevel.maxVal, sensors.fuelLevel.avgVal, "%"},
    {"TPS",         sensors.tps.rawADC,   sensors.tps.value,      sensors.tps.minVal,      sensors.tps.maxVal,      sensors.tps.avgVal,      "%"},
    {"MAP",         sensors.mapSensor.rawADC, sensors.mapSensor.value, sensors.mapSensor.minVal, sensors.mapSensor.maxVal, sensors.mapSensor.avgVal, "kPa"},
    {"IAT",         sensors.iat.rawADC,   sensors.iat.value,      sensors.iat.minVal,      sensors.iat.maxVal,      sensors.iat.avgVal,      "C"},
    {"INJ PULSE",   0,                    sensors.injPulse.value, sensors.injPulse.minVal, sensors.injPulse.maxVal, sensors.injPulse.avgVal, "ms"},
  };

  int rowH = 36;
  int startY = 46;
  for (int i = 0; i < (int)(sizeof(entries)/sizeof(entries[0])); i++) {
    int ry = startY + i * rowH;
    uint16_t rowBg = (i % 2 == 0) ? CLR_PANEL : CLR_BG;
    tft.fillRect(0, ry, SCREEN_W, rowH - 2, rowBg);
    tft.setTextColor(CLR_TEXT, rowBg);
    tft.setTextSize(1);
    tft.setCursor(4,   ry + 13); tft.print(entries[i].name);
    tft.setCursor(120, ry + 13); tft.printf("%.0f", entries[i].raw);
    tft.setCursor(200, ry + 13); tft.printf("%.2f", entries[i].val);
    tft.setTextColor(CLR_ACCENT, rowBg);
    tft.setCursor(290, ry + 13); tft.printf("%.1f", entries[i].mn);
    tft.setTextColor(CLR_ORANGE, rowBg);
    tft.setCursor(360, ry + 13); tft.printf("%.1f", entries[i].mx);
    tft.setTextColor(CLR_GRAY, rowBg);
    tft.setCursor(430, ry + 13); tft.printf("%.2f", entries[i].avg);
    tft.setTextColor(CLR_DARKGRAY, rowBg);
    tft.setCursor(510, ry + 13); tft.print(entries[i].unit);
  }
}

// ============================================================
// UPDATE DATA SENSOR (non-blocking)
// ============================================================
void updateSimulationTime() {
  simTime = millis() / 1000.0f;
}

void updateFastSensors() {
  // RPM
  float rawRPM = readRPM();
  sensors.rpm.updateStats(sensors.rpm.ma.update(rawRPM));
  sensors.rpm.status = (sensors.rpm.value > 0) ? STATUS_OK : STATUS_WARNING;

  // Speed
  float rawSpd = readSpeed();
  sensors.speed.updateStats(sensors.speed.ma.update(rawSpd));
  sensors.speed.status = STATUS_OK;

  // AFR
  float rawAFR = readAFR();
  sensors.afr.updateStats(sensors.afr.ma.update(rawAFR));
  sensors.afr.status = (sensors.afr.value >= 8 && sensors.afr.value <= 22) ? STATUS_OK : STATUS_ERROR;

  // TPS
  float rawTPS = readTPS();
  sensors.tps.updateStats(sensors.tps.ma.update(rawTPS));
  sensors.tps.status = STATUS_OK;

  // MAP
  float rawMAP = readMAP();
  sensors.mapSensor.updateStats(sensors.mapSensor.ma.update(rawMAP));
  sensors.mapSensor.status = STATUS_OK;

  // Injector pulse
  float rawInj = readInjectorPulse();
  sensors.injPulse.updateStats(sensors.injPulse.ma.update(rawInj));
  sensors.injPulse.status = STATUS_OK;

  // Analisis cepat
  afrStatus = analyzeAFR(sensors.afr.value);
  rpmStatus = analyzeRPM(sensors.rpm.value);
}

void updateSlowSensors() {
  // Engine temp
  float rawTemp = readEngineTemp();
  sensors.engTemp.updateStats(lowPassFilter(rawTemp, sensors.engTemp.value, 0.15f));
  sensors.engTemp.status = (sensors.engTemp.value < 200) ? STATUS_OK : STATUS_ERROR;

  // Battery
  float rawBatt = readBatteryVoltage();
  sensors.battery.updateStats(lowPassFilter(rawBatt, sensors.battery.value, 0.2f));
  sensors.battery.status = (sensors.battery.value > 8 && sensors.battery.value < 18) ? STATUS_OK : STATUS_ERROR;

  // Fuel
  float rawFuel = readFuelLevel();
  sensors.fuelLevel.updateStats(lowPassFilter(rawFuel, sensors.fuelLevel.value, 0.1f));
  sensors.fuelLevel.status = (sensors.fuelLevel.value < 10) ? STATUS_WARNING : STATUS_OK;

  // IAT
  float rawIAT = readIAT();
  sensors.iat.updateStats(lowPassFilter(rawIAT, sensors.iat.value, 0.2f));
  sensors.iat.status = STATUS_OK;

  // Analisis lambat
  tempStatus = analyzeTemperature(sensors.engTemp.value);
  battStatus = analyzeBattery(sensors.battery.value);
}

void updateGraphs() {
  graphRPM[graphIdx]  = sensors.rpm.value;
  graphAFR[graphIdx]  = sensors.afr.value;
  graphTemp[graphIdx] = sensors.engTemp.value;
  graphBatt[graphIdx] = sensors.battery.value;
  graphIdx = (graphIdx + 1) % GRAPH_POINTS;

  // Shift array agar tampil dari kiri ke kanan
  // (Alternatif: gunakan circular buffer dengan offset render - lebih efisien)
  // Untuk kesederhanaan, rotate array
  float tmpR[GRAPH_POINTS], tmpA[GRAPH_POINTS], tmpT[GRAPH_POINTS], tmpB[GRAPH_POINTS];
  for (int i = 0; i < GRAPH_POINTS; i++) {
    int idx = (graphIdx + i) % GRAPH_POINTS;
    tmpR[i] = graphRPM[idx];
    tmpA[i] = graphAFR[idx];
    tmpT[i] = graphTemp[idx];
    tmpB[i] = graphBatt[idx];
  }
  memcpy(graphRPM,  tmpR, sizeof(graphRPM));
  memcpy(graphAFR,  tmpA, sizeof(graphAFR));
  memcpy(graphTemp, tmpT, sizeof(graphTemp));
  memcpy(graphBatt, tmpB, sizeof(graphBatt));
  graphIdx = GRAPH_POINTS - 1;
}

void updateEngineHealth() {
  engineHealth = calculateEngineHealth();
  fuelConsumptionL100 = calculateFuelConsumption();
  if (engineHealth >= 80)      healthStatus = HEALTH_EXCELLENT;
  else if (engineHealth >= 60) healthStatus = HEALTH_NORMAL;
  else if (engineHealth >= 40) healthStatus = HEALTH_WARNING;
  else                         healthStatus = HEALTH_CRITICAL;
}

// ============================================================
// MODE SWITCH
// ============================================================
void checkModeButton() {
  bool btnState = digitalRead(PIN_MODE_BTN);
  if (prevModBtn == HIGH && btnState == LOW) {  // falling edge
    uint32_t now = millis();
    if (now - lastModeBtn > 300) {  // debounce 300ms
      currentMode = (DisplayMode)((currentMode + 1) % MODE_COUNT);
      needFullRedraw = true;
      lastModeBtn = now;
    }
  }
  prevModBtn = btnState;
}

// ============================================================
// DEBUG SERIAL
// ============================================================
void debugSerial() {
  if (!SERIAL_DEBUG) return;
  Serial.printf("[DASH] t=%.1fs | SPD=%.1f | RPM=%.0f | AFR=%.2f | TMP=%.1f | BAT=%.2fV | FUL=%.0f%% | HLTH=%.0f%%\n",
    simTime,
    sensors.speed.value,
    sensors.rpm.value,
    sensors.afr.value,
    sensors.engTemp.value,
    sensors.battery.value,
    sensors.fuelLevel.value,
    engineHealth);
}

// ============================================================
// PLACEHOLDER - FUTURE ECU INTEGRATION
// ============================================================
/**
 * Placeholder: Integrasi CAN Bus (ELM327 / MCP2515)
 * - Pasang MCP2515 CAN module ke SPI ESP32
 * - Library: mcp_can atau ESP32-CAN
 * - Contoh read RPM dari CAN: PID 0x010C (RPM) via OBD-II
 *
 * void readFromCAN() {
 *   if (CAN.parseFrame(frame)) {
 *     if (frame.id == 0x7E8 && frame.data[2] == 0x0C) {
 *       float rpm = ((frame.data[3] * 256) + frame.data[4]) / 4.0f;
 *       sensors.rpm.value = rpm;
 *     }
 *   }
 * }
 *
 * Placeholder: K-Line (ISO 9141 / KWP2000)
 * - Gunakan MAX232 atau L9637 transceiver
 * - UART ESP32 (Serial2) ke K-Line transceiver
 * - Protocol: KWP2000 slow init, fast init
 *
 * Placeholder: OBD-II via ELM327 UART
 * - ESP32 Serial2 -> ELM327 UART
 * - Kirim AT command: "01 0C\r" untuk RPM
 * - Parse response hex ke nilai numerik
 */

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== ESP32 MOTO ECU DASHBOARD BOOT ===");

  // Inisialisasi TFT
  tft.begin();
  tft.setRotation(1);  // landscape
  tft.fillScreen(CLR_BG);

  // Boot screen
  tft.setTextColor(CLR_ACCENT, CLR_BG);
  tft.setTextSize(3);
  tft.setCursor(200, 150);
  tft.print("MOTO ECU DASHBOARD");
  tft.setTextSize(1);
  tft.setTextColor(CLR_GRAY, CLR_BG);
  tft.setCursor(280, 200);
  tft.print("ESP32 / TFT 800x480");
  tft.setCursor(260, 216);
  if (SIMULATION_MODE) tft.print("[SIMULATION MODE AKTIF]");
  else                 tft.print("[SENSOR MODE AKTIF]");

  // Init pins
  pinMode(PIN_MODE_BTN, INPUT_PULLUP);
  pinMode(PIN_FUEL_PUMP, INPUT);
  pinMode(PIN_FAN, INPUT);

  // ADC setup
  analogReadResolution(12);   // 12-bit ADC ESP32
  analogSetAttenuation(ADC_11db);  // 0-3.3V range

  // Interrupt RPM dan Speed
  if (!SIMULATION_MODE) {
    attachInterrupt(digitalPinToInterrupt(PIN_RPM),   ISR_RPM,   FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_SPEED), ISR_Speed, FALLING);
  }

  // Init graph buffers
  memset(graphRPM,  0, sizeof(graphRPM));
  memset(graphAFR,  0, sizeof(graphAFR));
  memset(graphTemp, 0, sizeof(graphTemp));
  memset(graphBatt, 0, sizeof(graphBatt));

  // Splash delay via millis (non-blocking)
  uint32_t splashEnd = millis() + 2000;
  while (millis() < splashEnd) { /* non-blocking wait */ }

  needFullRedraw = true;
  Serial.println("=== BOOT SELESAI - MEMASUKI LOOP UTAMA ===");
}

// ============================================================
// LOOP UTAMA - FULLY NON-BLOCKING
// ============================================================
void loop() {
  uint32_t now = millis();

  updateSimulationTime();
  checkModeButton();

  // Update sensor cepat (50ms)
  if (now - lastFastSensor >= INTERVAL_SENSOR_FAST) {
    lastFastSensor = now;
    updateFastSensors();
  }

  // Update sensor lambat (500ms)
  if (now - lastSlowSensor >= INTERVAL_SENSOR_SLOW) {
    lastSlowSensor = now;
    updateSlowSensors();
  }

  // Update grafik (200ms)
  if (now - lastGraph >= INTERVAL_GRAPH) {
    lastGraph = now;
    updateGraphs();
  }

  // Hitung engine health (2000ms)
  if (now - lastHealth >= INTERVAL_HEALTH) {
    lastHealth = now;
    updateEngineHealth();
  }

  // Refresh display (100ms)
  if (now - lastDisplay >= INTERVAL_DISPLAY) {
    lastDisplay = now;
    switch (currentMode) {
      case MODE_DASHBOARD:     drawDashboardMode();     break;
      case MODE_ECU_MAPPING:   drawECUMappingMode();    break;
      case MODE_DIAGNOSTIC:    drawDiagnosticMode();    break;
      case MODE_SENSOR_MONITOR: drawSensorMonitorMode(); break;
    }
  }

  // Debug serial (1000ms)
  if (now - lastDebug >= INTERVAL_DEBUG) {
    lastDebug = now;
    debugSerial();
  }
}

// ============================================================
// CATATAN WIRING LENGKAP:
// ============================================================
//
// [RPM SENSOR - PULSER/COIL]
// Coil pulser -> optocoupler (PC817) -> GPIO34 (INPUT dengan pullup)
// Tegangan coil bisa >12V, WAJIB optocoupler!
// Rangkaian: sinyal coil -> dioda 1N4007 -> optocoupler input
//            optocoupler output -> resistor 10kΩ pullup 3.3V -> GPIO34
//
// [SPEED SENSOR]
// Hall effect sensor roda -> GPIO35 (sama seperti RPM, via optocoupler jika perlu)
//
// [AFR / WIDEBAND O2]
// Wideband controller output 0-5V -> voltage divider -> GPIO36
// Voltage divider: Vout = Vin * R2/(R1+R2)
// R1=10kΩ, R2=6.8kΩ -> Vout = Vin * 0.405 (5V -> 2.025V, aman untuk 3.3V ADC)
//
// [ENGINE TEMP - NTC]
// Sensor NTC -> voltage divider dengan R_fixed -> GPIO39
// R_fixed pilih sesuai NTC (biasanya 2.2kΩ atau 4.7kΩ)
// 3.3V -> R_fixed -> NODE -> NTC -> GND
// NODE -> GPIO39
//
// [BATTERY VOLTAGE]
// 12V aki -> R1(10kΩ) -> NODE -> R2(3.3kΩ) -> GND
// NODE -> GPIO32
// Tambahkan zener 3.3V antara NODE dan GND sebagai proteksi
//
// [FUEL LEVEL]
// Sensor pelampung -> voltage divider -> GPIO33
// Ukur ADC saat kosong dan penuh untuk kalibrasi
//
// [TPS]
// TPS 0-5V -> voltage divider (sama seperti AFR) -> GPIO25
//
// [MAP SENSOR]
// MAP sensor 0-5V -> voltage divider -> GPIO26
//
// [IAT]
// NTC di intake -> voltage divider -> GPIO27
//
// [MODE BUTTON]
// Tombol push -> GND, dengan INPUT_PULLUP ke GPIO0
//
// [TFT SPI - sesuaikan di User_Setup.h TFT_eSPI]
// TFT_MOSI  -> GPIO23
// TFT_SCLK  -> GPIO18
// TFT_CS    -> GPIO15
// TFT_DC    -> GPIO2
// TFT_RST   -> GPIO4
// TFT_BL    -> 3.3V atau GPIO via transistor untuk brightness control
//
// ============================================================
