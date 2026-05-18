#pragma once
// ╔══════════════════════════════════════════════════════════════╗
// ║          VARIO DASHBOARD — sensors.h                        ║
// ║  Baca sensor, filtering, kalibrasi, kalkulasi               ║
// ╚══════════════════════════════════════════════════════════════╝

#include <Arduino.h>
#include <math.h>
#include "config.h"

// ════════════════════════════════════════════
//  MOVING AVERAGE FILTER
//  Template generik — bisa dipakai untuk semua sensor
// ════════════════════════════════════════════
template<typename T, uint8_t N>
class MovingAvg {
  T      _buf[N] = {};
  uint8_t _idx   = 0;
  bool    _full  = false;
public:
  void add(T val) {
    _buf[_idx] = val;
    _idx = (_idx + 1) % N;
    if (_idx == 0) _full = true;
  }
  T get() const {
    uint8_t count = _full ? N : _idx;
    if (count == 0) return T(0);
    T sum = T(0);
    for (uint8_t i = 0; i < count; i++) sum += _buf[i];
    return sum / count;
  }
  void reset() { _idx = 0; _full = false; memset(_buf, 0, sizeof(_buf)); }
};

// ════════════════════════════════════════════
//  VARIABEL INTERRUPT (volatile)
// ════════════════════════════════════════════
volatile uint32_t rpmPulseCount    = 0;
volatile uint32_t rpmLastUs        = 0;
volatile uint32_t speedPulseCount  = 0;
volatile uint32_t speedLastUs      = 0;

// ISR RPM — dipanggil tiap ada pulsa dari pickup coil
void IRAM_ATTR isr_RPM() {
  rpmPulseCount++;
  rpmLastUs = micros();
}

// ISR Speed — dipanggil tiap ada pulsa dari sensor Hall roda
void IRAM_ATTR isr_Speed() {
  speedPulseCount++;
  speedLastUs = micros();
}

// ════════════════════════════════════════════
//  FILTER INSTANCES
// ════════════════════════════════════════════
MovingAvg<float,  SMOOTH_SAMPLES> avgAFR;
MovingAvg<float,  SMOOTH_SAMPLES> avgTemp;
MovingAvg<float,  SMOOTH_SAMPLES> avgBatt;
MovingAvg<float,  SMOOTH_SAMPLES> avgFuel;
MovingAvg<float,  4>              avgSpeed;
MovingAvg<uint16_t, 4>            avgRPM;

// ════════════════════════════════════════════
//  KALIBRASI
// ════════════════════════════════════════════
CalibData calib;

// ── Load kalibrasi default (bisa dikembangkan ke EEPROM/NVS) ──
void calibLoadDefault() {
  calib.speedFactor = 1.00f;
  calib.afrOffset   = 0.00f;
  calib.tempOffset  = 0.00f;
  calib.battFactor  = 1.00f;
  calib.fuelEmpty   = (float)FUEL_ADC_EMPTY;
  calib.fuelFull    = (float)FUEL_ADC_FULL;
}

// ════════════════════════════════════════════
//  INIT SENSOR
// ════════════════════════════════════════════
void sensorsInit() {
  // ADC resolution ESP32 = 12 bit (0–4095)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // range 0–3.3V

  // Setup interrupt RPM
  pinMode(PIN_RPM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), isr_RPM, FALLING);

  // Setup interrupt Speed
  pinMode(PIN_SPEED_HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SPEED_HALL), isr_Speed, FALLING);

  calibLoadDefault();

  if (DEBUG_MODE) Serial.println("[SENSOR] Init OK");
}

// ════════════════════════════════════════════
//  BACA RPM
//
//  Rumus:
//    RPM = (pulsa per interval / RPM_PULSES_PER_ROT) × (60000 / interval_ms)
//
//  Contoh:
//    10 pulsa dalam 100ms → (10/1) × (60000/100) = 6000 RPM
// ════════════════════════════════════════════
static uint32_t _rpmLastCount = 0;
static uint32_t _rpmLastCalcMs = 0;

uint16_t readRPM() {
  uint32_t now = millis();
  uint32_t elapsed = now - _rpmLastCalcMs;

  if (elapsed < 100) return avgRPM.get();  // terlalu cepat, pakai cache

  // Ambil jumlah pulsa dalam interval
  noInterrupts();
  uint32_t pulses = rpmPulseCount - _rpmLastCount;
  _rpmLastCount   = rpmPulseCount;
  uint32_t lastUs = rpmLastUs;
  interrupts();

  // Timeout — jika tidak ada pulsa lebih dari RPM_TIMEOUT_MS
  if ((now - (lastUs / 1000)) > RPM_TIMEOUT_MS) {
    avgRPM.reset();
    _rpmLastCalcMs = now;
    return 0;
  }

  // Hitung RPM
  float rpm = (float)pulses / (float)RPM_PULSES_PER_ROT
              * (60000.0f / (float)elapsed);

  _rpmLastCalcMs = now;

  // Clamp ke range masuk akal
  rpm = constrain(rpm, 0.0f, 14000.0f);

  avgRPM.add((uint16_t)rpm);
  return avgRPM.get();
}

// ════════════════════════════════════════════
//  BACA SPEED
//
//  Rumus:
//    v (m/s) = (pulsa × keliling_roda) / (interval_ms / 1000)
//    v (km/h) = v (m/s) × 3.6
//
//  Contoh:
//    Keliling 1.72m, 5 pulsa dalam 100ms:
//    v = (5 × 1.72) / 0.1 = 86 m/s → salah, harus per ROT
//    v = (5/PULSES_PER_ROT × 1.72) / 0.1 × 3.6 = 309.6 km/h → terlalu banyak pulsa
//    Artinya 5 pulsa = 5 rotasi → kecepatan tinggi (motor GP) — angka realistis 1-2 pulsa
// ════════════════════════════════════════════
static uint32_t _spdLastCount = 0;
static uint32_t _spdLastCalcMs = 0;

float readSpeed() {
  uint32_t now = millis();
  uint32_t elapsed = now - _spdLastCalcMs;

  if (elapsed < 100) return avgSpeed.get();

  noInterrupts();
  uint32_t pulses = speedPulseCount - _spdLastCount;
  _spdLastCount   = speedPulseCount;
  uint32_t lastUs = speedLastUs;
  interrupts();

  // Timeout
  if ((now - (lastUs / 1000)) > SPEED_TIMEOUT_MS) {
    avgSpeed.reset();
    _spdLastCalcMs = now;
    return 0.0f;
  }

  // Hitung kecepatan
  float rotations = (float)pulses / (float)SPEED_PULSES_PER_ROT;
  float distM     = rotations * WHEEL_CIRCUMFERENCE_M;
  float speedMs   = distM / ((float)elapsed / 1000.0f);
  float speedKmh  = speedMs * 3.6f * calib.speedFactor;

  _spdLastCalcMs = now;

  speedKmh = constrain(speedKmh, 0.0f, 220.0f);
  avgSpeed.add(speedKmh);
  return avgSpeed.get();
}

// Akumulasi jarak (dipanggil bersamaan readSpeed)
static float _distAccumM = 0.0f;
void updateDistance(SensorData& d) {
  noInterrupts();
  uint32_t pulses = speedPulseCount;
  interrupts();
  d.distanceM = (uint32_t)((float)pulses / SPEED_PULSES_PER_ROT
                            * WHEEL_CIRCUMFERENCE_M);
}

// ════════════════════════════════════════════
//  BACA AFR (Air/Fuel Ratio)
//
//  Wideband O2 sensor (misal Innovate LC-2):
//    Output: 0–5V linear → AFR 7.35–22.39
//    Via voltage divider (÷2.2) → ESP32 ADC 0–2.27V
//
//  Rumus:
//    AFR = AFR_MIN + (Vadc - Vmin) / (Vmax - Vmin) × (AFR_MAX - AFR_MIN)
//
//  Lambda = AFR / 14.7
// ════════════════════════════════════════════
float readAFR() {
  // Rata-rata 4 sampel ADC untuk kurangi noise
  uint32_t raw = 0;
  for (uint8_t i = 0; i < 4; i++) {
    raw += analogRead(PIN_AFR_ADC);
    delayMicroseconds(200);
  }
  raw /= 4;

  // Konversi ADC → volt (12-bit, ref 3.3V)
  float volt = (raw / 4095.0f) * 3.3f;

  // Clamp ke range valid
  volt = constrain(volt, AFR_VOLT_MIN, AFR_VOLT_MAX);

  // Map volt → AFR
  float afr = AFR_MIN + (volt - AFR_VOLT_MIN) / (AFR_VOLT_MAX - AFR_VOLT_MIN)
              * (AFR_MAX - AFR_MIN);

  afr += calib.afrOffset;
  afr  = constrain(afr, 9.0f, 22.0f);

  avgAFR.add(afr);
  return avgAFR.get();
}

// ════════════════════════════════════════════
//  BACA SUHU MESIN (NTC 10kΩ)
//
//  Rangkaian: 3.3V — R_fixed (10kΩ) — ADC — NTC — GND
//
//  Rumus Steinhart-Hart (disederhanakan):
//    R_ntc = R_fixed × Vadc / (Vref - Vadc)
//    1/T = 1/T0 + (1/Beta) × ln(R_ntc / R0)
//    T(Kelvin) → T(°C) = T - 273.15
//
//  Atau rumus Beta langsung:
//    T = Beta / ln(R_ntc × exp(Beta/T0) / R0) — lebih akurat
// ════════════════════════════════════════════
float readEngineTemp() {
  uint32_t raw = 0;
  for (uint8_t i = 0; i < 4; i++) {
    raw += analogRead(PIN_TEMP_ADC);
    delayMicroseconds(200);
  }
  raw /= 4;

  if (raw == 0 || raw >= 4095) {
    // Sensor putus atau short — fail-safe
    if (DEBUG_MODE) Serial.println("[TEMP] Sensor error");
    return -999.0f;
  }

  float volt  = (raw / 4095.0f) * TEMP_ADC_REF;
  float r_ntc = TEMP_R_FIXED * volt / (TEMP_ADC_REF - volt);

  // Steinhart-Hart Beta equation
  float tempK = TEMP_BETA / (log(r_ntc / TEMP_R25) + (TEMP_BETA / 298.15f));
  float tempC = tempK - 273.15f + calib.tempOffset;

  tempC = constrain(tempC, -40.0f, 150.0f);
  avgTemp.add(tempC);
  return avgTemp.get();
}

// ════════════════════════════════════════════
//  BACA VOLTASE BATERAI
//
//  Voltage divider: Vbatt → R1(100kΩ) → ADC → R2(22kΩ) → GND
//
//  Rumus:
//    Vadc = Vbatt × R2 / (R1 + R2)
//    Vbatt = Vadc × (R1 + R2) / R2
//
//  Contoh: Vbatt=12V → Vadc = 12 × 22/122 = 2.16V (aman untuk ESP32)
// ════════════════════════════════════════════
float readBattVoltage() {
  uint32_t raw = 0;
  for (uint8_t i = 0; i < 8; i++) {
    raw += analogRead(PIN_BATT_ADC);
    delayMicroseconds(200);
  }
  raw /= 8;

  float vadc  = (raw / 4095.0f) * BATT_ADC_REF;
  float vbatt = vadc * (BATT_R1 + BATT_R2) / BATT_R2 * calib.battFactor;

  vbatt = constrain(vbatt, 0.0f, 20.0f);
  avgBatt.add(vbatt);
  return avgBatt.get();
}

// ════════════════════════════════════════════
//  BACA PERSENTASE BENSIN
//
//  Sensor pelampung resistif:
//    Kosong ≈ 180Ω → ADC tinggi (pull-up menarik tinggi)
//    Penuh  ≈ 10Ω  → ADC rendah
//
//  Rangkaian: 3.3V — R_pull(220Ω) — ADC — sensor — GND
//
//  Rumus:
//    pct = (ADC_empty - ADC_raw) / (ADC_empty - ADC_full) × 100
// ════════════════════════════════════════════
float readFuelPercent() {
  uint32_t raw = 0;
  for (uint8_t i = 0; i < 4; i++) {
    raw += analogRead(PIN_FUEL_ADC);
    delayMicroseconds(200);
  }
  raw /= 4;

  float pct = (calib.fuelEmpty - (float)raw)
              / (calib.fuelEmpty - calib.fuelFull) * 100.0f;
  pct = constrain(pct, 0.0f, 100.0f);

  avgFuel.add(pct);
  return avgFuel.get();
}

// ════════════════════════════════════════════
//  HITUNG KONSUMSI BBM (km/L)
//
//  Estimasi berbasis jarak dan injector pulse width (dari ECU/K-Line)
//  atau pendekatan sederhana berbasis kecepatan + load.
//
//  Metode 1 (K-Line tersedia):
//    flow_rate (cc/s) = injector_duty × inj_static_flow_cc_min / 60
//    volume_L += flow_rate × interval_s / 1000
//    konsumsi = jarak_km / volume_L
//
//  Metode 2 (tanpa ECU — estimasi):
//    Asumsi konsumsi dasar + faktor RPM dan TPS
//    Base: Vario 125 ≈ 45 km/L di kondisi ideal
// ════════════════════════════════════════════
#define INJ_STATIC_FLOW_CC_MIN  120.0f   // cc/menit pada 3 bar (sesuaikan injector)
#define TANK_CAPACITY_L           5.5f   // kapasitas tangki Vario 125

static float  _fuelUsedL    = 0.0f;
static float  _distKm       = 0.0f;
static uint32_t _lastFuelMs = 0;

// Panggil tiap loop dengan injector duty % (0–100)
// Jika tidak ada data ECU, gunakan estimasiByLoad()
float estimateFuelConsumption(SensorData& d, float injDutyPct) {
  uint32_t now     = millis();
  float    elapsed = (now - _lastFuelMs) / 1000.0f;  // detik
  _lastFuelMs      = now;

  if (elapsed <= 0.0f || elapsed > 5.0f) return d.fuelConsKmL;

  // Hitung laju injeksi (cc/s)
  float flowCcPerSec = (injDutyPct / 100.0f)
                       * (INJ_STATIC_FLOW_CC_MIN / 60.0f);

  // Akumulasi bensin terpakai
  _fuelUsedL += flowCcPerSec * elapsed / 1000.0f;

  // Akumulasi jarak
  _distKm = d.distanceM / 1000.0f;

  // Hitung rata-rata konsumsi
  d.fuelUsedL = _fuelUsedL;
  if (_fuelUsedL > 0.001f && _distKm > 0.1f) {
    d.fuelConsKmL = _distKm / _fuelUsedL;
  } else {
    d.fuelConsKmL = 0.0f;
  }

  return d.fuelConsKmL;
}

// Estimasi konsumsi tanpa K-Line (mode fallback)
float estimateFuelByLoad(float speedKmh, uint16_t rpm, float throttlePct) {
  if (speedKmh < 2.0f) return 0.0f;  // idle / berhenti
  // Base consumption Vario 125 ≈ 45 km/L, turun dengan RPM & throttle
  float base = 45.0f;
  float rpmFactor  = 1.0f - constrain((rpm - 3000.0f) / 10000.0f, 0.0f, 0.5f);
  float tpsFactor  = 1.0f - constrain(throttlePct / 200.0f, 0.0f, 0.4f);
  return base * rpmFactor * tpsFactor;
}

// ════════════════════════════════════════════
//  HITUNG ENGINE HEALTH (0–100%)
//
//  Skor berdasarkan seberapa "normal" tiap parameter:
//    Suhu   : ideal 75–95°C   → skor linear di luar range
//    Volt   : ideal 12.0–14.5V
//    AFR    : ideal 14.2–15.1 (±0.5 dari stoich 14.7)
//    RPM    : idle normal 1200–1600, cruise 3000–6000
//    Konsumsi: ideal ≥ 35 km/L
// ════════════════════════════════════════════
static float scoreNormal(float val, float lo, float hi,
                          float warnLo, float warnHi) {
  if (val >= lo && val <= hi)    return 1.0f;
  if (val < warnLo || val > warnHi) return 0.0f;
  if (val < lo) return (val - warnLo) / (lo - warnLo);
  return (warnHi - val) / (warnHi - hi);
}

float calcEngineHealth(const SensorData& d) {
  float sTemp  = scoreNormal(d.engineTempC, 75.0f, 95.0f, 30.0f, 115.0f);
  float sVolt  = scoreNormal(d.battVolt,    12.0f, 14.5f, 10.0f,  16.0f);
  float sAFR   = scoreNormal(d.afr,         14.2f, 15.1f, 12.0f,  17.0f);
  float sRPM   = (d.rpm == 0) ? 0.5f :
                  scoreNormal((float)d.rpm, 1200.0f, 7000.0f, 500.0f, 10000.0f);
  float sCons  = (d.fuelConsKmL < 1.0f) ? 0.5f :
                  scoreNormal(d.fuelConsKmL, 35.0f, 55.0f, 15.0f, 70.0f);

  float health = (sTemp  * HEALTH_W_TEMP
                + sVolt  * HEALTH_W_VOLT
                + sAFR   * HEALTH_W_AFR
                + sRPM   * HEALTH_W_RPM
                + sCons  * HEALTH_W_FUEL_CONS) * 100.0f;

  return constrain(health, 0.0f, 100.0f);
}

// ════════════════════════════════════════════
//  SIMULASI SENSOR (SIMULATION_MODE = 1)
//  Untuk test dashboard tanpa hardware
// ════════════════════════════════════════════
#if SIMULATION_MODE
static float _simTime = 0;
void simulateSensors(SensorData& d) {
  _simTime += 0.15f;

  // Speed: 0–120 km/h, siklus sinusoidal
  d.speedKmh    = 60.0f + 55.0f * sin(_simTime * 0.3f);
  d.speedKmh    = max(0.0f, d.speedKmh);
  d.speedValid  = true;

  // RPM: mengikuti kecepatan
  d.rpm         = (uint16_t)(800 + d.speedKmh * 50 + 500 * sin(_simTime));
  d.rpmValid    = true;

  // AFR: sekitar stoich dengan sedikit variasi
  d.afr         = 14.7f + 1.5f * sin(_simTime * 1.7f);
  d.afrValid    = true;

  // Suhu: naik dari 25 ke 90°C
  d.engineTempC = 25.0f + 65.0f * (1.0f - exp(-_simTime * 0.05f))
                  + 2.0f * sin(_simTime * 0.8f);
  d.tempValid   = true;

  // Baterai: 12.6 + ripple charging
  d.battVolt    = 12.6f + 1.2f * (d.rpm > 1500 ? 1 : 0)
                  + 0.3f * sin(_simTime * 2.0f);
  d.battValid   = true;

  // Bensin: turun perlahan
  d.fuelPct     = max(0.0f, 85.0f - _simTime * 0.1f);
  d.fuelValid   = true;

  // Jarak
  d.distanceM   = (uint32_t)(_simTime * d.speedKmh * 100.0f / 3.6f);

  // Konsumsi BBM (estimasi)
  d.fuelConsKmL = estimateFuelByLoad(d.speedKmh, d.rpm, d.tps);

  // Health
  d.engineHealthPct = calcEngineHealth(d);

  d.ecuOnline = true;
}
#endif

// ════════════════════════════════════════════
//  UPDATE SEMUA SENSOR (dipanggil dari loop)
// ════════════════════════════════════════════
void sensorsUpdate(SensorData& d) {
#if SIMULATION_MODE
  simulateSensors(d);
  return;
#endif

  // Speed
  d.speedKmh  = readSpeed();
  d.speedValid = (d.speedKmh >= 0.0f);

  // RPM
  d.rpm       = readRPM();
  d.rpmValid  = true;

  // AFR
  d.afr       = readAFR();
  d.afrValid  = (d.afr > 9.0f && d.afr < 22.0f);

  // Suhu
  float t     = readEngineTemp();
  if (t > -998.0f) {
    d.engineTempC = t;
    d.tempValid   = true;
  } else {
    d.tempValid = false;
  }

  // Voltase
  d.battVolt  = readBattVoltage();
  d.battValid = (d.battVolt > 5.0f);

  // Bensin
  d.fuelPct   = readFuelPercent();
  d.fuelValid = true;

  // Jarak
  updateDistance(d);

  // Konsumsi (gunakan injDuty dari ECU jika tersedia, else estimasi)
  if (d.ecuOnline) {
    estimateFuelConsumption(d, d.tps);  // d.tps sebagai proxy injector duty
  } else {
    d.fuelConsKmL = estimateFuelByLoad(d.speedKmh, d.rpm, d.tps);
  }

  // Health
  d.engineHealthPct = calcEngineHealth(d);
}

// ── Debug print ──
void sensorsPrint(const SensorData& d) {
  if (!DEBUG_MODE) return;
  Serial.printf("[DATA] SPD:%.1f RPM:%d AFR:%.2f TEMP:%.1f BATT:%.2fV"
                " FUEL:%.0f%% CONS:%.1fkm/L HLTH:%.0f%%\n",
    d.speedKmh, d.rpm, d.afr, d.engineTempC,
    d.battVolt, d.fuelPct, d.fuelConsKmL, d.engineHealthPct);
}
