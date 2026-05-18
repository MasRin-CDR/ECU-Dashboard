/**
 * ============================================================
 * Analyzers.cpp
 * Implementasi AFRAnalyzer, FuelAnalyzer, EngineHealthAnalyzer
 * ============================================================
 */

#include "Analyzers.h"
#include <math.h>

// ============================================================
// AFRAnalyzer
// ============================================================

AFRAnalyzer::AFRAnalyzer()
    : _afr(14.7f), _lambda(1.0f), _status(AFRStatus::NORMAL),
      _stability(100.0f), _sampleIdx(0), _samplesReady(false)
{
    memset(_samples, 0, sizeof(_samples));
}

void AFRAnalyzer::update(const ECUData& data) {
    // Hitung AFR menggunakan multiple sensor
    float afrFromO2  = calculateAFRFromO2(data.o2Voltage);
    float afrFromMAP = calculateAFRFromMAP(data.mapPressure, data.rpm, data.throttlePos);

    // Blend berdasarkan sensor availability
    if (data.o2Voltage > 0.01f) {
        // O2 sensor lebih akurat jika available
        _afr = afrFromO2 * 0.7f + afrFromMAP * 0.3f;
    } else {
        // Fallback ke estimasi berbasis MAP/RPM/TPS
        _afr = afrFromMAP;
    }

    // Koreksi berdasarkan fuel trim
    float trimCorrection = 1.0f + ((float)(data.shortFuelTrim + data.longFuelTrim) / 200.0f);
    _afr *= trimCorrection;

    // Clamp ke range wajar
    _afr = constrain(_afr, 8.0f, 22.0f);

    // Hitung lambda
    _lambda = _afr / LAMBDA_MULTIPLIER;

    // Tentukan status
    if (_afr > AFR_LEAN_THRESHOLD) {
        _status = AFRStatus::LEAN;
    } else if (_afr < AFR_RICH_THRESHOLD) {
        _status = AFRStatus::RICH;
    } else {
        _status = AFRStatus::NORMAL;
    }

    // Update moving average untuk stabilitas
    _samples[_sampleIdx] = _afr;
    _sampleIdx = (_sampleIdx + 1) % SAMPLE_COUNT;
    if (_sampleIdx == 0) _samplesReady = true;

    _stability = calculateStability();

    Serial.printf("[AFR] AFR:%.2f Lambda:%.3f Status:%s Stability:%.0f%%\n",
                  _afr, _lambda, getStatusString().c_str(), _stability);
}

// ─── Hitung AFR dari O2 sensor voltage ───────────────────────
/**
 * Model Nerst equation approximation:
 * < 0.45V = Lean (excess oxygen)
 * > 0.45V = Rich (insufficient oxygen)
 * Interpolasi non-linear untuk akurasi lebih baik
 */
float AFRAnalyzer::calculateAFRFromO2(float o2Voltage) {
    if (o2Voltage < 0.01f) return STOICH_AFR; // No sensor

    // Narrowband O2 sensor curve approximation
    if (o2Voltage < 0.1f)  return 17.0f;
    if (o2Voltage < 0.2f)  return 16.0f + (0.2f - o2Voltage) / 0.1f;
    if (o2Voltage < 0.35f) return 15.5f + (0.35f - o2Voltage) / 0.15f * 0.5f;
    if (o2Voltage < 0.45f) return 14.7f + (0.45f - o2Voltage) / 0.1f * 0.8f;
    if (o2Voltage < 0.55f) return 14.7f - (o2Voltage - 0.45f) / 0.1f * 0.8f;
    if (o2Voltage < 0.7f)  return 13.8f - (o2Voltage - 0.55f) / 0.15f * 0.5f;
    if (o2Voltage < 0.9f)  return 13.0f - (o2Voltage - 0.7f) / 0.2f;
    return 12.0f;
}

// ─── Estimasi AFR dari MAP/RPM/TPS (Speed Density method) ────
/**
 * Speed Density AFR estimation
 * VE (Volumetric Efficiency) model sederhana
 */
float AFRAnalyzer::calculateAFRFromMAP(float mapKpa, float rpm, float tps) {
    if (rpm < 200) return STOICH_AFR;

    // Engine displacement (cc) - sesuaikan dengan motor target
    const float DISPLACEMENT_CC = 150.0f;
    const float IDEAL_GAS_CONSTANT = 0.287f; // kJ/(kg·K)
    const float INTAKE_TEMP_K = 300.0f;       // ~27°C asumsi

    // Massa udara per siklus (estimasi)
    float airMassPerCycle = (mapKpa * 1000.0f * DISPLACEMENT_CC * 1e-6f) /
                            (IDEAL_GAS_CONSTANT * 1000.0f * INTAKE_TEMP_K);

    // Estimasi injeksi BBM (mg per siklus) berdasarkan TPS dan RPM
    float fuelMgPerCycle = 5.0f + (tps / 100.0f) * 30.0f + (rpm / 10000.0f) * 10.0f;

    if (fuelMgPerCycle < 0.1f) return 20.0f; // Cutoff

    float afr = (airMassPerCycle * 1000.0f) / fuelMgPerCycle;
    return constrain(afr, 9.0f, 20.0f);
}

// ─── Hitung stabilitas AFR ────────────────────────────────────
float AFRAnalyzer::calculateStability() {
    if (!_samplesReady) return 80.0f;

    float sum = 0, sumSq = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        sum   += _samples[i];
        sumSq += _samples[i] * _samples[i];
    }
    float mean = sum / SAMPLE_COUNT;
    float variance = (sumSq / SAMPLE_COUNT) - (mean * mean);
    float stdDev = sqrt(variance);

    // StdDev 0 = 100%, StdDev >= 2.0 = 0%
    float stability = max(0.0f, 100.0f - (stdDev * 50.0f));
    return stability;
}

String AFRAnalyzer::getStatusString() const {
    switch (_status) {
        case AFRStatus::LEAN:    return "LEAN";
        case AFRStatus::NORMAL:  return "NORMAL";
        case AFRStatus::RICH:    return "RICH";
        default:                 return "UNKNOWN";
    }
}

// ============================================================
// FuelAnalyzer
// ============================================================

FuelAnalyzer::FuelAnalyzer()
    : _instantKmL(0), _avgKmL(0), _estimatedRange(0),
      _totalFuelMl(0), _totalDistKm(0), _sampleIdx(0)
{
    memset(_samples, 0, sizeof(_samples));
}

void FuelAnalyzer::update(const ECUData& data) {
    // Hitung konsumsi sesaat
    _instantKmL = calcInstantConsumption(data);

    // Update moving average
    _samples[_sampleIdx] = _instantKmL;
    _sampleIdx = (_sampleIdx + 1) % AVG_SAMPLES;

    float sum = 0;
    int validSamples = 0;
    for (int i = 0; i < AVG_SAMPLES; i++) {
        if (_samples[i] > 0.1f) {
            sum += _samples[i];
            validSamples++;
        }
    }
    _avgKmL = (validSamples > 0) ? (sum / validSamples) : 0;

    // Estimasi range (asumsi 3.5L tangki)
    const float TANK_LITERS = 3.5f;
    _estimatedRange = (_avgKmL > 0) ? (_avgKmL * TANK_LITERS) : 0;

    Serial.printf("[Fuel] Instant:%.1f km/L Avg:%.1f km/L Range:%.0f km\n",
                  _instantKmL, _avgKmL, _estimatedRange);
}

// ─── Estimasi injector pulse width (ms) ──────────────────────
float FuelAnalyzer::estimateInjectorPulse(const ECUData& data) {
    if (data.rpm < 200) return 0.0f;

    // Base pulse dari MAP (load)
    float basePulse = (data.mapPressure / 101.3f) * 10.0f; // ms at sea level MAP

    // Koreksi TPS
    float tpsCorrection = 1.0f + (data.throttlePos / 100.0f) * 0.5f;

    // Koreksi RPM (lebih banyak injeksi di RPM tinggi)
    float rpmCorrection = data.rpm / 3000.0f;

    float pulseMs = basePulse * tpsCorrection * rpmCorrection;
    return constrain(pulseMs, 0.5f, 20.0f);
}

// ─── Hitung konsumsi sesaat (km/L) ───────────────────────────
/**
 * Formula:
 * Fuel flow (mL/min) = injector_flow_rate * pulse_width * rpm / 2 / 60
 * Konsumsi (km/L) = speed_km_h / (fuel_flow_L_h)
 */
float FuelAnalyzer::calcInstantConsumption(const ECUData& data) {
    if (data.rpm < 500 || data.speed < 2) {
        return 0.0f; // Idle atau diam, tidak hitung konsumsi
    }

    // Asumsi injector: 125cc/min pada 3 bar
    const float INJECTOR_CC_PER_MIN = 125.0f;

    float pulseDuty = estimateInjectorPulse(data) / (60000.0f / data.rpm);
    pulseDuty = constrain(pulseDuty, 0.01f, 0.95f);

    float fuelFlowCcMin = INJECTOR_CC_PER_MIN * pulseDuty;
    float fuelFlowLH    = fuelFlowCcMin * 60.0f / 1000.0f;

    if (fuelFlowLH < 0.01f) return 99.9f; // Sangat ekonomis

    float kmL = (float)data.speed / fuelFlowLH;
    return constrain(kmL, 0.1f, 99.9f);
}

// ============================================================
// EngineHealthAnalyzer
// ============================================================

EngineHealthAnalyzer::EngineHealthAnalyzer()
    : _healthPercent(100), _status(EngineStatus::EXCELLENT),
      _idleSampleIdx(0)
{
    memset(_idleRPMSamples, 0, sizeof(_idleRPMSamples));
}

void EngineHealthAnalyzer::update(const ECUData& data, float afrStability) {
    // Skor setiap komponen (0-100)
    uint8_t sAFR      = scoreAFRStability(afrStability);
    uint8_t sIdle     = scoreIdleRPM(data.rpm);
    uint8_t sBattery  = scoreBatteryVoltage(data.batteryVoltage);
    uint8_t sTemp     = scoreCoolantTemp(data.coolantTemp);
    uint8_t sTPS      = scoreThrottleConsistency(data.throttlePos);
    uint8_t sDTC      = scoreDTC(data.dtcCount);

    // Weighted average
    // AFR 25%, Battery 20%, Temp 20%, Idle 15%, TPS 10%, DTC 10%
    uint16_t total = (sAFR * 25) + (sBattery * 20) + (sTemp * 20) +
                     (sIdle * 15) + (sTPS * 10)     + (sDTC * 10);
    _healthPercent = total / 100;
    _healthPercent = constrain(_healthPercent, 0, 100);

    // Tentukan status
    if (_healthPercent >= 80)      _status = EngineStatus::EXCELLENT;
    else if (_healthPercent >= 60) _status = EngineStatus::GOOD;
    else if (_healthPercent >= 40) _status = EngineStatus::WARNING;
    else                           _status = EngineStatus::CRITICAL;

    Serial.printf("[Health] %d%% (%s) | AFR:%d Bat:%d Temp:%d Idle:%d DTC:%d\n",
                  _healthPercent, getStatusString().c_str(),
                  sAFR, sBattery, sTemp, sIdle, sDTC);
}

uint8_t EngineHealthAnalyzer::scoreAFRStability(float stability) {
    return (uint8_t)constrain(stability, 0.0f, 100.0f);
}

uint8_t EngineHealthAnalyzer::scoreIdleRPM(uint16_t rpm) {
    if (rpm < 300 || rpm > 12000) return 20; // Tidak normal
    if (rpm < 800) return 50;                // Idle terlalu rendah

    // Rekam sample di idle (RPM 700-1200)
    if (rpm >= 700 && rpm <= 1200) {
        _idleRPMSamples[_idleSampleIdx] = rpm;
        _idleSampleIdx = (_idleSampleIdx + 1) % IDLE_SAMPLES;

        // Hitung variance idle RPM
        float sum = 0;
        for (int i = 0; i < IDLE_SAMPLES; i++) sum += _idleRPMSamples[i];
        float mean = sum / IDLE_SAMPLES;

        float variance = 0;
        for (int i = 0; i < IDLE_SAMPLES; i++) {
            float diff = _idleRPMSamples[i] - mean;
            variance += diff * diff;
        }
        variance /= IDLE_SAMPLES;

        // Variance < 1000 = 100%, variance > 50000 = 0%
        float score = max(0.0f, 100.0f - (sqrt(variance) / 2.0f));
        return (uint8_t)score;
    }

    return 85; // Non-idle, assume OK
}

uint8_t EngineHealthAnalyzer::scoreBatteryVoltage(float voltage) {
    if (voltage < 10.0f) return 0;   // Kritis
    if (voltage < 11.5f) return 20;  // Sangat lemah
    if (voltage < 12.0f) return 50;  // Lemah
    if (voltage < 12.5f) return 70;  // Normal bawah
    if (voltage < 14.5f) return 100; // Normal charging
    if (voltage < 15.0f) return 80;  // Sedikit tinggi
    return 30;                        // Overcharge
}

uint8_t EngineHealthAnalyzer::scoreCoolantTemp(int8_t temp) {
    if (temp < -20)  return 30;  // Terlalu dingin
    if (temp < 40)   return 60;  // Belum warm-up
    if (temp < 80)   return 85;  // Normal
    if (temp < 100)  return 100; // Optimal
    if (temp < 110)  return 70;  // Mulai panas
    if (temp < 120)  return 30;  // Overheat warning
    return 0;                    // Overheat kritis
}

uint8_t EngineHealthAnalyzer::scoreThrottleConsistency(float tps) {
    // Hanya check range yang valid
    if (tps >= 0 && tps <= 100) return 90;
    return 40;
}

uint8_t EngineHealthAnalyzer::scoreDTC(uint8_t dtcCount) {
    if (dtcCount == 0) return 100;
    if (dtcCount == 1) return 60;
    if (dtcCount == 2) return 40;
    return 0;
}

String EngineHealthAnalyzer::getStatusString() const {
    switch (_status) {
        case EngineStatus::EXCELLENT: return "Excellent";
        case EngineStatus::GOOD:      return "Good";
        case EngineStatus::WARNING:   return "Warning";
        case EngineStatus::CRITICAL:  return "Critical";
        default:                      return "Unknown";
    }
}

uint32_t EngineHealthAnalyzer::getStatusColor() const {
    switch (_status) {
        case EngineStatus::EXCELLENT: return 0x07E0; // TFT Green
        case EngineStatus::GOOD:      return 0x87E0; // TFT Light Green
        case EngineStatus::WARNING:   return 0xFFE0; // TFT Yellow
        case EngineStatus::CRITICAL:  return 0xF800; // TFT Red
        default:                      return 0xFFFF; // TFT White
    }
}
