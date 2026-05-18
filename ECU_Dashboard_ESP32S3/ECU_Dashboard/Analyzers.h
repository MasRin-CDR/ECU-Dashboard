#pragma once
/**
 * ============================================================
 * AFRAnalyzer.h / FuelAnalyzer.h / EngineHealthAnalyzer.h
 * Sistem analisa AFR, konsumsi BBM, dan kesehatan mesin
 * ============================================================
 */

#include <Arduino.h>
#include "ECUManager.h"

// ─── AFR Constants ────────────────────────────────────────────
#define STOICH_AFR          14.7f    // Stoichiometric AFR bensin
#define AFR_LEAN_THRESHOLD  15.5f   // Di atas ini: Lean
#define AFR_RICH_THRESHOLD  13.5f   // Di bawah ini: Rich
#define LAMBDA_MULTIPLIER   14.7f   // Lambda = AFR / 14.7

// ─── AFR Status ───────────────────────────────────────────────
enum class AFRStatus {
    LEAN,
    NORMAL,
    RICH,
    UNKNOWN
};

// ─── AFRAnalyzer Class ────────────────────────────────────────
class AFRAnalyzer {
public:
    AFRAnalyzer();
    void     update(const ECUData& data);
    float    getAFR()     const { return _afr; }
    float    getLambda()  const { return _lambda; }
    AFRStatus getStatus() const { return _status; }
    String   getStatusString() const;
    float    getStability()    const { return _stability; }  // 0-100%

private:
    float    _afr;
    float    _lambda;
    AFRStatus _status;
    float    _stability;

    // Moving average untuk stabilitas AFR
    static const int SAMPLE_COUNT = 20;
    float    _samples[SAMPLE_COUNT];
    uint8_t  _sampleIdx;
    bool     _samplesReady;

    float calculateAFRFromO2(float o2Voltage);
    float calculateAFRFromMAP(float map, float rpm, float tps);
    float calculateStability();
};

// ─── FuelAnalyzer Class ───────────────────────────────────────
class FuelAnalyzer {
public:
    FuelAnalyzer();
    void  update(const ECUData& data);

    float getInstantConsumption() const { return _instantKmL; }
    float getAverageConsumption() const { return _avgKmL; }
    float getEstimatedRange()     const { return _estimatedRange; }

private:
    float    _instantKmL;     // Konsumsi sesaat (km/L)
    float    _avgKmL;         // Rata-rata konsumsi
    float    _estimatedRange; // Jarak tempuh estimasi (km)
    float    _totalFuelMl;    // Total BBM terpakai (mL)
    float    _totalDistKm;    // Total jarak (km)

    // Moving average
    static const int AVG_SAMPLES = 60;
    float    _samples[AVG_SAMPLES];
    uint8_t  _sampleIdx;

    float estimateInjectorPulse(const ECUData& data);
    float calcInstantConsumption(const ECUData& data);
};

// ─── EngineHealthAnalyzer Class ───────────────────────────────
enum class EngineStatus {
    EXCELLENT = 0,  // 80-100%
    GOOD      = 1,  // 60-79%
    WARNING   = 2,  // 40-59%
    CRITICAL  = 3   // 0-39%
};

class EngineHealthAnalyzer {
public:
    EngineHealthAnalyzer();
    void update(const ECUData& data, float afrStability);

    uint8_t      getHealthPercent() const { return _healthPercent; }
    EngineStatus getStatus()        const { return _status; }
    String       getStatusString()  const;
    uint32_t     getStatusColor()   const; // TFT color

private:
    uint8_t      _healthPercent;
    EngineStatus _status;

    // Scoring components
    uint8_t scoreAFRStability(float stability);
    uint8_t scoreIdleRPM(uint16_t rpm);
    uint8_t scoreBatteryVoltage(float voltage);
    uint8_t scoreCoolantTemp(int8_t temp);
    uint8_t scoreThrottleConsistency(float tps);
    uint8_t scoreDTC(uint8_t dtcCount);

    // History untuk idle RPM stability
    static const int IDLE_SAMPLES = 30;
    uint16_t _idleRPMSamples[IDLE_SAMPLES];
    uint8_t  _idleSampleIdx;
};
