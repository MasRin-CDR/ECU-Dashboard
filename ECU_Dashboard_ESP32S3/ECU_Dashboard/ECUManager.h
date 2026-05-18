#pragma once
/**
 * ============================================================
 * ECUManager.h
 * Manajer komunikasi ECU - request PID dan parse data
 * ============================================================
 * Mengatur seluruh siklus request/response ke ECU
 * Menyimpan data sensor realtime
 * ============================================================
 */

#include <Arduino.h>
#include "KLineProtocol.h"

// ─── OBD2 Service & PIDs ─────────────────────────────────────
#define OBD2_MODE_CURRENT       0x01  // Mode 1: Current data
#define OBD2_MODE_DTC           0x03  // Mode 3: Read DTCs
#define OBD2_MODE_CLEAR_DTC     0x04  // Mode 4: Clear DTCs

// Standard OBD2 PIDs
#define PID_ENGINE_SPEED        0x0C  // RPM
#define PID_VEHICLE_SPEED       0x0D  // Speed km/h
#define PID_THROTTLE_POS        0x11  // Throttle Position %
#define PID_ENGINE_COOLANT_TEMP 0x05  // Coolant Temperature
#define PID_O2_SENSOR_1         0x14  // O2 Sensor Voltage
#define PID_MAF_SENSOR          0x10  // Mass Air Flow
#define PID_MAP_SENSOR          0x0B  // Manifold Absolute Pressure
#define PID_INTAKE_AIR_TEMP     0x0F  // Intake Air Temperature
#define PID_FUEL_PRESSURE       0x0A  // Fuel Pressure
#define PID_SHORT_FUEL_TRIM     0x06  // Short-term Fuel Trim
#define PID_LONG_FUEL_TRIM      0x07  // Long-term Fuel Trim
#define PID_SUPPORTED_PIDS_1    0x00  // Supported PIDs 01-20
#define PID_BATTERY_VOLTAGE     0xFF  // Non-standard (custom sensor)

// ─── Battery Voltage (via ADC jika ECU tidak support) ────────
#define BATTERY_ADC_PIN         34
#define BATTERY_DIVIDER_RATIO   3.7f  // Voltage divider ratio
#define ADC_REF_VOLTAGE         3.3f
#define ADC_RESOLUTION          4095.0f

// ─── ECU Data Structure ───────────────────────────────────────
struct ECUData {
    // Engine
    uint16_t rpm;               // RPM saat ini
    uint8_t  speed;             // Kecepatan km/h
    int8_t   coolantTemp;       // Suhu coolant °C
    int8_t   intakeTemp;        // Suhu intake °C

    // Sensors
    float    throttlePos;       // Posisi throttle 0-100%
    float    o2Voltage;         // Tegangan O2 sensor (0-1.275V)
    float    mapPressure;       // MAP kPa
    float    mafFlow;           // MAF g/s
    float    fuelPressure;      // Tekanan bahan bakar kPa
    int8_t   shortFuelTrim;     // Short-term fuel trim %
    int8_t   longFuelTrim;      // Long-term fuel trim %

    // Battery
    float    batteryVoltage;    // Tegangan baterai (V)

    // Calculated
    float    afr;               // Air/Fuel Ratio
    float    fuelConsumption;   // Konsumsi BBM km/L
    float    avgFuelConsumption;// Rata-rata konsumsi BBM
    uint8_t  engineHealth;      // Kesehatan mesin 0-100%
    String   engineStatus;      // Excellent/Good/Warning/Critical

    // DTC
    uint8_t  dtcCount;          // Jumlah DTC

    // Timestamps
    uint32_t lastUpdateMs;      // Waktu update terakhir
    bool     valid;             // Data valid?
};

// ─── PID Request Queue ────────────────────────────────────────
struct PIDRequest {
    uint8_t service;
    uint8_t pid;
    uint32_t intervalMs;
    uint32_t lastRequestMs;
};

// ─── ECUManager Class ─────────────────────────────────────────
class ECUManager {
public:
    ECUManager();

    bool begin();
    void update();  // Panggil di setiap loop

    // Getters untuk data ECU
    const ECUData& getData() const { return _data; }
    bool isConnected() const;
    String getConnectionStatus() const;
    uint32_t getErrorCount() const;

    // Force reconnect
    void forceReconnect();

    // Request specific PID
    bool requestPID(uint8_t service, uint8_t pid);

    // Debug
    void printAllData();

private:
    KLineProtocol _kline;
    ECUData       _data;

    // PID request schedule
    PIDRequest _pidQueue[12];
    uint8_t    _pidQueueSize;
    uint8_t    _currentPIDIdx;

    // State tracking
    uint32_t _lastConnectionCheckMs;
    bool     _initialized;

    // Parser functions
    bool parsePIDResponse(const ECUFrame& frame);
    void parseRPM(uint8_t a, uint8_t b);
    void parseSpeed(uint8_t a);
    void parseCoolantTemp(uint8_t a);
    void parseThrottlePos(uint8_t a);
    void parseO2Sensor(uint8_t a, uint8_t b);
    void parseMAP(uint8_t a);
    void parseMAF(uint8_t a, uint8_t b);
    void parseIntakeTemp(uint8_t a);
    void parseFuelTrim(bool isShort, uint8_t a);

    // Battery voltage via ADC
    float readBatteryVoltage();

    // PID scheduling
    void setupPIDQueue();
    void processNextPID();

    // Connection management
    bool checkConnection();
};
