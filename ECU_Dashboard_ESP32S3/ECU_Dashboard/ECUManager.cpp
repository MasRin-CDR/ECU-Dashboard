/**
 * ============================================================
 * ECUManager.cpp
 * Implementasi manajer komunikasi ECU
 * ============================================================
 */

#include "ECUManager.h"

// ─── Constructor ─────────────────────────────────────────────
ECUManager::ECUManager()
    : _currentPIDIdx(0),
      _lastConnectionCheckMs(0),
      _initialized(false),
      _pidQueueSize(0)
{
    memset(&_data, 0, sizeof(ECUData));
    _data.engineStatus = "Unknown";
    _data.valid = false;
}

// ─── Begin ────────────────────────────────────────────────────
bool ECUManager::begin() {
    Serial.println("[ECUMgr] Initializing ECU Manager...");

    // Setup ADC untuk battery voltage
    analogReadResolution(12);
    pinMode(BATTERY_ADC_PIN, INPUT);

    // Setup PID request queue
    setupPIDQueue();

    // Inisialisasi K-Line hardware
    if (!_kline.begin()) {
        Serial.println("[ECUMgr][ERROR] K-Line hardware init failed");
        return false;
    }

    // Attempt handshake
    Serial.println("[ECUMgr] Attempting ECU handshake...");
    if (_kline.handshake()) {
        Serial.println("[ECUMgr] ECU connected!");
        Serial.printf("[ECUMgr] Protocol: %s\n", _kline.getProtocolString().c_str());
        _initialized = true;
        _data.valid   = true;
        return true;
    }

    Serial.println("[ECUMgr][WARN] ECU not connected - will retry");
    _initialized = false;
    return false;
}

// ─── Setup PID Queue ─────────────────────────────────────────
void ECUManager::setupPIDQueue() {
    // Definisikan PID yang akan di-request secara bergilir
    // Format: {service, pid, interval_ms, last_ms}
    _pidQueue[0]  = {OBD2_MODE_CURRENT, PID_ENGINE_SPEED,        50,  0}; // RPM: 20Hz
    _pidQueue[1]  = {OBD2_MODE_CURRENT, PID_VEHICLE_SPEED,       100, 0}; // Speed: 10Hz
    _pidQueue[2]  = {OBD2_MODE_CURRENT, PID_THROTTLE_POS,        50,  0}; // TPS: 20Hz
    _pidQueue[3]  = {OBD2_MODE_CURRENT, PID_O2_SENSOR_1,         100, 0}; // O2: 10Hz
    _pidQueue[4]  = {OBD2_MODE_CURRENT, PID_MAP_SENSOR,          100, 0}; // MAP: 10Hz
    _pidQueue[5]  = {OBD2_MODE_CURRENT, PID_ENGINE_COOLANT_TEMP, 500, 0}; // Coolant: 2Hz
    _pidQueue[6]  = {OBD2_MODE_CURRENT, PID_INTAKE_AIR_TEMP,     500, 0}; // IAT: 2Hz
    _pidQueue[7]  = {OBD2_MODE_CURRENT, PID_MAF_SENSOR,          100, 0}; // MAF: 10Hz
    _pidQueue[8]  = {OBD2_MODE_CURRENT, PID_SHORT_FUEL_TRIM,     200, 0}; // STFT: 5Hz
    _pidQueue[9]  = {OBD2_MODE_CURRENT, PID_LONG_FUEL_TRIM,      500, 0}; // LTFT: 2Hz
    _pidQueue[10] = {OBD2_MODE_CURRENT, PID_FUEL_PRESSURE,       200, 0}; // Fuel P: 5Hz
    _pidQueue[11] = {OBD2_MODE_DTC,     0x00,                   5000, 0}; // DTC: 0.2Hz

    _pidQueueSize  = 12;
    _currentPIDIdx = 0;
}

// ─── Main Update Loop ─────────────────────────────────────────
void ECUManager::update() {
    uint32_t now = millis();

    // Check koneksi setiap 1 detik
    if (now - _lastConnectionCheckMs > 1000) {
        _lastConnectionCheckMs = now;
        if (!checkConnection()) return;
    }

    // Update battery voltage via ADC (selalu bisa dibaca)
    _data.batteryVoltage = readBatteryVoltage();

    // Process PID requests jika connected
    if (_kline.isConnected()) {
        processNextPID();
    }

    // Timeout detection: jika tidak ada komunikasi > 3 detik
    if (_kline.isConnected() && (now - _kline.getLastCommsMs() > 3000)) {
        Serial.println("[ECUMgr][WARN] Communication timeout detected");
        _data.valid = false;
    }
}

// ─── Process Next PID ─────────────────────────────────────────
void ECUManager::processNextPID() {
    uint32_t now = millis();
    PIDRequest& req = _pidQueue[_currentPIDIdx];

    // Cek apakah sudah waktunya request PID ini
    if (now - req.lastRequestMs < req.intervalMs) {
        // Belum waktunya, coba PID berikutnya
        _currentPIDIdx = (_currentPIDIdx + 1) % _pidQueueSize;
        return;
    }

    req.lastRequestMs = now;

    // Kirim request
    if (_kline.sendRequest(req.service, req.pid)) {
        // Tunggu dan baca response
        ECUFrame response;
        if (_kline.receiveResponse(response, ECU_TIMEOUT_MS)) {
            if (response.valid) {
                parsePIDResponse(response);
                _data.lastUpdateMs = millis();
                _data.valid = true;
            }
        } else {
            Serial.printf("[ECUMgr][WARN] No response for PID 0x%02X\n", req.pid);
        }
    }

    // Advance ke PID berikutnya
    _currentPIDIdx = (_currentPIDIdx + 1) % _pidQueueSize;
}

// ─── Parse PID Response ───────────────────────────────────────
bool ECUManager::parsePIDResponse(const ECUFrame& frame) {
    if (frame.dataLen < 2) return false;

    // Byte 0 = Service response (mode + 0x40)
    // Byte 1 = PID
    uint8_t responsePID = frame.data[1];
    uint8_t a = (frame.dataLen > 2) ? frame.data[2] : 0;
    uint8_t b = (frame.dataLen > 3) ? frame.data[3] : 0;

    switch (responsePID) {
        case PID_ENGINE_SPEED:
            parseRPM(a, b);
            break;
        case PID_VEHICLE_SPEED:
            parseSpeed(a);
            break;
        case PID_THROTTLE_POS:
            parseThrottlePos(a);
            break;
        case PID_ENGINE_COOLANT_TEMP:
            parseCoolantTemp(a);
            break;
        case PID_O2_SENSOR_1:
            parseO2Sensor(a, b);
            break;
        case PID_MAP_SENSOR:
            parseMAP(a);
            break;
        case PID_MAF_SENSOR:
            parseMAF(a, b);
            break;
        case PID_INTAKE_AIR_TEMP:
            parseIntakeTemp(a);
            break;
        case PID_SHORT_FUEL_TRIM:
            parseFuelTrim(true, a);
            break;
        case PID_LONG_FUEL_TRIM:
            parseFuelTrim(false, a);
            break;
        default:
            Serial.printf("[ECUMgr] Unknown PID response: 0x%02X\n", responsePID);
            return false;
    }

    return true;
}

// ─── Individual PID Parsers ───────────────────────────────────

/**
 * RPM = ((A * 256) + B) / 4
 * Range: 0 - 16383.75 RPM
 */
void ECUManager::parseRPM(uint8_t a, uint8_t b) {
    _data.rpm = ((uint16_t)(a * 256) + b) / 4;
    Serial.printf("[ECUMgr] RPM: %d\n", _data.rpm);
}

/**
 * Speed = A (km/h)
 * Range: 0 - 255 km/h
 */
void ECUManager::parseSpeed(uint8_t a) {
    _data.speed = a;
    Serial.printf("[ECUMgr] Speed: %d km/h\n", _data.speed);
}

/**
 * Coolant Temp = A - 40 (°C)
 * Range: -40 to 215 °C
 */
void ECUManager::parseCoolantTemp(uint8_t a) {
    _data.coolantTemp = (int8_t)(a - 40);
    Serial.printf("[ECUMgr] Coolant: %d°C\n", _data.coolantTemp);
}

/**
 * Throttle = A * 100 / 255 (%)
 * Range: 0 - 100%
 */
void ECUManager::parseThrottlePos(uint8_t a) {
    _data.throttlePos = (float)a * 100.0f / 255.0f;
    Serial.printf("[ECUMgr] TPS: %.1f%%\n", _data.throttlePos);
}

/**
 * O2 Voltage = A * 0.005V (0 - 1.275V)
 * O2 Fuel Trim = (B - 128) * 100/128 (%)
 */
void ECUManager::parseO2Sensor(uint8_t a, uint8_t b) {
    _data.o2Voltage = (float)a * 0.005f;
    Serial.printf("[ECUMgr] O2: %.3fV\n", _data.o2Voltage);
}

/**
 * MAP = A (kPa)
 * Range: 0 - 255 kPa
 */
void ECUManager::parseMAP(uint8_t a) {
    _data.mapPressure = (float)a;
    Serial.printf("[ECUMgr] MAP: %.0f kPa\n", _data.mapPressure);
}

/**
 * MAF = ((A * 256) + B) / 100 (g/s)
 * Range: 0 - 655.35 g/s
 */
void ECUManager::parseMAF(uint8_t a, uint8_t b) {
    _data.mafFlow = ((float)(a * 256 + b)) / 100.0f;
    Serial.printf("[ECUMgr] MAF: %.2f g/s\n", _data.mafFlow);
}

/**
 * Intake Temp = A - 40 (°C)
 */
void ECUManager::parseIntakeTemp(uint8_t a) {
    _data.intakeTemp = (int8_t)(a - 40);
    Serial.printf("[ECUMgr] IAT: %d°C\n", _data.intakeTemp);
}

/**
 * Fuel Trim = (A - 128) * 100 / 128 (%)
 * Positive = lean correction, Negative = rich correction
 */
void ECUManager::parseFuelTrim(bool isShort, uint8_t a) {
    int8_t trim = (int8_t)(((int16_t)a - 128) * 100 / 128);
    if (isShort) {
        _data.shortFuelTrim = trim;
        Serial.printf("[ECUMgr] STFT: %d%%\n", trim);
    } else {
        _data.longFuelTrim = trim;
        Serial.printf("[ECUMgr] LTFT: %d%%\n", trim);
    }
}

// ─── Read Battery Voltage ─────────────────────────────────────
float ECUManager::readBatteryVoltage() {
    // Rata-rata 10 sample untuk stabilitas
    uint32_t adcSum = 0;
    for (int i = 0; i < 10; i++) {
        adcSum += analogRead(BATTERY_ADC_PIN);
        delayMicroseconds(100);
    }
    float adcAvg = adcSum / 10.0f;
    float voltage = (adcAvg / ADC_RESOLUTION) * ADC_REF_VOLTAGE * BATTERY_DIVIDER_RATIO;
    return voltage;
}

// ─── Check Connection ─────────────────────────────────────────
bool ECUManager::checkConnection() {
    if (_kline.isConnected()) return true;

    // Coba reconnect
    Serial.println("[ECUMgr] Not connected, attempting reconnect...");
    if (_kline.reconnect()) {
        Serial.println("[ECUMgr] Reconnected!");
        _data.valid = true;
        return true;
    }

    _data.valid = false;
    return false;
}

// ─── Is Connected ─────────────────────────────────────────────
bool ECUManager::isConnected() const {
    return _kline.isConnected();
}

// ─── Get Connection Status ────────────────────────────────────
String ECUManager::getConnectionStatus() const {
    if (!_kline.isConnected()) return "DISCONNECTED";
    return "CONNECTED [" + _kline.getProtocolString() + "]";
}

// ─── Get Error Count ──────────────────────────────────────────
uint32_t ECUManager::getErrorCount() const {
    return _kline.getErrorCount();
}

// ─── Force Reconnect ──────────────────────────────────────────
void ECUManager::forceReconnect() {
    _kline.resetState();
    _data.valid = false;
    Serial.println("[ECUMgr] Force reconnect initiated");
}

// ─── Print All Data (Debug) ───────────────────────────────────
void ECUManager::printAllData() {
    Serial.println("=== ECU DATA DUMP ===");
    Serial.printf("RPM:         %d\n",    _data.rpm);
    Serial.printf("Speed:       %d km/h\n", _data.speed);
    Serial.printf("Coolant:     %d°C\n",  _data.coolantTemp);
    Serial.printf("TPS:         %.1f%%\n", _data.throttlePos);
    Serial.printf("O2:          %.3fV\n", _data.o2Voltage);
    Serial.printf("MAP:         %.0f kPa\n", _data.mapPressure);
    Serial.printf("MAF:         %.2f g/s\n", _data.mafFlow);
    Serial.printf("Battery:     %.2fV\n", _data.batteryVoltage);
    Serial.printf("AFR:         %.2f\n",  _data.afr);
    Serial.printf("Fuel Cons:   %.2f km/L\n", _data.fuelConsumption);
    Serial.printf("Engine Health: %d%% (%s)\n", _data.engineHealth,
                  _data.engineStatus.c_str());
    Serial.printf("Valid: %s\n", _data.valid ? "YES" : "NO");
    Serial.println("=====================");
}
