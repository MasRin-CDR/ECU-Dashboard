/*
  ESP32 Motorcycle Digital Dashboard - Production Base

  Single-file Arduino sketch with:
  - Non-blocking sensor acquisition
  - Interrupt-driven RPM and wheel speed
  - ADC smoothing and moving-average filtering
  - Fuel consumption estimation
  - Engine health scoring
  - TFT_eSPI racing dashboard UI
  - K-Line / KWP2000 scaffold for future ECU integration

  Target display: 480x320 ILI9488 SPI
  Recommended hardware: ESP32 with PSRAM

  Wiring notes:
  - K-Line requires a proper transceiver (MC33290 / L9637D / equivalent)
  - Do not connect ESP32 GPIO directly to vehicle K-Line
  - Use fused power, reverse-polarity protection, and automotive-grade grounding
*/

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <math.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <esp_task_wdt.h>
#endif

// =====================================================================
// Secure configuration placeholder
// =====================================================================
struct SecureConfig {
  const char* wifiSsid;
  const char* wifiPassword;
  const char* mqttHost;
  uint16_t mqttPort;
  bool useProvisioning;

  SecureConfig()
    : wifiSsid(""),
      wifiPassword(""),
      mqttHost(""),
      mqttPort(0),
      useProvisioning(false) {}
};

static SecureConfig g_secureConfig;

// =====================================================================
// Compile-time configuration
// =====================================================================
namespace cfg {

constexpr bool DEBUG_MODE        = true;
constexpr bool SIMULATION_MODE   = false;   // set true to test without sensors
constexpr bool ECU_MANAGER_ENABLED = false; // set true to actively probe ECU K-Line

constexpr uint16_t TFT_WIDTH  = 480;
constexpr uint16_t TFT_HEIGHT = 320;

// Main timing
constexpr uint32_t FAST_UPDATE_MS   = 50;    // RPM, speed, AFR
constexpr uint32_t SLOW_UPDATE_MS   = 250;   // temp, battery, fuel
constexpr uint32_t UI_UPDATE_MS     = 33;    // ~30 FPS target
constexpr uint32_t DEBUG_PRINT_MS   = 1000;
constexpr uint32_t BOOT_ANIM_MS     = 900;
constexpr uint32_t FULL_REDRAW_MS   = 5000;
constexpr uint32_t ECU_EXTERNAL_TIMEOUT_MS = 2000;

// Sensor timeouts
constexpr uint32_t SPEED_TIMEOUT_MS = 900;
constexpr uint32_t RPM_TIMEOUT_MS   = 700;
constexpr uint32_t AFR_TIMEOUT_MS   = 1200;
constexpr uint32_t TEMP_TIMEOUT_MS  = 2000;
constexpr uint32_t BATT_TIMEOUT_MS  = 2000;
constexpr uint32_t FUEL_TIMEOUT_MS  = 3000;

// Debounce / minimum edge spacing for interrupts
constexpr uint32_t RPM_MIN_EDGE_US   = 650;
constexpr uint32_t SPEED_MIN_EDGE_US = 2500;
constexpr uint32_t RPM_TIMEOUT_US     = 650000;
constexpr uint32_t SPEED_TIMEOUT_US   = 900000;

// Pin map
constexpr uint8_t PIN_RPM          = 34; // input only
constexpr uint8_t PIN_SPEED        = 35; // input only
constexpr uint8_t PIN_AFR_ADC      = 36; // ADC1
constexpr uint8_t PIN_TEMP_ADC     = 39; // ADC1
constexpr uint8_t PIN_BATT_ADC     = 32; // ADC1
constexpr uint8_t PIN_FUEL_ADC     = 33; // ADC1
// NOTE: GPIO25/26 are ADC2. If Wi-Fi is enabled later, move these to an
// external ADC or another sensor interface because ADC2 is shared with Wi-Fi.
constexpr uint8_t PIN_TPS_ADC      = 25; // optional future
constexpr uint8_t PIN_MAP_ADC      = 26; // optional future
constexpr uint8_t PIN_FUEL_PUMP_IN = 21; // optional future, reserved
constexpr uint8_t PIN_FAN_IN       = 22; // optional future, reserved

constexpr uint8_t PIN_KLINE_RX     = 16;
constexpr uint8_t PIN_KLINE_TX     = 17;

// TFT_eSPI pin assignment is configured in the library User_Setup file.
// Keep these values aligned with your board wiring and TFT_eSPI setup.
constexpr uint8_t PIN_TFT_CS       = 5;
constexpr uint8_t PIN_TFT_DC       = 2;
constexpr uint8_t PIN_TFT_RST      = 4;
constexpr uint8_t PIN_TOUCH_CS     = 15;
constexpr uint8_t PIN_TOUCH_IRQ    = 27;

// ADC and calibration
constexpr float ADC_REF_VOLT       = 3.30f;
constexpr float ADC_MAX_RAW        = 4095.0f;
constexpr float BATT_R1_OHM        = 100000.0f;
constexpr float BATT_R2_OHM        = 22000.0f;
constexpr float BATT_DIVIDER_RATIO = (BATT_R1_OHM + BATT_R2_OHM) / BATT_R2_OHM; // 5.545x

// Wideband AFR scaling: sensor output is divided before reaching ADC
// Example divider: 68k / 47k -> sensorVoltage = adcVoltage * 2.447
constexpr float AFR_DIVIDER_RATIO  = 2.447f;
constexpr float AFR_SENSOR_V_MIN    = 0.50f;
constexpr float AFR_SENSOR_V_MAX    = 4.50f;
constexpr float AFR_MIN_VALUE       = 10.0f;
constexpr float AFR_MAX_VALUE       = 20.0f;

// NTC thermistor parameters
constexpr float TEMP_NTC_FIXED_OHM  = 10000.0f;
constexpr float TEMP_NTC_R25_OHM    = 10000.0f;
constexpr float TEMP_NTC_BETA       = 3950.0f;
constexpr float TEMP_OFFSET_C       = 0.0f;

// Fuel level calibration
constexpr float FUEL_ADC_EMPTY      = 3800.0f;
constexpr float FUEL_ADC_FULL       = 400.0f;

// Wheel / engine constants
constexpr float WHEEL_CIRCUMFERENCE_M = 1.720f;
constexpr float SPEED_PULSES_PER_REV  = 1.0f;
constexpr float RPM_PULSES_PER_REV    = 1.0f;
constexpr float SPEED_CAL_FACTOR      = 1.0f;
constexpr float RPM_CAL_FACTOR        = 1.0f;

// Fuel estimation
constexpr float INJECTOR_FLOW_CC_MIN  = 125.0f; // placeholder for 125cc injector
constexpr float TANK_CAPACITY_L      = 3.5f;

// UI layout
constexpr uint16_t ZONE_TOP_H    = 60;
constexpr uint16_t ZONE_MID_H    = 180;
constexpr uint16_t ZONE_BOT_H    = 80;

// Colors (RGB565)
constexpr uint16_t COL_BG          = 0x0810;
constexpr uint16_t COL_PANEL       = 0x1021;
constexpr uint16_t COL_PANEL_2     = 0x18A3;
constexpr uint16_t COL_ACCENT      = 0x07FF;
constexpr uint16_t COL_ACCENT2     = 0x051F;
constexpr uint16_t COL_GREEN       = 0x07E0;
constexpr uint16_t COL_GREEN_DIM   = 0x03A0;
constexpr uint16_t COL_YELLOW      = 0xFFE0;
constexpr uint16_t COL_ORANGE      = 0xFC20;
constexpr uint16_t COL_RED         = 0xF800;
constexpr uint16_t COL_WHITE       = 0xFFFF;
constexpr uint16_t COL_GRAY        = 0x7BEF;
constexpr uint16_t COL_DARK_GRAY   = 0x4208;
constexpr uint16_t COL_MUTED       = 0x6B4D;

// Health weights
constexpr float HEALTH_W_TEMP = 0.30f;
constexpr float HEALTH_W_AFR  = 0.25f;
constexpr float HEALTH_W_VOLT = 0.20f;
constexpr float HEALTH_W_RPM  = 0.15f;
constexpr float HEALTH_W_FUEL = 0.10f;

// ECU / K-Line
constexpr uint8_t OBD_MODE_CURRENT   = 0x01;
constexpr uint8_t PID_SUPPORTED_01   = 0x00;
constexpr uint8_t PID_ENGINE_RPM     = 0x0C;
constexpr uint8_t PID_SPEED          = 0x0D;
constexpr uint8_t PID_COOLANT_TEMP   = 0x05;
constexpr uint8_t PID_THROTTLE_POS   = 0x11;
constexpr uint8_t PID_O2_SENSOR_1    = 0x14;
constexpr uint8_t PID_MAP            = 0x0B;
constexpr uint8_t PID_IAT            = 0x0F;
constexpr uint8_t PID_BATTERY_VOLT   = 0x42;
constexpr uint8_t PID_STFT           = 0x06;
constexpr uint8_t PID_LTFT           = 0x07;

constexpr uint8_t ECU_ADDR_DEFAULT   = 0x6A;
constexpr uint8_t TESTER_ADDR_DEFAULT = 0xF1;
constexpr uint8_t KLINE_HEADER_DEFAULT = 0x68;
constexpr uint32_t KLINE_BAUD        = 10400;
constexpr uint32_t KLINE_PROBE_MS    = 3000;
constexpr uint32_t KLINE_RX_TIMEOUT_MS = 500;
constexpr uint32_t KLINE_RETRY_MS    = 3000;

constexpr uint32_t WDT_TIMEOUT_MS    = 8000;

} // namespace cfg

// =====================================================================
// Utility helpers
// =====================================================================
template <typename T>
static inline T clampValue(T v, T lo, T hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  if (fabsf(inMax - inMin) < 0.0001f) return outMin;
  float t = (x - inMin) / (inMax - inMin);
  return outMin + (t * (outMax - outMin));
}

static inline bool isFiniteNumber(float v) {
  return !isnan(v) && !isinf(v);
}

static inline const char* yesNo(bool v) {
  return v ? "ON" : "OFF";
}

static void formatHMS(char* out, size_t size, uint32_t seconds) {
  uint32_t h = seconds / 3600UL;
  uint32_t m = (seconds % 3600UL) / 60UL;
  uint32_t s = seconds % 60UL;
  snprintf(out, size, "%02lu:%02lu:%02lu",
           static_cast<unsigned long>(h),
           static_cast<unsigned long>(m),
           static_cast<unsigned long>(s));
}

static void formatMeters(char* out, size_t size, float meters) {
  if (meters >= 1000.0f) {
    snprintf(out, size, "%.1f km", meters / 1000.0f);
  } else {
    snprintf(out, size, "%.0f m", meters);
  }
}

// =====================================================================
// Generic windows
// =====================================================================
template <typename T, size_t N>
class MovingAverage {
public:
  MovingAverage() { reset(); }

  void reset() {
    _count = 0;
    _index = 0;
    _sum = T{};
    for (size_t i = 0; i < N; ++i) _values[i] = T{};
  }

  T add(T value) {
    if (_count < N) {
      _values[_index] = value;
      _sum += value;
      ++_count;
      _index = (_index + 1) % N;
      return average();
    }

    _sum -= _values[_index];
    _values[_index] = value;
    _sum += value;
    _index = (_index + 1) % N;
    return average();
  }

  T average() const {
    if (_count == 0) return T{};
    return _sum / static_cast<T>(_count);
  }

  size_t count() const { return _count; }

private:
  T _values[N];
  size_t _count;
  size_t _index;
  T _sum;
};

template <typename T, size_t N>
class HistoryWindow {
public:
  HistoryWindow() { reset(); }

  void reset() {
    _count = 0;
    _index = 0;
    for (size_t i = 0; i < N; ++i) _values[i] = T{};
  }

  void push(T value) {
    _values[_index] = value;
    _index = (_index + 1) % N;
    if (_count < N) ++_count;
  }

  size_t count() const { return _count; }

  float mean() const {
    if (_count == 0) return 0.0f;
    float sum = 0.0f;
    for (size_t i = 0; i < _count; ++i) sum += static_cast<float>(_values[i]);
    return sum / static_cast<float>(_count);
  }

  float stddev() const {
    if (_count < 2) return 0.0f;
    float m = mean();
    float acc = 0.0f;
    for (size_t i = 0; i < _count; ++i) {
      float d = static_cast<float>(_values[i]) - m;
      acc += d * d;
    }
    return sqrtf(acc / static_cast<float>(_count));
  }

private:
  T _values[N];
  size_t _count;
  size_t _index;
};

class SensorFilter {
public:
  SensorFilter() : _hasValue(false), _last(0.0f) {}

  void reset() {
    _avg.reset();
    _hasValue = false;
    _last = 0.0f;
  }

  float process(float raw, bool valid, float fallback, float maxDelta, bool& accepted) {
    if (!valid || !isFiniteNumber(raw)) {
      accepted = false;
      return _hasValue ? _last : fallback;
    }

    if (!_hasValue) {
      _avg.reset();
      _last = raw;
      _hasValue = true;
      accepted = true;
      return _avg.add(raw);
    }

    float delta = raw - _last;
    if (fabsf(delta) > maxDelta) {
      raw = _last + ((delta > 0.0f) ? maxDelta : -maxDelta);
    }

    _last = _avg.add(raw);
    accepted = true;
    return _last;
  }

  float last() const { return _last; }
  bool hasValue() const { return _hasValue; }

private:
  MovingAverage<float, 8> _avg;
  bool _hasValue;
  float _last;
};

class AnalogSampler {
public:
  static uint16_t averageRaw(uint8_t pin, uint8_t samples = 5, uint16_t settleUs = 50) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; ++i) {
      sum += analogRead(pin);
      delayMicroseconds(settleUs);
    }
    return static_cast<uint16_t>(sum / samples);
  }

  static float rawToVoltage(uint16_t raw) {
    return (static_cast<float>(raw) / cfg::ADC_MAX_RAW) * cfg::ADC_REF_VOLT;
  }
};

// =====================================================================
// Status enums and data structures
// =====================================================================
enum class SensorState : uint8_t {
  Ok = 0,
  Warning,
  Error,
  Offline
};

enum class WarningType : uint8_t {
  None = 0,
  BatteryLow,
  Overheat,
  AfrLean,
  AfrRich,
  EcuDisconnected,
  SensorFault
};

enum class KLineState : uint8_t {
  Disabled = 0,
  Idle,
  Probing,
  Connected,
  Requesting,
  WaitingResponse,
  Error,
  Reconnecting
};

enum class ProtocolType : uint8_t {
  Unknown = 0,
  ISO9141,
  KWP2000
};

struct TimedValue {
  float value = 0.0f;
  bool valid = false;
  uint32_t lastMs = 0;

  bool isFresh(uint32_t now, uint32_t timeoutMs) const {
    return valid && ((now - lastMs) <= timeoutMs);
  }
};

struct ChannelState {
  float value = 0.0f;
  float lastGood = 0.0f;
  bool valid = false;
  SensorState state = SensorState::Offline;
  uint32_t lastUpdateMs = 0;
  uint32_t lastGoodMs = 0;
  uint32_t timeoutMs = 1000;
  uint32_t errorCount = 0;
};

struct DashboardSnapshot {
  float speedKmh = 0.0f;
  uint16_t rpm = 0;
  float afr = 14.7f;
  float engineTempC = 25.0f;
  float batteryVolt = 12.6f;
  float fuelPercent = 100.0f;
  float fuelInstantKmL = 0.0f;
  float fuelAverageKmL = 0.0f;
  float fuelLPer100Km = 0.0f;
  float engineHealth = 100.0f;
  float throttlePct = 0.0f;
  float mapKpa = 101.3f;
  float injectorPulseMs = 0.0f;
  float distanceMeters = 0.0f;
  bool ecuEnabled = false;
  bool ecuOnline = false;
  bool simulationMode = false;

  SensorState speedState = SensorState::Offline;
  SensorState rpmState = SensorState::Offline;
  SensorState afrState = SensorState::Offline;
  SensorState tempState = SensorState::Offline;
  SensorState battState = SensorState::Offline;
  SensorState fuelState = SensorState::Offline;
  SensorState throttleState = SensorState::Offline;
  SensorState mapState = SensorState::Offline;
  SensorState injectorState = SensorState::Offline;

  uint32_t timestampMs = 0;
};

// =====================================================================
// SensorHub
// =====================================================================
class SensorHub {
public:
  SensorHub()
    : _simulationMode(cfg::SIMULATION_MODE),
      _ecuEnabled(cfg::ECU_MANAGER_ENABLED),
      _ecuOnline(false),
      _lastFastUpdateMs(0),
      _lastSlowUpdateMs(0),
      _lastDistanceMs(0),
      _simPhase(0.0f),
      _distanceMeters(0.0f) {
    _speed.timeoutMs = cfg::SPEED_TIMEOUT_MS;
    _rpm.timeoutMs = cfg::RPM_TIMEOUT_MS;
    _afr.timeoutMs = cfg::AFR_TIMEOUT_MS;
    _temp.timeoutMs = cfg::TEMP_TIMEOUT_MS;
    _batt.timeoutMs = cfg::BATT_TIMEOUT_MS;
    _fuel.timeoutMs = cfg::FUEL_TIMEOUT_MS;
    _throttle.timeoutMs = cfg::ECU_EXTERNAL_TIMEOUT_MS;
    _map.timeoutMs = cfg::ECU_EXTERNAL_TIMEOUT_MS;
    _injPulse.timeoutMs = cfg::ECU_EXTERNAL_TIMEOUT_MS;
  }

  bool begin() {
    analogReadResolution(12);
    analogSetPinAttenuation(cfg::PIN_AFR_ADC, ADC_11db);
    analogSetPinAttenuation(cfg::PIN_TEMP_ADC, ADC_11db);
    analogSetPinAttenuation(cfg::PIN_BATT_ADC, ADC_11db);
    analogSetPinAttenuation(cfg::PIN_FUEL_ADC, ADC_11db);
    analogSetPinAttenuation(cfg::PIN_TPS_ADC, ADC_11db);
    analogSetPinAttenuation(cfg::PIN_MAP_ADC, ADC_11db);

    pinMode(cfg::PIN_RPM, INPUT_PULLUP);
    pinMode(cfg::PIN_SPEED, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(cfg::PIN_RPM), isrRpm, FALLING);
    attachInterrupt(digitalPinToInterrupt(cfg::PIN_SPEED), isrSpeed, FALLING);

    if (cfg::DEBUG_MODE) {
      Serial.println("[SENSOR] Interrupts and ADC ready");
    }
    return true;
  }

  void setSimulationMode(bool enabled) { _simulationMode = enabled; }
  void setEcuEnabled(bool enabled) { _ecuEnabled = enabled; }
  void setEcuOnline(bool online) { _ecuOnline = online; }

  bool ecuEnabled() const { return _ecuEnabled; }
  bool ecuOnline() const { return _ecuOnline; }
  bool simulationMode() const { return _simulationMode; }

  void injectSpeed(float value, bool valid = true) { _extSpeed = { value, valid, millis() }; }
  void injectRpm(float value, bool valid = true) { _extRpm = { value, valid, millis() }; }
  void injectAfr(float value, bool valid = true) { _extAfr = { value, valid, millis() }; }
  void injectTemp(float value, bool valid = true) { _extTemp = { value, valid, millis() }; }
  void injectBattery(float value, bool valid = true) { _extBatt = { value, valid, millis() }; }
  void injectFuel(float value, bool valid = true) { _extFuel = { value, valid, millis() }; }
  void injectThrottle(float value, bool valid = true) { _extThrottle = { value, valid, millis() }; }
  void injectMap(float value, bool valid = true) { _extMap = { value, valid, millis() }; }
  void injectInjectorPulse(float value, bool valid = true) { _extInj = { value, valid, millis() }; }

  void update() {
    const uint32_t now = millis();

    if (_simulationMode) {
      updateSimulation(now);
      publishSnapshot(now);
      return;
    }

    if ((now - _lastFastUpdateMs) >= cfg::FAST_UPDATE_MS) {
      _lastFastUpdateMs = now;
      readFastChannels(now);
    }

    if ((now - _lastSlowUpdateMs) >= cfg::SLOW_UPDATE_MS) {
      _lastSlowUpdateMs = now;
      readSlowChannels(now);
    }

    updateDistance(now);
    applyTimeouts(now);
    publishSnapshot(now);
  }

  const DashboardSnapshot& snapshot() const { return _snapshot; }

private:
  static volatile uint32_t s_rpmPulseCount;
  static volatile uint32_t s_rpmLastEdgeUs;
  static volatile uint32_t s_speedPulseCount;
  static volatile uint32_t s_speedLastEdgeUs;

  bool _simulationMode;
  bool _ecuEnabled;
  bool _ecuOnline;

  uint32_t _lastFastUpdateMs;
  uint32_t _lastSlowUpdateMs;
  uint32_t _lastDistanceMs;

  float _simPhase;
  float _distanceMeters;

  ChannelState _speed;
  ChannelState _rpm;
  ChannelState _afr;
  ChannelState _temp;
  ChannelState _batt;
  ChannelState _fuel;
  ChannelState _throttle;
  ChannelState _map;
  ChannelState _injPulse;

  SensorFilter _speedFilter;
  SensorFilter _rpmFilter;
  SensorFilter _afrFilter;
  SensorFilter _tempFilter;
  SensorFilter _battFilter;
  SensorFilter _fuelFilter;
  SensorFilter _throttleFilter;
  SensorFilter _mapFilter;
  SensorFilter _injFilter;

  TimedValue _extSpeed;
  TimedValue _extRpm;
  TimedValue _extAfr;
  TimedValue _extTemp;
  TimedValue _extBatt;
  TimedValue _extFuel;
  TimedValue _extThrottle;
  TimedValue _extMap;
  TimedValue _extInj;

  DashboardSnapshot _snapshot;

  static void IRAM_ATTR isrRpm() {
    const uint32_t nowUs = micros();
    if ((nowUs - s_rpmLastEdgeUs) < cfg::RPM_MIN_EDGE_US) return;
    s_rpmLastEdgeUs = nowUs;
    ++s_rpmPulseCount;
  }

  static void IRAM_ATTR isrSpeed() {
    const uint32_t nowUs = micros();
    if ((nowUs - s_speedLastEdgeUs) < cfg::SPEED_MIN_EDGE_US) return;
    s_speedLastEdgeUs = nowUs;
    ++s_speedPulseCount;
  }

  void readFastChannels(uint32_t now) {
    float speedRaw = 0.0f;
    bool speedValid = false;
    float rpmRaw = 0.0f;
    bool rpmValid = false;
    float afrRaw = 14.7f;
    bool afrValid = false;
    float throttleRaw = 0.0f;
    bool throttleValid = false;
    float mapRaw = 101.3f;
    bool mapValid = false;
    float injRaw = 0.0f;
    bool injValid = false;

    // Priority 1: external injected telemetry (future GPS / ECU / BLE)
    if (_extSpeed.isFresh(now, cfg::ECU_EXTERNAL_TIMEOUT_MS)) { speedRaw = _extSpeed.value; speedValid = true; }
    if (_extRpm.isFresh(now, cfg::ECU_EXTERNAL_TIMEOUT_MS))   { rpmRaw = _extRpm.value; rpmValid = true; }
    if (_extAfr.isFresh(now, cfg::ECU_EXTERNAL_TIMEOUT_MS))   { afrRaw = _extAfr.value; afrValid = true; }
    if (_extThrottle.isFresh(now, cfg::ECU_EXTERNAL_TIMEOUT_MS)) { throttleRaw = _extThrottle.value; throttleValid = true; }
    if (_extMap.isFresh(now, cfg::ECU_EXTERNAL_TIMEOUT_MS))    { mapRaw = _extMap.value; mapValid = true; }
    if (_extInj.isFresh(now, cfg::ECU_EXTERNAL_TIMEOUT_MS))    { injRaw = _extInj.value; injValid = true; }

    // Priority 2: hardware sensors
    if (!speedValid) {
      float measured = readSpeedSensor(now, speedValid);
      if (speedValid) speedRaw = measured;
    }
    if (!rpmValid) {
      float measured = readRpmSensor(now, rpmValid);
      if (rpmValid) rpmRaw = measured;
    }
    if (!afrValid) {
      float measured = readAfrSensor(now, afrValid);
      if (afrValid) afrRaw = measured;
    }

    // Optional analog sensors for future load estimation
    if (!throttleValid) {
      float measured = readThrottleSensor(now, throttleValid);
      if (throttleValid) throttleRaw = measured;
    }
    if (!mapValid) {
      float measured = readMapSensor(now, mapValid);
      if (mapValid) mapRaw = measured;
    }
    if (!injValid) {
      float measured = readInjectorPulseEstimate(now, injValid);
      if (injValid) injRaw = measured;
    }

    updateChannel(_speed, _speedFilter, speedRaw, speedValid, 0.0f, 10.0f, 0.0f, 220.0f, now);
    updateChannel(_rpm, _rpmFilter, rpmRaw, rpmValid, 0.0f, 800.0f, 0.0f, 14000.0f, now);
    updateChannel(_afr, _afrFilter, afrRaw, afrValid, 14.7f, 0.8f, 8.0f, 22.0f, now);
    updateChannel(_throttle, _throttleFilter, throttleRaw, throttleValid, 0.0f, 15.0f, 0.0f, 100.0f, now);
    updateChannel(_map, _mapFilter, mapRaw, mapValid, 101.3f, 15.0f, 0.0f, 300.0f, now);
    updateChannel(_injPulse, _injFilter, injRaw, injValid, 0.0f, 3.0f, 0.0f, 20.0f, now);
  }

  void readSlowChannels(uint32_t now) {
    float tempRaw = 25.0f;
    bool tempValid = false;
    float battRaw = 12.6f;
    bool battValid = false;
    float fuelRaw = 100.0f;
    bool fuelValid = false;

    if (_extTemp.isFresh(now, cfg::ECU_EXTERNAL_TIMEOUT_MS)) { tempRaw = _extTemp.value; tempValid = true; }
    if (_extBatt.isFresh(now, cfg::ECU_EXTERNAL_TIMEOUT_MS)) { battRaw = _extBatt.value; battValid = true; }
    if (_extFuel.isFresh(now, cfg::ECU_EXTERNAL_TIMEOUT_MS)) { fuelRaw = _extFuel.value; fuelValid = true; }

    if (!tempValid) {
      float measured = readTempSensor(now, tempValid);
      if (tempValid) tempRaw = measured;
    }
    if (!battValid) {
      float measured = readBatterySensor(now, battValid);
      if (battValid) battRaw = measured;
    }
    if (!fuelValid) {
      float measured = readFuelSensor(now, fuelValid);
      if (fuelValid) fuelRaw = measured;
    }

    updateChannel(_temp, _tempFilter, tempRaw, tempValid, 25.0f, 1.5f, -20.0f, 160.0f, now);
    updateChannel(_batt, _battFilter, battRaw, battValid, 12.6f, 0.4f, 6.0f, 16.5f, now);
    updateChannel(_fuel, _fuelFilter, fuelRaw, fuelValid, 100.0f, 5.0f, 0.0f, 100.0f, now);
  }

  void updateChannel(ChannelState& ch,
                     SensorFilter& filter,
                     float raw,
                     bool rawValid,
                     float fallback,
                     float maxDelta,
                     float minValue,
                     float maxValue,
                     uint32_t now) {
    const float safeRaw = clampValue(raw, minValue, maxValue);
    bool accepted = false;
    float filtered = filter.process(safeRaw, rawValid, fallback, maxDelta, accepted);

    if (accepted) {
      ch.value = filtered;
      ch.lastGood = filtered;
      ch.lastGoodMs = now;
      ch.valid = true;
      ch.state = SensorState::Ok;
    } else {
      ch.valid = false;
      if (ch.lastGoodMs == 0 || ((now - ch.lastGoodMs) > ch.timeoutMs)) {
        ch.state = SensorState::Offline;
        ch.value = ch.lastGoodMs ? ch.lastGood : fallback;
        ch.errorCount++;
      } else {
        ch.state = SensorState::Warning;
        ch.value = ch.lastGood;
      }
    }
    ch.lastUpdateMs = now;
  }

  float readSpeedSensor(uint32_t now, bool& valid) {
    noInterrupts();
    const uint32_t pulseCount = s_speedPulseCount;
    const uint32_t lastEdgeUs  = s_speedLastEdgeUs;
    interrupts();

    if (_lastSpeedCalcMs == 0) {
      _lastSpeedCalcMs = now;
      _speed.valid = false;
      valid = false;
      return _speed.lastGood;
    }

    const uint32_t elapsedMs = now - _lastSpeedCalcMs;
    _lastSpeedCalcMs = now;

    if ((micros() - lastEdgeUs) > cfg::SPEED_TIMEOUT_US) {
      valid = false;
      return 0.0f;
    }

    noInterrupts();
    const uint32_t deltaCount = pulseCount - _prevSpeedPulseCount;
    _prevSpeedPulseCount = pulseCount;
    interrupts();

    if (elapsedMs == 0) {
      valid = false;
      return _speed.lastGood;
    }

    float rotations = static_cast<float>(deltaCount) / cfg::SPEED_PULSES_PER_REV;
    float meters = rotations * cfg::WHEEL_CIRCUMFERENCE_M;
    float kmh = (meters / (elapsedMs / 1000.0f)) * 3.6f * cfg::SPEED_CAL_FACTOR;
    kmh = clampValue(kmh, 0.0f, 220.0f);

    valid = true;
    return kmh;
  }

  float readRpmSensor(uint32_t now, bool& valid) {
    noInterrupts();
    const uint32_t pulseCount = s_rpmPulseCount;
    const uint32_t lastEdgeUs  = s_rpmLastEdgeUs;
    interrupts();

    if (_lastRpmCalcMs == 0) {
      _lastRpmCalcMs = now;
      _prevRpmPulseCount = pulseCount;
      valid = false;
      return _rpm.lastGood;
    }

    const uint32_t elapsedMs = now - _lastRpmCalcMs;
    _lastRpmCalcMs = now;

    if ((micros() - lastEdgeUs) > cfg::RPM_TIMEOUT_US) {
      valid = false;
      return 0.0f;
    }

    noInterrupts();
    const uint32_t deltaCount = pulseCount - _prevRpmPulseCount;
    _prevRpmPulseCount = pulseCount;
    interrupts();

    if (elapsedMs == 0) {
      valid = false;
      return _rpm.lastGood;
    }

    float rpm = (static_cast<float>(deltaCount) / cfg::RPM_PULSES_PER_REV) * (60000.0f / elapsedMs);
    rpm *= cfg::RPM_CAL_FACTOR;
    rpm = clampValue(rpm, 0.0f, 14000.0f);
    valid = true;
    return rpm;
  }

  float readAfrSensor(uint32_t, bool& valid) {
    const uint16_t raw = AnalogSampler::averageRaw(cfg::PIN_AFR_ADC, 6, 40);
    float adcVolt = AnalogSampler::rawToVoltage(raw);
    float sensorVolt = adcVolt * cfg::AFR_DIVIDER_RATIO;
    sensorVolt = clampValue(sensorVolt, 0.0f, 5.2f);

    if (sensorVolt < cfg::AFR_SENSOR_V_MIN - 0.10f || sensorVolt > cfg::AFR_SENSOR_V_MAX + 0.10f) {
      valid = false;
      return _afr.lastGood;
    }

    float afr = mapFloat(sensorVolt,
                         cfg::AFR_SENSOR_V_MIN,
                         cfg::AFR_SENSOR_V_MAX,
                         cfg::AFR_MIN_VALUE,
                         cfg::AFR_MAX_VALUE);
    afr = clampValue(afr, 8.0f, 22.0f);
    valid = true;
    return afr;
  }

  float readTempSensor(uint32_t, bool& valid) {
    const uint16_t raw = AnalogSampler::averageRaw(cfg::PIN_TEMP_ADC, 6, 40);
    float v = AnalogSampler::rawToVoltage(raw);

    if (v < 0.02f || v > (cfg::ADC_REF_VOLT - 0.02f)) {
      valid = false;
      return _temp.lastGood;
    }

    float r = cfg::TEMP_NTC_FIXED_OHM * (v / (cfg::ADC_REF_VOLT - v));
    if (r <= 0.0f) {
      valid = false;
      return _temp.lastGood;
    }

    float invT = (1.0f / 298.15f) + (1.0f / cfg::TEMP_NTC_BETA) * logf(r / cfg::TEMP_NTC_R25_OHM);
    float tempK = 1.0f / invT;
    float tempC = tempK - 273.15f + cfg::TEMP_OFFSET_C;
    tempC = clampValue(tempC, -20.0f, 160.0f);

    valid = true;
    return tempC;
  }

  float readBatterySensor(uint32_t, bool& valid) {
    const uint16_t raw = AnalogSampler::averageRaw(cfg::PIN_BATT_ADC, 8, 40);
    float v = AnalogSampler::rawToVoltage(raw) * cfg::BATT_DIVIDER_RATIO;
    if (v < 6.0f || v > 16.5f) {
      valid = false;
      return _batt.lastGood;
    }
    valid = true;
    return v;
  }

  float readFuelSensor(uint32_t, bool& valid) {
    const uint16_t raw = AnalogSampler::averageRaw(cfg::PIN_FUEL_ADC, 8, 30);
    float pct = mapFloat(static_cast<float>(raw), cfg::FUEL_ADC_EMPTY, cfg::FUEL_ADC_FULL, 0.0f, 100.0f);
    pct = clampValue(pct, 0.0f, 100.0f);
    valid = (raw > 0 && raw < 4095);
    return pct;
  }

  float readThrottleSensor(uint32_t, bool& valid) {
    const uint16_t raw = analogRead(cfg::PIN_TPS_ADC);
    float pct = mapFloat(static_cast<float>(raw), 200.0f, 3800.0f, 0.0f, 100.0f);
    pct = clampValue(pct, 0.0f, 100.0f);
    valid = (raw > 0 && raw < 4095);
    return pct;
  }

  float readMapSensor(uint32_t, bool& valid) {
    const uint16_t raw = analogRead(cfg::PIN_MAP_ADC);
    float kpa = mapFloat(static_cast<float>(raw), 100.0f, 3900.0f, 20.0f, 140.0f);
    kpa = clampValue(kpa, 0.0f, 300.0f);
    valid = (raw > 0 && raw < 4095);
    return kpa;
  }

  float readInjectorPulseEstimate(uint32_t, bool& valid) {
    // Placeholder injector pulse width derived from load.
    // Future ECU/K-Line data should overwrite this via injectInjectorPulse().
    float rpm = _rpm.value;
    float tps = _throttle.value;
    float map = _map.value;
    if (rpm < 100.0f) {
      valid = false;
      return 0.0f;
    }

    float load = 0.5f * (tps / 100.0f) + 0.5f * clampValue(map / 101.3f, 0.0f, 1.6f);
    float pulse = 1.0f + (load * 6.0f) + (rpm / 10000.0f) * 1.8f;
    pulse = clampValue(pulse, 0.8f, 18.0f);
    valid = true;
    return pulse;
  }

  void updateDistance(uint32_t now) {
    if (_lastDistanceMs == 0) {
      _lastDistanceMs = now;
      return;
    }

    const uint32_t dtMs = now - _lastDistanceMs;
    _lastDistanceMs = now;
    float hours = dtMs / 3600000.0f;
    _distanceMeters += _speed.value * hours * 1000.0f;
  }

  void applyTimeouts(uint32_t now) {
    updateStateTimeout(_speed, now);
    updateStateTimeout(_rpm, now);
    updateStateTimeout(_afr, now);
    updateStateTimeout(_temp, now);
    updateStateTimeout(_batt, now);
    updateStateTimeout(_fuel, now);
    updateStateTimeout(_throttle, now);
    updateStateTimeout(_map, now);
    updateStateTimeout(_injPulse, now);
  }

  void updateStateTimeout(ChannelState& ch, uint32_t now) {
    if (ch.lastGoodMs == 0) return;
    if ((now - ch.lastGoodMs) > ch.timeoutMs) {
      ch.state = SensorState::Offline;
      ch.valid = false;
    } else if (!ch.valid) {
      ch.state = SensorState::Warning;
    }
  }

  void updateSimulation(uint32_t now) {
    // Non-blocking synthetic telemetry for bench testing
    _simPhase += 0.035f;
    if (_simPhase > 10000.0f) _simPhase = 0.0f;

    float speed = 55.0f + 35.0f * sinf(_simPhase * 0.35f);
    speed = clampValue(speed, 0.0f, 125.0f);
    float rpm = 1400.0f + speed * 75.0f + 900.0f * fabsf(sinf(_simPhase * 1.3f));
    rpm = clampValue(rpm, 0.0f, 12000.0f);
    float afr = 14.7f + 1.4f * sinf(_simPhase * 1.7f);
    float temp = 35.0f + 55.0f * (1.0f - expf(-_simPhase * 0.01f)) + 2.0f * sinf(_simPhase * 0.5f);
    float batt = 12.6f + 1.2f * (rpm > 1800.0f ? 1.0f : 0.0f) + 0.2f * sinf(_simPhase * 2.2f);
    float fuel = clampValue(82.0f - _simPhase * 0.02f, 0.0f, 100.0f);
    float throttle = clampValue(25.0f + 40.0f * fabsf(sinf(_simPhase * 0.7f)), 0.0f, 100.0f);
    float map = clampValue(40.0f + 70.0f * fabsf(sinf(_simPhase * 0.55f)), 0.0f, 140.0f);
    float inj = clampValue(1.2f + throttle * 0.04f + rpm / 12000.0f * 4.0f, 0.8f, 15.0f);

    updateChannel(_speed, _speedFilter, speed, true, 0.0f, 8.0f, 0.0f, 220.0f, now);
    updateChannel(_rpm, _rpmFilter, rpm, true, 0.0f, 700.0f, 0.0f, 14000.0f, now);
    updateChannel(_afr, _afrFilter, afr, true, 14.7f, 0.8f, 8.0f, 22.0f, now);
    updateChannel(_temp, _tempFilter, temp, true, 25.0f, 1.2f, -20.0f, 160.0f, now);
    updateChannel(_batt, _battFilter, batt, true, 12.6f, 0.3f, 6.0f, 16.5f, now);
    updateChannel(_fuel, _fuelFilter, fuel, true, 100.0f, 1.0f, 0.0f, 100.0f, now);
    updateChannel(_throttle, _throttleFilter, throttle, true, 0.0f, 5.0f, 0.0f, 100.0f, now);
    updateChannel(_map, _mapFilter, map, true, 101.3f, 5.0f, 0.0f, 300.0f, now);
    updateChannel(_injPulse, _injFilter, inj, true, 0.0f, 1.0f, 0.0f, 20.0f, now);

    _distanceMeters += speed * (cfg::FAST_UPDATE_MS / 3600000.0f) * 1000.0f;
  }

  void publishSnapshot(uint32_t now) {
    _snapshot.timestampMs = now;
    _snapshot.speedKmh = _speed.value;
    _snapshot.rpm = static_cast<uint16_t>(_rpm.value + 0.5f);
    _snapshot.afr = _afr.value;
    _snapshot.engineTempC = _temp.value;
    _snapshot.batteryVolt = _batt.value;
    _snapshot.fuelPercent = _fuel.value;
    _snapshot.throttlePct = _throttle.value;
    _snapshot.mapKpa = _map.value;
    _snapshot.injectorPulseMs = _injPulse.value;
    _snapshot.distanceMeters = _distanceMeters;
    _snapshot.ecuEnabled = _ecuEnabled;
    _snapshot.ecuOnline = _ecuOnline;
    _snapshot.simulationMode = _simulationMode;

    _snapshot.speedState = _speed.state;
    _snapshot.rpmState = _rpm.state;
    _snapshot.afrState = _afr.state;
    _snapshot.tempState = _temp.state;
    _snapshot.battState = _batt.state;
    _snapshot.fuelState = _fuel.state;
    _snapshot.throttleState = _throttle.state;
    _snapshot.mapState = _map.state;
    _snapshot.injectorState = _injPulse.state;
  }

  uint32_t _prevRpmPulseCount = 0;
  uint32_t _prevSpeedPulseCount = 0;
  uint32_t _lastRpmCalcMs = 0;
  uint32_t _lastSpeedCalcMs = 0;
};

volatile uint32_t SensorHub::s_rpmPulseCount = 0;
volatile uint32_t SensorHub::s_rpmLastEdgeUs = 0;
volatile uint32_t SensorHub::s_speedPulseCount = 0;
volatile uint32_t SensorHub::s_speedLastEdgeUs = 0;

// =====================================================================
// FuelEstimator
// =====================================================================
class FuelEstimator {
public:
  FuelEstimator()
    : _instantKmL(0.0f),
      _averageKmL(0.0f),
      _l100Km(0.0f),
      _totalFuelL(0.0f),
      _totalDistanceKm(0.0f),
      _lastUpdateMs(0) {}

  void reset() {
    _instantKmL = 0.0f;
    _averageKmL = 0.0f;
    _l100Km = 0.0f;
    _totalFuelL = 0.0f;
    _totalDistanceKm = 0.0f;
    _lastUpdateMs = 0;
    _avgHistory.reset();
  }

  void update(DashboardSnapshot& s) {
    uint32_t now = millis();
    if (_lastUpdateMs == 0) _lastUpdateMs = now;
    uint32_t dtMs = now - _lastUpdateMs;
    _lastUpdateMs = now;

    float dtHours = dtMs / 3600000.0f;
    float rpm = static_cast<float>(s.rpm);
    float speed = s.speedKmh;
    float throttle = s.throttlePct;
    float map = s.mapKpa;

    float injectorPulseMs = s.injectorPulseMs;
    if (injectorPulseMs <= 0.0f) {
      injectorPulseMs = estimateInjectorPulseMs(rpm, throttle, map, s.engineTempC);
    }
    s.injectorPulseMs = injectorPulseMs;

    if (speed < 1.5f || rpm < 300.0f) {
      _instantKmL = 0.0f;
    } else {
      float duty = injectorPulseMs / (60000.0f / clampValue(rpm, 1.0f, 14000.0f));
      duty = clampValue(duty, 0.01f, 0.95f);
      float fuelFlowCcMin = cfg::INJECTOR_FLOW_CC_MIN * duty;
      float fuelFlowLh = fuelFlowCcMin * 60.0f / 1000.0f;
      if (fuelFlowLh < 0.01f) {
        _instantKmL = 0.0f;
      } else {
        _instantKmL = clampValue(speed / fuelFlowLh, 0.1f, 99.9f);
      }
    }

    if (dtHours > 0.0f) {
      _totalDistanceKm += speed * dtHours;
      if (_instantKmL > 0.01f && speed > 0.1f) {
        float fuelFlowLh = speed / _instantKmL;
        _totalFuelL += fuelFlowLh * dtHours;
      }
    }

    _avgHistory.push(_instantKmL);
    _averageKmL = _avgHistory.mean();
    if (_averageKmL < 0.01f && _instantKmL > 0.01f) _averageKmL = _instantKmL;

    if (_totalDistanceKm > 0.01f && _totalFuelL > 0.0001f) {
      _averageKmL = _totalDistanceKm / _totalFuelL;
      _l100Km = (_totalFuelL / _totalDistanceKm) * 100.0f;
    } else {
      _l100Km = 0.0f;
    }

    s.fuelInstantKmL = _instantKmL;
    s.fuelAverageKmL = _averageKmL;
    s.fuelLPer100Km = _l100Km;
  }

  float instantKmL() const { return _instantKmL; }
  float averageKmL() const { return _averageKmL; }
  float l100Km() const { return _l100Km; }
  float totalFuelL() const { return _totalFuelL; }
  float totalDistanceKm() const { return _totalDistanceKm; }

private:
  float _instantKmL;
  float _averageKmL;
  float _l100Km;
  float _totalFuelL;
  float _totalDistanceKm;
  uint32_t _lastUpdateMs;
  MovingAverage<float, 30> _avgHistory;

  float estimateInjectorPulseMs(float rpm, float throttlePct, float mapKpa, float tempC) {
    if (rpm < 100.0f) return 0.0f;
    float load = 0.55f * (throttlePct / 100.0f) + 0.45f * clampValue(mapKpa / 101.3f, 0.0f, 1.6f);
    float warmup = (tempC < 60.0f) ? 1.12f : 1.0f;
    float pulse = (1.0f + load * 5.5f + (rpm / 10000.0f) * 1.8f) * warmup;
    return clampValue(pulse, 0.8f, 16.0f);
  }
};

// =====================================================================
// EngineHealthModel
// =====================================================================
enum class EngineTier : uint8_t {
  Excellent = 0,
  Good,
  Warning,
  Critical
};

class EngineHealthModel {
public:
  EngineHealthModel() : _health(100.0f), _tier(EngineTier::Excellent) {}

  void reset() {
    _health = 100.0f;
    _tier = EngineTier::Excellent;
    _rpmHistory.reset();
    _afrHistory.reset();
  }

  void update(const DashboardSnapshot& s) {
    _rpmHistory.push(static_cast<float>(s.rpm));
    _afrHistory.push(s.afr);

    float tempScore = scoreTemperature(s.engineTempC);
    float afrScore   = scoreAfr(s.afr, _afrHistory.stddev());
    float voltScore  = scoreBattery(s.batteryVolt);
    float rpmScore   = scoreRpm(s.rpm, _rpmHistory.stddev());
    float fuelScore  = scoreFuelEconomy(s.fuelAverageKmL, s.fuelLPer100Km);

    float health = (tempScore * cfg::HEALTH_W_TEMP) +
                   (afrScore  * cfg::HEALTH_W_AFR) +
                   (voltScore  * cfg::HEALTH_W_VOLT) +
                   (rpmScore   * cfg::HEALTH_W_RPM) +
                   (fuelScore  * cfg::HEALTH_W_FUEL);

    if (s.tempState != SensorState::Ok) health -= 3.0f;
    if (s.battState == SensorState::Offline) health -= 5.0f;
    if (s.afrState  == SensorState::Offline) health -= 5.0f;
    if (s.ecuEnabled && !s.ecuOnline) health -= 5.0f;

    _health = clampValue(health, 0.0f, 100.0f);

    if (_health >= 80.0f) _tier = EngineTier::Excellent;
    else if (_health >= 60.0f) _tier = EngineTier::Good;
    else if (_health >= 40.0f) _tier = EngineTier::Warning;
    else _tier = EngineTier::Critical;
  }

  float health() const { return _health; }
  EngineTier tier() const { return _tier; }

  const char* statusString() const {
    switch (_tier) {
      case EngineTier::Excellent: return "EXCELLENT";
      case EngineTier::Good:      return "GOOD";
      case EngineTier::Warning:   return "WARNING";
      case EngineTier::Critical:  return "CRITICAL";
      default:                    return "UNKNOWN";
    }
  }

  uint16_t statusColor() const {
    switch (_tier) {
      case EngineTier::Excellent: return cfg::COL_GREEN;
      case EngineTier::Good:      return cfg::COL_ACCENT;
      case EngineTier::Warning:   return cfg::COL_YELLOW;
      case EngineTier::Critical:  return cfg::COL_RED;
      default:                    return cfg::COL_WHITE;
    }
  }

  float afrStability() const {
    float stdev = _afrHistory.stddev();
    return clampValue(100.0f - (stdev * 28.0f), 0.0f, 100.0f);
  }

private:
  float _health;
  EngineTier _tier;
  HistoryWindow<float, 24> _rpmHistory;
  HistoryWindow<float, 20> _afrHistory;

  float scoreTemperature(float tempC) {
    if (tempC < 0.0f) return 25.0f;
    if (tempC < 40.0f) return 60.0f;
    if (tempC <= 95.0f) return 100.0f;
    if (tempC <= 110.0f) return mapFloat(tempC, 95.0f, 110.0f, 100.0f, 40.0f);
    if (tempC <= 120.0f) return mapFloat(tempC, 110.0f, 120.0f, 40.0f, 10.0f);
    return 0.0f;
  }

  float scoreAfr(float afr, float stability) {
    float diff = fabsf(afr - 14.7f);
    float base = clampValue(100.0f - (diff * 18.0f), 0.0f, 100.0f);
    return (base * 0.75f) + (stability * 0.25f);
  }

  float scoreBattery(float volt) {
    if (volt < 10.0f) return 0.0f;
    if (volt < 11.5f) return 25.0f;
    if (volt < 12.2f) return 60.0f;
    if (volt <= 14.8f) return 100.0f;
    if (volt <= 15.2f) return 75.0f;
    return 20.0f;
  }

  float scoreRpm(uint16_t rpm, float jitter) {
    float base = 100.0f;
    if (rpm == 0) base = 40.0f;
    else if (rpm < 700) base = 60.0f;
    else if (rpm > 10500) base = 50.0f;

    float smoothness = clampValue(100.0f - (jitter * 0.25f), 0.0f, 100.0f);
    return (base * 0.5f) + (smoothness * 0.5f);
  }

  float scoreFuelEconomy(float kmL, float l100) {
    if (kmL <= 0.1f) return 55.0f;
    if (kmL >= 55.0f) return 100.0f;
    if (kmL >= 40.0f) return 90.0f;
    if (kmL >= 30.0f) return 80.0f;
    if (kmL >= 20.0f) return 65.0f;
    if (kmL >= 12.0f) return 45.0f;
    if (l100 > 0.0f && l100 < 3.0f) return 90.0f;
    return 30.0f;
  }
};

// =====================================================================
// K-Line / KWP2000 scaffolding
// =====================================================================
constexpr size_t MAX_ECU_PAYLOAD = 28;
constexpr size_t MAX_ECU_FRAME   = 40;

struct ECUFrame {
  uint8_t header = 0;
  uint8_t target = 0;
  uint8_t source = 0;
  uint8_t length = 0;
  uint8_t payload[MAX_ECU_PAYLOAD] = {};
  uint8_t payloadLen = 0;
  uint8_t checksum = 0;
  bool valid = false;
  uint32_t timestampMs = 0;
};

class Kwp2000Parser {
public:
  static uint8_t checksum(const uint8_t* data, size_t length) {
    uint16_t sum = 0;
    for (size_t i = 0; i < length; ++i) sum += data[i];
    return static_cast<uint8_t>(sum & 0xFF);
  }

  static size_t buildRequestFrame(uint8_t* out,
                                  size_t outSize,
                                  uint8_t header,
                                  uint8_t target,
                                  uint8_t source,
                                  uint8_t service,
                                  uint8_t pid) {
    if (outSize < 7) return 0;
    out[0] = header;
    out[1] = target;
    out[2] = source;
    out[3] = 0x02;
    out[4] = service;
    out[5] = pid;
    out[6] = checksum(out, 6);
    return 7;
  }

  static bool decodeFrame(const uint8_t* raw, size_t rawLen, ECUFrame& frame) {
    if (rawLen < 5) return false;

    const uint8_t header = raw[0];
    if (header != 0x68 && header != 0x80 && header != 0xC2 && header != 0x02 && header != 0x0E) {
      return false;
    }

    const uint8_t length = raw[3];
    if (length > MAX_ECU_PAYLOAD) return false;

    const size_t totalLen = static_cast<size_t>(4) + static_cast<size_t>(length) + 1;
    if (rawLen < totalLen) return false;

    uint8_t cs = checksum(raw, 4 + length);
    if (cs != raw[4 + length]) return false;

    frame.header = raw[0];
    frame.target = raw[1];
    frame.source = raw[2];
    frame.length = length;
    frame.payloadLen = length;
    frame.checksum = raw[4 + length];
    frame.timestampMs = millis();
    frame.valid = true;

    for (size_t i = 0; i < length; ++i) {
      frame.payload[i] = raw[4 + i];
    }
    return true;
  }
};

class KLineManager {
public:
  KLineManager()
    : _serial(Serial2),
      _state(KLineState::Disabled),
      _protocol(ProtocolType::Unknown),
      _enabled(false),
      _connected(false),
      _ecuAddress(cfg::ECU_ADDR_DEFAULT),
      _testerAddress(cfg::TESTER_ADDR_DEFAULT),
      _header(cfg::KLINE_HEADER_DEFAULT),
      _lastProbeMs(0),
      _lastRxMs(0),
      _lastRetryMs(0),
      _errorCount(0),
      _rxLen(0),
      _haveFrame(false) {}

  bool begin(uint8_t rxPin = cfg::PIN_KLINE_RX,
             uint8_t txPin = cfg::PIN_KLINE_TX,
             uint32_t baud = cfg::KLINE_BAUD) {
    _rxPin = rxPin;
    _txPin = txPin;
    _baud = baud;

    pinMode(_txPin, OUTPUT);
    digitalWrite(_txPin, HIGH);

    _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
    _serial.setTimeout(cfg::KLINE_RX_TIMEOUT_MS);
    _state = KLineState::Idle;

    if (cfg::DEBUG_MODE) {
      Serial.printf("[KLINE] UART ready RX=%u TX=%u BAUD=%lu\n",
                    _rxPin, _txPin, static_cast<unsigned long>(_baud));
      Serial.println("[KLINE] Use MC33290 / L9637D transceiver between ECU K-Line and ESP32");
    }
    return true;
  }

  void enable(bool enabled) {
    _enabled = enabled;
    if (!enabled) {
      _state = KLineState::Disabled;
      _connected = false;
      return;
    }
    _state = KLineState::Reconnecting;
  }

  bool isEnabled() const { return _enabled; }
  bool isConnected() const { return _connected; }
  KLineState state() const { return _state; }
  ProtocolType protocol() const { return _protocol; }
  uint32_t errorCount() const { return _errorCount; }

  const char* stateString() const {
    switch (_state) {
      case KLineState::Disabled:        return "DISABLED";
      case KLineState::Idle:            return "IDLE";
      case KLineState::Probing:         return "PROBING";
      case KLineState::Connected:       return "CONNECTED";
      case KLineState::Requesting:      return "REQUESTING";
      case KLineState::WaitingResponse: return "WAITING";
      case KLineState::Error:           return "ERROR";
      case KLineState::Reconnecting:    return "RECONNECTING";
      default:                          return "UNKNOWN";
    }
  }

  void update() {
    if (!_enabled) return;

    readIncoming();

    const uint32_t now = millis();

    if (_connected && ((now - _lastRxMs) > cfg::KLINE_RX_TIMEOUT_MS)) {
      _connected = false;
      _state = KLineState::Reconnecting;
      if (cfg::DEBUG_MODE) Serial.println("[KLINE] Timeout, scheduling reconnect");
    }

    if (!_connected && ((now - _lastRetryMs) > cfg::KLINE_RETRY_MS)) {
      _lastRetryMs = now;
      _state = KLineState::Probing;
      requestPid(cfg::OBD_MODE_CURRENT, cfg::PID_SUPPORTED_01, true);
    }
  }

  bool requestPid(uint8_t service, uint8_t pid, bool force = false) {
    if (!_enabled) return false;
    if (!_connected && !force) return false;

    uint8_t frame[8] = {};
    size_t frameLen = Kwp2000Parser::buildRequestFrame(frame,
                                                       sizeof(frame),
                                                       _header,
                                                       _ecuAddress,
                                                       _testerAddress,
                                                       service,
                                                       pid);
    if (frameLen == 0) return false;

    _serial.write(frame, frameLen);
    _serial.flush();
    _state = KLineState::WaitingResponse;
    if (cfg::DEBUG_MODE) {
      Serial.printf("[KLINE] TX service=0x%02X pid=0x%02X len=%u\n", service, pid, static_cast<unsigned>(frameLen));
    }
    return true;
  }

  bool pollFrame(ECUFrame& frame) {
    if (!_haveFrame) return false;
    frame = _pendingFrame;
    _haveFrame = false;
    return true;
  }

  void markConnected(ProtocolType proto = ProtocolType::ISO9141) {
    _connected = true;
    _protocol = proto;
    _state = KLineState::Connected;
    _lastRxMs = millis();
    if (cfg::DEBUG_MODE) Serial.println("[KLINE] ECU connected");
  }

  void reset() {
    _connected = false;
    _state = _enabled ? KLineState::Reconnecting : KLineState::Disabled;
    _rxLen = 0;
    _haveFrame = false;
    _protocol = ProtocolType::Unknown;
  }

private:
  HardwareSerial& _serial;
  KLineState _state;
  ProtocolType _protocol;
  bool _enabled;
  bool _connected;

  uint8_t _rxPin;
  uint8_t _txPin;
  uint32_t _baud;
  uint8_t _ecuAddress;
  uint8_t _testerAddress;
  uint8_t _header;

  uint32_t _lastProbeMs;
  uint32_t _lastRxMs;
  uint32_t _lastRetryMs;
  uint32_t _errorCount;

  uint8_t _rxBuffer[MAX_ECU_FRAME];
  size_t _rxLen;
  ECUFrame _pendingFrame;
  bool _haveFrame;

  void readIncoming() {
    while (_serial.available()) {
      if (_rxLen < sizeof(_rxBuffer)) {
        _rxBuffer[_rxLen++] = static_cast<uint8_t>(_serial.read());
      } else {
        // Simple overflow protection: drop the buffer and resync
        _rxLen = 0;
        _errorCount++;
        if (cfg::DEBUG_MODE) Serial.println("[KLINE] RX overflow, resync");
        return;
      }
    }

    if (_rxLen < 5) return;

    // Try to decode a complete frame from the current buffer
    size_t frameLen = static_cast<size_t>(4) + static_cast<size_t>(_rxBuffer[3]) + 1;
    if (frameLen > sizeof(_rxBuffer)) {
      if (_rxLen > 1) {
        memmove(_rxBuffer, _rxBuffer + 1, _rxLen - 1);
        --_rxLen;
      } else {
        _rxLen = 0;
      }
      _errorCount++;
      return;
    }
    if (_rxLen < frameLen) return;

    ECUFrame frame;
    if (Kwp2000Parser::decodeFrame(_rxBuffer, frameLen, frame)) {
      _pendingFrame = frame;
      _haveFrame = true;
      _lastRxMs = millis();
      if (!_connected) markConnected(ProtocolType::ISO9141);
      if (cfg::DEBUG_MODE) {
        Serial.printf("[KLINE] RX frame len=%u payload=%u\n",
                      static_cast<unsigned>(frameLen), static_cast<unsigned>(frame.payloadLen));
      }
    } else {
      _errorCount++;
      if (cfg::DEBUG_MODE) Serial.println("[KLINE] Invalid frame");
      if (_rxLen > 1) {
        memmove(_rxBuffer, _rxBuffer + 1, _rxLen - 1);
        --_rxLen;
        return;
      }
    }

    // Drop consumed bytes and keep remaining tail, if any
    if (_rxLen > frameLen) {
      memmove(_rxBuffer, _rxBuffer + frameLen, _rxLen - frameLen);
      _rxLen -= frameLen;
    } else {
      _rxLen = 0;
    }
  }
};

// =====================================================================
// ECU Request Handler
// =====================================================================
struct RequestSlot {
  uint8_t service = 0;
  uint8_t pid = 0;
  uint32_t intervalMs = 0;
  uint32_t lastMs = 0;
};

class ECURequestHandler {
public:
  void begin(bool enabled) {
    _enabled = enabled;
    _slotCount = 0;
    _cursor = 0;
    _lastProcessMs = 0;

    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_ENGINE_RPM,   100);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_SPEED,        100);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_COOLANT_TEMP, 500);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_THROTTLE_POS, 200);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_O2_SENSOR_1,  200);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_MAP,          200);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_IAT,          500);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_BATTERY_VOLT, 1000);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_STFT,         500);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_LTFT,         1000);
  }

  void update(KLineManager& kline, SensorHub& hub) {
    if (!_enabled) return;

    const uint32_t now = millis();

    // Keep ECU online state in sync with K-Line layer
    hub.setEcuOnline(kline.isConnected());

    ECUFrame frame;
    while (kline.pollFrame(frame)) {
      decodeFrame(frame, hub);
    }

    if (!kline.isConnected()) return;

    if (_slotCount == 0) return;
    if ((now - _lastProcessMs) < 20) return;
    _lastProcessMs = now;

    RequestSlot& slot = _slots[_cursor];
    if ((now - slot.lastMs) >= slot.intervalMs) {
      slot.lastMs = now;
      kline.requestPid(slot.service, slot.pid, false);
    }
    _cursor = (_cursor + 1) % _slotCount;
  }

private:
  bool _enabled = false;
  RequestSlot _slots[12];
  uint8_t _slotCount = 0;
  uint8_t _cursor = 0;
  uint32_t _lastProcessMs = 0;

  void addSlot(uint8_t service, uint8_t pid, uint32_t intervalMs) {
    if (_slotCount >= (sizeof(_slots) / sizeof(_slots[0]))) return;
    _slots[_slotCount++] = { service, pid, intervalMs, 0 };
  }

  static void decodeFrame(const ECUFrame& frame, SensorHub& hub) {
    if (!frame.valid || frame.payloadLen < 2) return;
    const uint8_t service = frame.payload[0];
    const uint8_t pid = frame.payload[1];
    if (service < 0x40) return; // response must be service + 0x40

    switch (pid) {
      case cfg::PID_ENGINE_RPM:
        if (frame.payloadLen >= 4) {
          float rpm = ((static_cast<uint16_t>(frame.payload[2]) << 8) | frame.payload[3]) / 4.0f;
          hub.injectRpm(rpm, true);
        }
        break;

      case cfg::PID_SPEED:
        if (frame.payloadLen >= 3) {
          hub.injectSpeed(static_cast<float>(frame.payload[2]), true);
        }
        break;

      case cfg::PID_COOLANT_TEMP:
      case cfg::PID_IAT:
        if (frame.payloadLen >= 3) {
          float tempC = static_cast<float>(frame.payload[2]) - 40.0f;
          hub.injectTemp(tempC, true);
        }
        break;

      case cfg::PID_THROTTLE_POS:
        if (frame.payloadLen >= 3) {
          float tps = (static_cast<float>(frame.payload[2]) * 100.0f) / 255.0f;
          hub.injectThrottle(tps, true);
        }
        break;

      case cfg::PID_O2_SENSOR_1:
        if (frame.payloadLen >= 3) {
          float voltage = static_cast<float>(frame.payload[2]) * 0.005f;
          float afr = clampValue(14.7f + ((voltage - 0.45f) * 6.0f), 8.0f, 22.0f);
          hub.injectAfr(afr, true);
        }
        break;

      case cfg::PID_MAP:
        if (frame.payloadLen >= 3) {
          hub.injectMap(static_cast<float>(frame.payload[2]), true);
        }
        break;

      case cfg::PID_BATTERY_VOLT:
        if (frame.payloadLen >= 3) {
          hub.injectBattery(static_cast<float>(frame.payload[2]) * 0.1f, true);
        }
        break;

      case cfg::PID_STFT:
      case cfg::PID_LTFT:
        // Fuel trim available for future AFR correction
        break;

      default:
        break;
    }
  }
};

// =====================================================================
// Dashboard UI
// =====================================================================
class DashboardUI {
public:
  DashboardUI()
    : _topSprite(&_tft),
      _midSprite(&_tft),
      _botSprite(&_tft),
      _activeWarning(WarningType::None),
      _warningBlinkState(false),
      _warningVisible(false),
      _lastBlinkMs(0),
      _lastFullRedrawMs(0),
      _forceFullRedraw(true) {}

  bool begin() {
    _tft.init();
    _tft.setRotation(1);
    _tft.fillScreen(cfg::COL_BG);
    _tft.setTextWrap(false);

    _topSprite.setColorDepth(16);
    _midSprite.setColorDepth(16);
    _botSprite.setColorDepth(16);

    _topSprite.createSprite(cfg::TFT_WIDTH, cfg::ZONE_TOP_H);
    _midSprite.createSprite(cfg::TFT_WIDTH, cfg::ZONE_MID_H);
    _botSprite.createSprite(cfg::TFT_WIDTH, cfg::ZONE_BOT_H);

    if (cfg::DEBUG_MODE) {
      Serial.println("[UI] TFT initialized");
      if (!psramFound()) {
        Serial.println("[UI][WARN] PSRAM not detected; large sprites may reduce headroom");
      }
    }

    runBootAnimation();
    drawStaticFrame();
    _forceFullRedraw = true;
    _lastFullRedrawMs = millis();
    return true;
  }

  void update(const DashboardSnapshot& s) {
    updateBlinkState();

    WarningType warning = evaluateWarning(s);
    bool warningChanged = (warning != _activeWarning);
    _activeWarning = warning;
    _warningVisible = (warning != WarningType::None);
    if (warningChanged) _forceFullRedraw = true;

    const uint32_t now = millis();
    if ((now - _lastFullRedrawMs) >= cfg::FULL_REDRAW_MS) {
      _forceFullRedraw = true;
      _lastFullRedrawMs = now;
    }

    bool topDirty = _forceFullRedraw || warningChanged ||
                    (abs(static_cast<int>(s.rpm) - static_cast<int>(_prev.rpm)) > 25) ||
                    (s.ecuOnline != _prev.ecuOnline) ||
                    (s.ecuEnabled != _prev.ecuEnabled) ||
                    (s.simulationMode != _prev.simulationMode);

    bool midDirty = _forceFullRedraw ||
                    (fabsf(s.speedKmh - _prev.speedKmh) >= 1.0f) ||
                    (fabsf(s.afr - _prev.afr) >= 0.05f) ||
                    (fabsf(s.engineTempC - _prev.engineTempC) >= 0.5f) ||
                    (fabsf(s.batteryVolt - _prev.batteryVolt) >= 0.05f) ||
                    (fabsf(s.fuelPercent - _prev.fuelPercent) >= 0.5f) ||
                    (fabsf(s.throttlePct - _prev.throttlePct) >= 1.0f) ||
                    (fabsf(s.mapKpa - _prev.mapKpa) >= 1.0f);

    bool botDirty = _forceFullRedraw ||
                    (fabsf(s.fuelInstantKmL - _prev.fuelInstantKmL) >= 0.2f) ||
                    (fabsf(s.fuelAverageKmL - _prev.fuelAverageKmL) >= 0.2f) ||
                    (fabsf(s.fuelLPer100Km - _prev.fuelLPer100Km) >= 0.1f) ||
                    (fabsf(s.engineHealth - _prev.engineHealth) >= 1.0f);

    if (topDirty) {
      drawTopZone(s);
      _prev.ecuOnline = s.ecuOnline;
      _prev.ecuEnabled = s.ecuEnabled;
      _prev.simulationMode = s.simulationMode;
      _prev.rpm = s.rpm;
    }

    if (midDirty) {
      drawMidZone(s);
      _prev.speedKmh = s.speedKmh;
      _prev.afr = s.afr;
      _prev.engineTempC = s.engineTempC;
      _prev.batteryVolt = s.batteryVolt;
      _prev.fuelPercent = s.fuelPercent;
      _prev.throttlePct = s.throttlePct;
      _prev.mapKpa = s.mapKpa;
    }

    if (botDirty) {
      drawBottomZone(s);
      _prev.fuelInstantKmL = s.fuelInstantKmL;
      _prev.fuelAverageKmL = s.fuelAverageKmL;
      _prev.fuelLPer100Km = s.fuelLPer100Km;
      _prev.engineHealth = s.engineHealth;
    }

    if (_warningVisible && _warningBlinkState) {
      drawWarningOverlay(s, _activeWarning);
    } else if (!_warningVisible) {
      // nothing
    }

    _forceFullRedraw = false;
  }

  void showImmediateWarning(WarningType warning) {
    _activeWarning = warning;
    _warningVisible = (warning != WarningType::None);
    _forceFullRedraw = true;
  }

private:
  TFT_eSPI _tft;
  TFT_eSprite _topSprite;
  TFT_eSprite _midSprite;
  TFT_eSprite _botSprite;

  DashboardSnapshot _prev;
  WarningType _activeWarning;
  bool _warningBlinkState;
  bool _warningVisible;
  uint32_t _lastBlinkMs;
  uint32_t _lastFullRedrawMs;
  bool _forceFullRedraw;

  void runBootAnimation() {
    _tft.fillScreen(cfg::COL_BG);
    _tft.setTextColor(cfg::COL_ACCENT, cfg::COL_BG);
    _tft.setTextDatum(MC_DATUM);
    _tft.drawString("MOTOR ECU DASHBOARD", cfg::TFT_WIDTH / 2, cfg::TFT_HEIGHT / 2 - 30, 4);
    _tft.setTextColor(cfg::COL_GRAY, cfg::COL_BG);
    _tft.drawString("ESP32 + TFT_eSPI + K-Line scaffold", cfg::TFT_WIDTH / 2, cfg::TFT_HEIGHT / 2 - 2, 2);
    _tft.drawRoundRect(60, cfg::TFT_HEIGHT / 2 + 20, cfg::TFT_WIDTH - 120, 16, 6, cfg::COL_DARK_GRAY);

    const uint32_t start = millis();
    uint32_t lastFrame = 0;
    while ((millis() - start) < cfg::BOOT_ANIM_MS) {
      const uint32_t elapsed = millis() - start;
      if ((millis() - lastFrame) < 16) {
        yield();
        continue;
      }
      lastFrame = millis();

      float p = clampValue(static_cast<float>(elapsed) / cfg::BOOT_ANIM_MS, 0.0f, 1.0f);
      int fillW = static_cast<int>((cfg::TFT_WIDTH - 124) * p);
      _tft.fillRoundRect(62, cfg::TFT_HEIGHT / 2 + 22, fillW, 12, 5, cfg::COL_ACCENT2);
      _tft.fillCircle(62 + fillW, cfg::TFT_HEIGHT / 2 + 28, 5, cfg::COL_YELLOW);
      yield();
    }
    _tft.fillScreen(cfg::COL_BG);
  }

  void drawStaticFrame() {
    _tft.fillScreen(cfg::COL_BG);
    _tft.drawFastHLine(0, cfg::ZONE_TOP_H - 1, cfg::TFT_WIDTH, cfg::COL_DARK_GRAY);
    _tft.drawFastHLine(0, cfg::ZONE_TOP_H + cfg::ZONE_MID_H - 1, cfg::TFT_WIDTH, cfg::COL_DARK_GRAY);
    _tft.drawFastHLine(0, cfg::TFT_HEIGHT - 1, cfg::TFT_WIDTH, cfg::COL_DARK_GRAY);
  }

  void updateBlinkState() {
    const uint32_t now = millis();
    if ((now - _lastBlinkMs) >= 450) {
      _lastBlinkMs = now;
      _warningBlinkState = !_warningBlinkState;
      if (_warningVisible) _forceFullRedraw = true;
    }
  }

  WarningType evaluateWarning(const DashboardSnapshot& s) const {
    if (s.ecuEnabled && !s.ecuOnline) return WarningType::EcuDisconnected;
    if (s.batteryVolt > 0.1f && s.batteryVolt < 11.4f) return WarningType::BatteryLow;
    if (s.engineTempC > 108.0f) return WarningType::Overheat;
    if (s.afr > 17.0f) return WarningType::AfrLean;
    if (s.afr > 0.0f && s.afr < 12.0f) return WarningType::AfrRich;
    return WarningType::None;
  }

  uint16_t rpmColor(uint16_t rpm) const {
    if (rpm >= 10000) return cfg::COL_RED;
    if (rpm >= 8000)  return cfg::COL_ORANGE;
    if (rpm >= 6000)  return cfg::COL_YELLOW;
    return cfg::COL_GREEN;
  }

  uint16_t afrColor(float afr) const {
    if (afr < 12.2f || afr > 16.8f) return cfg::COL_RED;
    if (afr < 13.6f || afr > 15.4f) return cfg::COL_ORANGE;
    if (afr < 14.2f || afr > 15.1f) return cfg::COL_YELLOW;
    return cfg::COL_GREEN;
  }

  uint16_t tempColor(float t) const {
    if (t >= 110.0f) return cfg::COL_RED;
    if (t >= 95.0f) return cfg::COL_ORANGE;
    if (t < 40.0f) return cfg::COL_ACCENT;
    return cfg::COL_GREEN;
  }

  uint16_t voltColor(float v) const {
    if (v < 11.5f || v > 15.6f) return cfg::COL_RED;
    if (v < 12.2f) return cfg::COL_ORANGE;
    return cfg::COL_GREEN;
  }

  uint16_t fuelColor(float p) const {
    if (p < 10.0f) return cfg::COL_RED;
    if (p < 25.0f) return cfg::COL_ORANGE;
    if (p < 50.0f) return cfg::COL_YELLOW;
    return cfg::COL_GREEN;
  }

  uint16_t healthColor(float p) const {
    if (p < 40.0f) return cfg::COL_RED;
    if (p < 70.0f) return cfg::COL_YELLOW;
    return cfg::COL_GREEN;
  }

  const char* warningText(WarningType w) const {
    switch (w) {
      case WarningType::BatteryLow:     return "BATTERY LOW";
      case WarningType::Overheat:       return "OVERHEAT";
      case WarningType::AfrLean:        return "AFR LEAN";
      case WarningType::AfrRich:        return "AFR RICH";
      case WarningType::EcuDisconnected:return "ECU DISCONNECTED";
      case WarningType::SensorFault:    return "SENSOR FAULT";
      default:                          return "";
    }
  }

  void drawPanel(TFT_eSprite& sprite, int x, int y, int w, int h, const char* title, uint16_t accent) {
    sprite.fillRoundRect(x, y, w, h, 8, cfg::COL_PANEL);
    sprite.drawRoundRect(x, y, w, h, 8, accent);
    sprite.fillRect(x + 1, y + 1, w - 2, 8, cfg::COL_PANEL_2);
    sprite.setTextDatum(TL_DATUM);
    sprite.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL_2);
    sprite.drawString(title, x + 8, y + 2, 2);
  }

  void drawProgressBar(TFT_eSprite& sprite, int x, int y, int w, int h, float pct, uint16_t color) {
    pct = clampValue(pct, 0.0f, 100.0f);
    int fillW = static_cast<int>((w - 2) * pct / 100.0f);
    sprite.drawRoundRect(x, y, w, h, 3, cfg::COL_DARK_GRAY);
    if (fillW > 0) {
      sprite.fillRoundRect(x + 1, y + 1, fillW, h - 2, 3, color);
    }
  }

  void drawVerticalGauge(TFT_eSprite& sprite, int x, int y, int w, int h, float pct, uint16_t color) {
    pct = clampValue(pct, 0.0f, 100.0f);
    int fillH = static_cast<int>((h - 2) * pct / 100.0f);
    sprite.drawRoundRect(x, y, w, h, 4, cfg::COL_DARK_GRAY);
    if (fillH > 0) {
      sprite.fillRect(x + 1, y + h - 1 - fillH, w - 2, fillH, color);
    }
  }

  void drawTopZone(const DashboardSnapshot& s) {
    _topSprite.fillSprite(cfg::COL_BG);
    drawPanel(_topSprite, 0, 0, cfg::TFT_WIDTH, cfg::ZONE_TOP_H, "SYSTEM", cfg::COL_ACCENT);

    const char* mode = "LOCAL";
    if (s.simulationMode) mode = "SIM";
    else if (s.ecuEnabled && s.ecuOnline) mode = "ECU";
    else if (s.ecuEnabled && !s.ecuOnline) mode = "WAIT";

    _topSprite.setTextDatum(TL_DATUM);
    _topSprite.setTextColor(cfg::COL_WHITE, cfg::COL_PANEL);
    _topSprite.drawString("MODE:", 320, 6, 2);
    _topSprite.setTextColor(s.simulationMode ? cfg::COL_YELLOW : (s.ecuOnline ? cfg::COL_GREEN : cfg::COL_GRAY), cfg::COL_PANEL);
    _topSprite.drawString(mode, 370, 6, 2);

    char uptimeBuf[16];
    formatHMS(uptimeBuf, sizeof(uptimeBuf), millis() / 1000UL);
    _topSprite.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    _topSprite.drawString(uptimeBuf, 392, 24, 1);

    // RPM digits
    char rpmBuf[12];
    snprintf(rpmBuf, sizeof(rpmBuf), "%5u", static_cast<unsigned>(s.rpm));
    _topSprite.setTextColor(rpmColor(s.rpm), cfg::COL_PANEL);
    _topSprite.drawString(rpmBuf, 10, 26, 2);
    _topSprite.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    _topSprite.drawString("RPM", 92, 34, 1);

    // Shift lights
    const int ledCount = 10;
    const int ledW = 38;
    const int ledH = 8;
    const int ledGap = 4;
    const int ledY = 16;
    int totalW = ledCount * ledW + (ledCount - 1) * ledGap;
    int ledStartX = (cfg::TFT_WIDTH - totalW) / 2;
    for (int i = 0; i < ledCount; ++i) {
      int ledX = ledStartX + i * (ledW + ledGap);
      float threshold = (static_cast<float>(i + 1) / ledCount) * 12000.0f;
      bool active = s.rpm >= threshold;
      uint16_t col = cfg::COL_DARK_GRAY;
      if (active) {
        if (threshold < 7000.0f) col = cfg::COL_GREEN;
        else if (threshold < 9000.0f) col = cfg::COL_YELLOW;
        else col = cfg::COL_RED;
      }
      _topSprite.fillRoundRect(ledX, ledY, ledW, ledH, 3, col);
    }

    // RPM bar
    float rpmPct = clampValue(static_cast<float>(s.rpm) / 12000.0f * 100.0f, 0.0f, 100.0f);
    uint16_t barColor = rpmColor(s.rpm);
    drawProgressBar(_topSprite, 10, 44, cfg::TFT_WIDTH - 20, 10, rpmPct, barColor);

    _topSprite.drawFastHLine(0, cfg::ZONE_TOP_H - 1, cfg::TFT_WIDTH, cfg::COL_DARK_GRAY);
    _topSprite.pushSprite(0, 0);
  }

  void drawMidZone(const DashboardSnapshot& s) {
    _midSprite.fillSprite(cfg::COL_BG);

    drawPanel(_midSprite, 0,   0, 210, cfg::ZONE_MID_H, "SPEED", cfg::COL_ACCENT2);
    drawPanel(_midSprite, 210, 0, 135, 90, "AFR", afrColor(s.afr));
    drawPanel(_midSprite, 210, 90, 135, 90, "TEMP", tempColor(s.engineTempC));
    drawPanel(_midSprite, 345, 0, 135, 90, "BATT", voltColor(s.batteryVolt));
    drawPanel(_midSprite, 345, 90, 135, 90, "FUEL", fuelColor(s.fuelPercent));

    // Speed card
    _midSprite.setTextDatum(MC_DATUM);
    _midSprite.setTextColor(cfg::COL_WHITE, cfg::COL_PANEL);
    char speedBuf[12];
    snprintf(speedBuf, sizeof(speedBuf), "%3.0f", s.speedKmh);
    _midSprite.drawCentreString(speedBuf, 105, 82, 7);
    _midSprite.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    _midSprite.drawCentreString("km/h", 105, 142, 2);
    char distBuf[20];
    formatMeters(distBuf, sizeof(distBuf), s.distanceMeters);
    _midSprite.setTextColor(cfg::COL_MUTED, cfg::COL_PANEL);
    _midSprite.drawCentreString(distBuf, 105, 162, 2);

    // AFR card
    char afrBuf[12];
    snprintf(afrBuf, sizeof(afrBuf), "%.2f", s.afr);
    _midSprite.setTextColor(afrColor(s.afr), cfg::COL_PANEL);
    _midSprite.drawCentreString(afrBuf, 278, 42, 4);
    const char* afrText = (s.afr < 12.5f) ? "RICH" : (s.afr > 15.5f ? "LEAN" : "NORMAL");
    _midSprite.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    _midSprite.drawCentreString(afrText, 278, 70, 2);
    float lambda = s.afr / 14.7f;
    char lamBuf[16];
    snprintf(lamBuf, sizeof(lamBuf), "Lambda %.3f", lambda);
    _midSprite.setTextColor(cfg::COL_MUTED, cfg::COL_PANEL);
    _midSprite.drawCentreString(lamBuf, 278, 84, 1);
    drawProgressBar(_midSprite, 220, 76, 116, 8, clampValue((s.afr - 10.0f) / 10.0f * 100.0f, 0.0f, 100.0f), afrColor(s.afr));

    // Temp card
    char tempBuf[16];
    snprintf(tempBuf, sizeof(tempBuf), "%.1fC", s.engineTempC);
    _midSprite.setTextColor(tempColor(s.engineTempC), cfg::COL_PANEL);
    _midSprite.drawCentreString(tempBuf, 278, 120, 4);
    const char* tempText = (s.engineTempC < 40.0f) ? "COLD" : (s.engineTempC < 95.0f ? "NORMAL" : (s.engineTempC < 110.0f ? "HOT" : "OVERHEAT"));
    _midSprite.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    _midSprite.drawCentreString(tempText, 278, 152, 2);
    drawProgressBar(_midSprite, 220, 166, 116, 8, clampValue((s.engineTempC + 20.0f) / 140.0f * 100.0f, 0.0f, 100.0f), tempColor(s.engineTempC));

    // Battery card
    char battBuf[16];
    snprintf(battBuf, sizeof(battBuf), "%.2fV", s.batteryVolt);
    _midSprite.setTextColor(voltColor(s.batteryVolt), cfg::COL_PANEL);
    _midSprite.drawCentreString(battBuf, 412, 42, 4);
    _midSprite.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    _midSprite.drawCentreString((s.batteryVolt < 12.2f) ? "LOW" : (s.batteryVolt < 14.8f ? "CHG" : "HIGH"), 412, 70, 2);
    drawProgressBar(_midSprite, 355, 76, 116, 8, clampValue((s.batteryVolt - 11.0f) / 4.0f * 100.0f, 0.0f, 100.0f), voltColor(s.batteryVolt));

    // Fuel card
    char fuelBuf[16];
    snprintf(fuelBuf, sizeof(fuelBuf), "%2.0f%%", s.fuelPercent);
    _midSprite.setTextColor(fuelColor(s.fuelPercent), cfg::COL_PANEL);
    _midSprite.drawCentreString(fuelBuf, 412, 120, 4);
    _midSprite.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    _midSprite.drawCentreString("TANK", 412, 150, 2);
    drawVerticalGauge(_midSprite, 448, 108, 18, 60, s.fuelPercent, fuelColor(s.fuelPercent));

    // Sensor status chips
    drawSensorChip(_midSprite, 10, 170, "RPM", s.rpmState);
    drawSensorChip(_midSprite, 60, 170, "SPD", s.speedState);
    drawSensorChip(_midSprite, 110, 170, "AFR", s.afrState);
    drawSensorChip(_midSprite, 160, 170, "TMP", s.tempState);
    drawSensorChip(_midSprite, 210, 170, "BAT", s.battState);
    drawSensorChip(_midSprite, 260, 170, "FUL", s.fuelState);

    _midSprite.drawFastHLine(0, cfg::ZONE_MID_H - 1, cfg::TFT_WIDTH, cfg::COL_DARK_GRAY);
    _midSprite.pushSprite(0, cfg::ZONE_TOP_H);
  }

  void drawBottomZone(const DashboardSnapshot& s) {
    _botSprite.fillSprite(cfg::COL_BG);
    drawPanel(_botSprite, 0,   0, 240, cfg::ZONE_BOT_H, "FUEL ECONOMY", cfg::COL_ACCENT2);
    drawPanel(_botSprite, 240, 0, 240, cfg::ZONE_BOT_H, "ENGINE HEALTH", cfg::COL_ACCENT);

    // Fuel economy card
    char instBuf[18];
    char avgBuf[18];
    char l100Buf[18];
    snprintf(instBuf, sizeof(instBuf), "%.1f km/L", s.fuelInstantKmL);
    snprintf(avgBuf, sizeof(avgBuf),  "AVG %.1f", s.fuelAverageKmL);
    snprintf(l100Buf, sizeof(l100Buf), "%.1f L/100", s.fuelLPer100Km);

    _botSprite.setTextDatum(TL_DATUM);
    _botSprite.setTextColor(cfg::COL_ACCENT2, cfg::COL_PANEL);
    _botSprite.drawString(instBuf, 12, 28, 4);
    _botSprite.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    _botSprite.drawString(avgBuf, 14, 56, 2);
    _botSprite.setTextColor(cfg::COL_MUTED, cfg::COL_PANEL);
    _botSprite.drawString(l100Buf, 120, 56, 2);

    // Health card
    char healthBuf[16];
    snprintf(healthBuf, sizeof(healthBuf), "%3.0f%%", s.engineHealth);
    _botSprite.setTextColor(healthColor(s.engineHealth), cfg::COL_PANEL);
    _botSprite.drawString(healthBuf, 276, 24, 4);
    _botSprite.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    _botSprite.drawString(engineStatusString(s.engineHealth), 278, 54, 2);
    drawProgressBar(_botSprite, 270, 60, 180, 10, s.engineHealth, healthColor(s.engineHealth));

    _botSprite.drawFastHLine(0, cfg::ZONE_BOT_H - 1, cfg::TFT_WIDTH, cfg::COL_DARK_GRAY);
    _botSprite.pushSprite(0, cfg::ZONE_TOP_H + cfg::ZONE_MID_H);
  }

  void drawSensorChip(TFT_eSprite& sprite, int x, int y, const char* label, SensorState state) {
    uint16_t chipColor = sensorStateColor(state);
    sprite.fillRoundRect(x, y, 44, 16, 4, cfg::COL_PANEL_2);
    sprite.drawRoundRect(x, y, 44, 16, 4, chipColor);
    sprite.setTextDatum(MC_DATUM);
    sprite.setTextColor(chipColor, cfg::COL_PANEL_2);
    sprite.drawString(label, x + 22, y + 8, 1);
  }

  uint16_t sensorStateColor(SensorState s) const {
    switch (s) {
      case SensorState::Ok:      return cfg::COL_GREEN;
      case SensorState::Warning: return cfg::COL_YELLOW;
      case SensorState::Error:   return cfg::COL_ORANGE;
      case SensorState::Offline: return cfg::COL_RED;
      default:                   return cfg::COL_GRAY;
    }
  }

  const char* engineStatusString(float health) const {
    if (health >= 80.0f) return "EXCELLENT";
    if (health >= 60.0f) return "GOOD";
    if (health >= 40.0f) return "WARNING";
    return "CRITICAL";
  }

  void drawWarningOverlay(const DashboardSnapshot& s, WarningType warning) {
    if (!_warningBlinkState) return;

    const int w = 320;
    const int h = 76;
    const int x = (cfg::TFT_WIDTH - w) / 2;
    const int y = (cfg::TFT_HEIGHT - h) / 2;
    uint16_t color = cfg::COL_YELLOW;
    switch (warning) {
      case WarningType::BatteryLow:      color = cfg::COL_YELLOW; break;
      case WarningType::Overheat:        color = cfg::COL_RED; break;
      case WarningType::AfrLean:
      case WarningType::AfrRich:         color = cfg::COL_ORANGE; break;
      case WarningType::EcuDisconnected:  color = cfg::COL_RED; break;
      case WarningType::SensorFault:     color = cfg::COL_ORANGE; break;
      default:                           color = cfg::COL_YELLOW; break;
    }

    _tft.fillRoundRect(x, y, w, h, 8, cfg::COL_PANEL_2);
    _tft.drawRoundRect(x, y, w, h, 8, color);
    _tft.setTextDatum(MC_DATUM);
    _tft.setTextColor(color, cfg::COL_PANEL_2);
    _tft.drawString(warningText(warning), cfg::TFT_WIDTH / 2, cfg::TFT_HEIGHT / 2 - 8, 4);
    _tft.setTextColor(cfg::COL_WHITE, cfg::COL_PANEL_2);
    _tft.drawString("TAP OR CLEAR IN CODE TO DISMISS", cfg::TFT_WIDTH / 2, cfg::TFT_HEIGHT / 2 + 20, 2);

    if (s.ecuEnabled && !s.ecuOnline) {
      _tft.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL_2);
      _tft.drawString("ECU link watchdog active", cfg::TFT_WIDTH / 2, cfg::TFT_HEIGHT / 2 + 36, 1);
    }
  }
};

// =====================================================================
// Dashboard application orchestrator
// =====================================================================
class DashboardApp {
public:
  bool begin() {
    Serial.begin(115200);
    while (!Serial && (millis() < 2000)) {
      yield();
    }

    if (cfg::DEBUG_MODE) {
      Serial.println();
      Serial.println("==================================================");
      Serial.println("ESP32 Motorcycle Digital Dashboard");
      Serial.println("Production single-file base");
      Serial.println("==================================================");
    }

    initWatchdog();

    _sensorHub.setSimulationMode(cfg::SIMULATION_MODE);
    _sensorHub.setEcuEnabled(cfg::ECU_MANAGER_ENABLED);
    _sensorHub.begin();

    _ui.begin();

    _kline.begin(cfg::PIN_KLINE_RX, cfg::PIN_KLINE_TX, cfg::KLINE_BAUD);
    _kline.enable(cfg::ECU_MANAGER_ENABLED);
    _ecuRequests.begin(cfg::ECU_MANAGER_ENABLED);

    _fuelEstimator.reset();
    _healthModel.reset();

    _lastUiMs = 0;
    _lastDebugMs = 0;

    printBanner();
    return true;
  }

  void update() {
    feedWatchdog();

    _kline.update();
    _sensorHub.setEcuOnline(_kline.isConnected());
    _ecuRequests.update(_kline, _sensorHub);
    _sensorHub.update();

    DashboardSnapshot frame = _sensorHub.snapshot();
    _fuelEstimator.update(frame);
    _healthModel.update(frame);
    frame.fuelInstantKmL = _fuelEstimator.instantKmL();
    frame.fuelAverageKmL = _fuelEstimator.averageKmL();
    frame.fuelLPer100Km = _fuelEstimator.l100Km();
    frame.engineHealth = _healthModel.health();

    if ((millis() - _lastUiMs) >= cfg::UI_UPDATE_MS) {
      _lastUiMs = millis();
      _ui.update(frame);
      _lastFrame = frame;
    }

    if (cfg::DEBUG_MODE && (millis() - _lastDebugMs) >= cfg::DEBUG_PRINT_MS) {
      _lastDebugMs = millis();
      printDebug(frame);
    }
  }

private:
  SensorHub _sensorHub;
  FuelEstimator _fuelEstimator;
  EngineHealthModel _healthModel;
  KLineManager _kline;
  ECURequestHandler _ecuRequests;
  DashboardUI _ui;

  uint32_t _lastUiMs;
  uint32_t _lastDebugMs;
  DashboardSnapshot _lastFrame;

  void printBanner() {
    if (!cfg::DEBUG_MODE) return;
    Serial.printf("[CFG] SIMULATION_MODE=%s ECU_MANAGER_ENABLED=%s\n",
                  yesNo(cfg::SIMULATION_MODE), yesNo(cfg::ECU_MANAGER_ENABLED));
    Serial.printf("[CFG] TFT %ux%u, zone split = %u / %u / %u\n",
                  cfg::TFT_WIDTH, cfg::TFT_HEIGHT, cfg::ZONE_TOP_H, cfg::ZONE_MID_H, cfg::ZONE_BOT_H);
    Serial.printf("[CFG] K-Line RX=%u TX=%u BAUD=%lu\n",
                  cfg::PIN_KLINE_RX, cfg::PIN_KLINE_TX,
                  static_cast<unsigned long>(cfg::KLINE_BAUD));
    Serial.println("[SEC] WiFi credentials intentionally left blank; use provisioning / NVS.");
    Serial.println("[ECU] K-Line scaffold ready for future ECU request/parse work.");
  }

  void printDebug(const DashboardSnapshot& s) {
    Serial.printf("[DATA] SPD:%5.1f RPM:%5u AFR:%5.2f TMP:%5.1f BATT:%5.2f FUEL:%5.0f%% KM/L:%5.1f L/100:%4.1f HLTH:%5.0f ECU:%s/%s\n",
                  s.speedKmh,
                  static_cast<unsigned>(s.rpm),
                  s.afr,
                  s.engineTempC,
                  s.batteryVolt,
                  s.fuelPercent,
                  s.fuelInstantKmL,
                  s.fuelLPer100Km,
                  s.engineHealth,
                  yesNo(s.ecuEnabled),
                  yesNo(s.ecuOnline));
  }

  void initWatchdog() {
#if defined(ARDUINO_ARCH_ESP32)
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    esp_task_wdt_config_t wdtConfig;
    wdtConfig.timeout_ms = cfg::WDT_TIMEOUT_MS;
    wdtConfig.idle_core_mask = 0;
    wdtConfig.trigger_panic = true;
    esp_task_wdt_init(&wdtConfig);
  #else
    esp_task_wdt_init(cfg::WDT_TIMEOUT_MS / 1000, true);
  #endif
    esp_task_wdt_add(NULL);
#endif
  }

  void feedWatchdog() {
#if defined(ARDUINO_ARCH_ESP32)
    esp_task_wdt_reset();
#endif
  }
};

// =====================================================================
// Global application instance
// =====================================================================
static DashboardApp g_app;

void setup() {
  g_app.begin();
}

void loop() {
  g_app.update();
}
