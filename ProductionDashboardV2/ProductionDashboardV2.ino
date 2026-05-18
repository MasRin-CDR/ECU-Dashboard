/*
  ESP32 / ESP32-S3 Motorcycle ECU Dashboard v2

  Production-grade single-file Arduino sketch with:
  - Non-blocking sensor acquisition
  - Interrupt-driven RPM / wheel speed
  - Moving average and low-pass filtering
  - Dashboard, ECU Mapping, Diagnostic, and Sensor Monitor modes
  - K-Line / KWP2000 communication scaffold
  - Watchdog-friendly architecture
  - TFT_eSPI rendering prepared for 800x480 TFT

  Hardware protection notes:
  - RPM and injector pulse inputs must use optocoupler / isolation
  - Battery voltage must pass a resistor divider + protection
  - K-Line must use MC33290 / L9637 or equivalent transceiver
  - Use TVS diode, proper grounding, and automotive noise filtering
*/

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <string.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <esp_task_wdt.h>
#endif

// ============================================================
// Secure config placeholder
// ============================================================
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

// ============================================================
// Configuration
// ============================================================
namespace cfg {

constexpr bool DEBUG_MODE          = true;
constexpr bool SIMULATION_MODE     = false;
constexpr bool ECU_MANAGER_ENABLED = false;

constexpr uint16_t DISPLAY_W = 800;
constexpr uint16_t DISPLAY_H = 480;
constexpr uint8_t  DISPLAY_ROTATION = 1;

constexpr uint32_t SENSOR_FAST_MS  = 20;
constexpr uint32_t SENSOR_SLOW_MS  = 120;
constexpr uint32_t UI_DASH_MS      = 33;
constexpr uint32_t UI_TABLE_MS     = 160;
constexpr uint32_t UI_GRAPH_MS      = 100;
constexpr uint32_t DEBUG_MS        = 1000;
constexpr uint32_t BOOT_MS         = 900;
constexpr uint32_t PAGE_SWITCH_MS  = 4500;
constexpr uint32_t FULL_REDRAW_MS  = 5000;
constexpr uint32_t ECU_EXTERNAL_TIMEOUT_MS = 2000;

constexpr uint32_t RPM_MIN_EDGE_US   = 650;
constexpr uint32_t SPEED_MIN_EDGE_US = 2500;
constexpr uint32_t RPM_TIMEOUT_US    = 650000;
constexpr uint32_t SPEED_TIMEOUT_US  = 900000;

constexpr uint32_t KLINE_RX_TIMEOUT_MS = 500;
constexpr uint32_t KLINE_RETRY_MS      = 3000;
constexpr uint32_t KLINE_PROBE_MS      = 3000;
constexpr uint32_t KLINE_POLL_MS       = 20;
constexpr uint32_t KLINE_BAUD          = 10400;

constexpr uint32_t WDT_TIMEOUT_MS      = 8000;

// Pin map
constexpr int16_t PIN_RPM          = 34;  // input only, external pull-up required
constexpr int16_t PIN_SPEED        = 35;  // input only, external pull-up required
constexpr int16_t PIN_AFR_ADC      = 36;  // ADC1
constexpr int16_t PIN_TEMP_ADC     = 39;  // ADC1
constexpr int16_t PIN_BATT_ADC     = 32;  // ADC1
constexpr int16_t PIN_FUEL_ADC     = 33;  // ADC1
constexpr int16_t PIN_TPS_ADC      = 25;  // ADC2, move to ADC1/external ADC if Wi-Fi later enabled
constexpr int16_t PIN_MAP_ADC      = 26;  // ADC2, move to ADC1/external ADC if Wi-Fi later enabled

// Optional future inputs, intentionally left unused until hardware exists.
constexpr int16_t PIN_IAT_ADC      = -1;
constexpr int16_t PIN_EOT_ADC      = -1;
constexpr int16_t PIN_OIL_ADC      = -1;
constexpr int16_t PIN_KNOCK_ADC    = -1;
constexpr int16_t PIN_LEAN_ADC     = -1;
constexpr int16_t PIN_AMBIENT_ADC  = -1;
constexpr int16_t PIN_HUMIDITY_ADC = -1;
constexpr int16_t PIN_BARO_ADC     = -1;
constexpr int16_t PIN_FUEL_PUMP_FB = -1;
constexpr int16_t PIN_FAN_FB       = -1;
constexpr int16_t PIN_GEAR_IN      = -1;
constexpr int16_t PIN_INJECTOR_IN  = -1;
constexpr int16_t PIN_IGNITION_IN  = -1;

// K-Line UART
constexpr int16_t PIN_KLINE_RX = 16;
constexpr int16_t PIN_KLINE_TX = 17;

// TFT pins are usually set in TFT_eSPI User_Setup.
// Keep these values aligned with your board wiring and library setup.
constexpr int16_t PIN_TFT_CS   = 5;
constexpr int16_t PIN_TFT_DC   = 2;
constexpr int16_t PIN_TFT_RST  = 4;
constexpr int16_t PIN_TOUCH_CS = -1;
constexpr int16_t PIN_TOUCH_IRQ = -1;
constexpr int16_t PIN_MODE_BUTTON = 27; // physical button for mode cycling

// ADC calibration
constexpr float ADC_REF_VOLT = 3.30f;
constexpr float ADC_MAX_RAW  = 4095.0f;
constexpr float BATT_R1_OHM  = 100000.0f;
constexpr float BATT_R2_OHM  = 22000.0f;
constexpr float BATT_DIVIDER_RATIO = (BATT_R1_OHM + BATT_R2_OHM) / BATT_R2_OHM;
constexpr float AFR_DIVIDER_RATIO  = 2.447f;
constexpr float AFR_SENSOR_V_MIN   = 0.50f;
constexpr float AFR_SENSOR_V_MAX   = 4.50f;
constexpr float AFR_MIN_VALUE      = 10.0f;
constexpr float AFR_MAX_VALUE      = 20.0f;

// NTC thermistor
constexpr float TEMP_NTC_FIXED_OHM = 10000.0f;
constexpr float TEMP_NTC_R25_OHM   = 10000.0f;
constexpr float TEMP_NTC_BETA      = 3950.0f;
constexpr float TEMP_OFFSET_C      = 0.0f;

// Vehicle / estimation constants
constexpr float WHEEL_CIRCUMFERENCE_M = 1.720f;
constexpr float SPEED_PULSES_PER_REV  = 1.0f;
constexpr float RPM_PULSES_PER_REV    = 1.0f;
constexpr float SPEED_CAL_FACTOR      = 1.0f;
constexpr float RPM_CAL_FACTOR        = 1.0f;

constexpr float INJECTOR_FLOW_CC_MIN = 125.0f;
constexpr float TANK_CAPACITY_L      = 3.5f;

// UI palette
constexpr uint16_t COL_BG        = 0x0410;
constexpr uint16_t COL_PANEL     = 0x0C10;
constexpr uint16_t COL_PANEL_2   = 0x1021;
constexpr uint16_t COL_ACCENT    = 0x07FF;
constexpr uint16_t COL_ACCENT2   = 0x051F;
constexpr uint16_t COL_GREEN     = 0x07E0;
constexpr uint16_t COL_GREEN_DIM = 0x03A0;
constexpr uint16_t COL_YELLOW    = 0xFFE0;
constexpr uint16_t COL_ORANGE    = 0xFC20;
constexpr uint16_t COL_RED       = 0xF800;
constexpr uint16_t COL_WHITE     = 0xFFFF;
constexpr uint16_t COL_GRAY      = 0xBDF7;
constexpr uint16_t COL_DARK      = 0x2945;
constexpr uint16_t COL_MUTED     = 0x6B4D;

// Engine health weights
constexpr float HEALTH_W_TEMP = 0.30f;
constexpr float HEALTH_W_AFR  = 0.25f;
constexpr float HEALTH_W_VOLT = 0.20f;
constexpr float HEALTH_W_RPM  = 0.15f;
constexpr float HEALTH_W_FUEL  = 0.10f;

// KWP2000 / OBD placeholder IDs
constexpr uint8_t OBD_MODE_CURRENT = 0x01;
constexpr uint8_t PID_SUPPORTED_01 = 0x00;
constexpr uint8_t PID_ENGINE_RPM   = 0x0C;
constexpr uint8_t PID_SPEED        = 0x0D;
constexpr uint8_t PID_COOLANT_TEMP = 0x05;
constexpr uint8_t PID_THROTTLE_POS = 0x11;
constexpr uint8_t PID_O2_SENSOR_1  = 0x14;
constexpr uint8_t PID_MAP          = 0x0B;
constexpr uint8_t PID_IAT          = 0x0F;
constexpr uint8_t PID_BATTERY_VOLT = 0x42;
constexpr uint8_t PID_STFT         = 0x06;
constexpr uint8_t PID_LTFT         = 0x07;
constexpr uint8_t PID_DTC          = 0x03;

constexpr uint8_t ECU_ADDR_DEFAULT    = 0x6A;
constexpr uint8_t TESTER_ADDR_DEFAULT  = 0xF1;
constexpr uint8_t KLINE_HEADER_DEFAULT = 0x68;

constexpr size_t MAX_ECU_PAYLOAD = 48;
constexpr size_t MAX_ECU_FRAME    = 64;

} // namespace cfg

// ============================================================
// Utility helpers
// ============================================================
template <typename T>
static inline T clampValue(T v, T lo, T hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  if (fabsf(inMax - inMin) < 0.0001f) return outMin;
  float t = (x - inMin) / (inMax - inMin);
  return outMin + (t * (outMax - outMin));
}

template <typename T>
static inline T lowPassFilter(T previous, T input, float alpha) {
  alpha = clampValue(alpha, 0.0f, 1.0f);
  return previous + (input - previous) * alpha;
}

template <typename T>
static inline T movingAverage(const T* values, size_t count) {
  if (count == 0) return T{};
  T sum = T{};
  for (size_t i = 0; i < count; ++i) sum += values[i];
  return sum / static_cast<T>(count);
}

struct DebounceTracker {
  bool stableState = false;
  bool lastRaw = false;
  uint32_t lastChangeMs = 0;
};

static inline bool debounceFilter(bool raw, DebounceTracker& tracker, uint32_t now, uint32_t debounceMs) {
  if (raw != tracker.lastRaw) {
    tracker.lastRaw = raw;
    tracker.lastChangeMs = now;
  }

  if ((now - tracker.lastChangeMs) < debounceMs) return false;

  if (tracker.stableState != tracker.lastRaw) {
    tracker.stableState = tracker.lastRaw;
    return true;
  }
  return false;
}

static inline bool isFiniteNumber(float v) {
  return !isnan(v) && !isinf(v);
}

static inline const char* yesNo(bool v) { return v ? "ON" : "OFF"; }

static inline bool pinActive(int16_t pin) { return pin >= 0; }

static void formatDuration(char* out, size_t size, uint32_t ms) {
  if (ms < 1000UL) {
    snprintf(out, size, "%lums", static_cast<unsigned long>(ms));
    return;
  }
  if (ms < 60000UL) {
    snprintf(out, size, "%.1fs", ms / 1000.0f);
    return;
  }
  uint32_t min = ms / 60000UL;
  uint32_t sec = (ms / 1000UL) % 60UL;
  snprintf(out, size, "%lum%02lus", static_cast<unsigned long>(min), static_cast<unsigned long>(sec));
}

static void formatDistance(char* out, size_t size, float meters) {
  if (meters >= 1000.0f) {
    snprintf(out, size, "%.2f km", meters / 1000.0f);
  } else {
    snprintf(out, size, "%.0f m", meters);
  }
}

static void formatUptime(char* out, size_t size, uint32_t seconds) {
  uint32_t h = seconds / 3600UL;
  uint32_t m = (seconds / 60UL) % 60UL;
  uint32_t s = seconds % 60UL;
  snprintf(out, size, "%02lu:%02lu:%02lu",
           static_cast<unsigned long>(h),
           static_cast<unsigned long>(m),
           static_cast<unsigned long>(s));
}

enum class UiMode : uint8_t;
static const char* uiModeText(UiMode mode);

// ============================================================
// Enums and data
// ============================================================
enum class SensorStatus : uint8_t {
  Ok = 0,
  Warning,
  Error,
  Offline
};

enum class SensorSource : uint8_t {
  Hardware = 0,
  Ecu,
  Estimated,
  Simulation,
  Offline
};

enum class UiMode : uint8_t {
  Dashboard = 0,
  EcuMapping,
  Diagnostic,
  SensorMonitor
};

enum class WarningLevel : uint8_t {
  None = 0,
  Info,
  Warning,
  Critical
};

enum class WarningReason : uint8_t {
  None = 0,
  EcuTimeout,
  BatteryLow,
  Overheat,
  AfrLean,
  AfrRich,
  SensorFault
};

enum class EcuState : uint8_t {
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

enum class SensorId : uint8_t {
  Speed = 0,
  Rpm,
  Afr,
  EngineTemp,
  Battery,
  FuelLevel,
  Tps,
  Map,
  Iat,
  Eot,
  InjectorPulse,
  IgnitionTiming,
  Ckp,
  FuelPump,
  RadiatorFan,
  GearPosition,
  LeanAngle,
  OilPressure,
  Knock,
  AmbientTemp,
  Humidity,
  BarometricPressure,
  DtcCode,
  Count
};

static constexpr size_t SENSOR_COUNT = static_cast<size_t>(SensorId::Count);

struct ReadResult {
  float raw = 0.0f;
  float value = 0.0f;
  bool valid = false;
  SensorStatus status = SensorStatus::Offline;
  SensorSource source = SensorSource::Offline;
};

struct SensorSample {
  float raw = 0.0f;
  float converted = 0.0f;
  float filtered = 0.0f;
  float avg = 0.0f;
  float min = 0.0f;
  float max = 0.0f;
  float lastGood = 0.0f;
  uint32_t lastUpdateMs = 0;
  uint32_t lastGoodMs = 0;
  uint32_t timeoutMs = 1000;
  uint32_t errorCount = 0;
  SensorStatus status = SensorStatus::Offline;
  SensorSource source = SensorSource::Offline;
  bool valid = false;
  bool hasValue = false;
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
  float iatC = 25.0f;
  float eotC = 25.0f;
  float injectorPulseMs = 0.0f;
  float ignitionTimingDeg = 0.0f;
  float ckpHz = 0.0f;
  float leanAngleDeg = 0.0f;
  float oilPressureBar = 0.0f;
  float knockLevel = 0.0f;
  float ambientTempC = 25.0f;
  float humidityPct = 50.0f;
  float baroKpa = 101.3f;
  float gearPosition = 0.0f;
  bool fuelPumpOn = false;
  bool radiatorFanOn = false;
  uint16_t dtcCode = 0;
  bool ecuEnabled = false;
  bool ecuOnline = false;
  EcuState ecuState = EcuState::Disabled;
  UiMode mode = UiMode::Dashboard;
  WarningLevel warningLevel = WarningLevel::None;
  WarningReason warningReason = WarningReason::None;
  bool engineRunning = false;
  uint32_t timestampMs = 0;
};

static constexpr const char* SENSOR_NAMES[SENSOR_COUNT] = {
  "SPEED",
  "RPM",
  "AFR",
  "ENGINE TEMP",
  "BATT VOLT",
  "FUEL LEVEL",
  "TPS",
  "MAP",
  "IAT",
  "EOT",
  "INJ PULSE",
  "IGN TIMING",
  "CKP",
  "FUEL PUMP",
  "RAD FAN",
  "GEAR",
  "LEAN ANG",
  "OIL PRES",
  "KNOCK",
  "AMBIENT",
  "HUMIDITY",
  "BARO",
  "DTC"
};

static constexpr const char* SENSOR_UNITS[SENSOR_COUNT] = {
  "km/h", "rpm", "AFR", "C", "V", "%", "%", "kPa", "C", "C",
  "ms", "deg", "Hz", "", "", "", "deg", "bar", "lvl", "C",
  "%", "kPa", ""
};

static constexpr uint32_t SENSOR_TIMEOUTS[SENSOR_COUNT] = {
  cfg::SPEED_TIMEOUT_US / 1000UL,
  cfg::RPM_TIMEOUT_US / 1000UL,
  1200UL,
  2000UL,
  2000UL,
  3000UL,
  1000UL,
  1000UL,
  2000UL,
  2000UL,
  1000UL,
  1000UL,
  1000UL,
  1000UL,
  1000UL,
  1000UL,
  1000UL,
  1000UL,
  1000UL,
  3000UL,
  3000UL,
  3000UL,
  5000UL
};

static const char* sensorStatusText(SensorStatus s) {
  switch (s) {
    case SensorStatus::Ok:      return "OK";
    case SensorStatus::Warning:  return "WARNING";
    case SensorStatus::Error:    return "ERROR";
    case SensorStatus::Offline:  return "OFF";
    default:                     return "?";
  }
}

static uint16_t sensorStatusColor(SensorStatus s) {
  switch (s) {
    case SensorStatus::Ok:      return cfg::COL_GREEN;
    case SensorStatus::Warning:  return cfg::COL_YELLOW;
    case SensorStatus::Error:    return cfg::COL_ORANGE;
    case SensorStatus::Offline:  return cfg::COL_RED;
    default:                     return cfg::COL_GRAY;
  }
}

static const char* sensorSourceText(SensorSource s) {
  switch (s) {
    case SensorSource::Hardware:   return "HW";
    case SensorSource::Ecu:        return "ECU";
    case SensorSource::Estimated:  return "EST";
    case SensorSource::Simulation: return "SIM";
    case SensorSource::Offline:    return "OFF";
    default:                       return "?";
  }
}

static const char* warningReasonText(WarningReason w) {
  switch (w) {
    case WarningReason::EcuTimeout: return "ECU TIMEOUT";
    case WarningReason::BatteryLow:  return "BATTERY LOW";
    case WarningReason::Overheat:    return "OVERHEAT";
    case WarningReason::AfrLean:     return "AFR LEAN";
    case WarningReason::AfrRich:     return "AFR RICH";
    case WarningReason::SensorFault:  return "SENSOR FAULT";
    default:                         return "SYSTEM OK";
  }
}

static uint16_t warningColor(WarningLevel level, WarningReason reason) {
  switch (level) {
    case WarningLevel::Info:      return cfg::COL_ACCENT;
    case WarningLevel::Warning:   return (reason == WarningReason::AfrLean || reason == WarningReason::AfrRich) ? cfg::COL_ORANGE : cfg::COL_YELLOW;
    case WarningLevel::Critical:  return cfg::COL_RED;
    default:                      return cfg::COL_GREEN;
  }
}

static const char* ecuStateText(EcuState s) {
  switch (s) {
    case EcuState::Disabled:        return "DISABLED";
    case EcuState::Idle:            return "IDLE";
    case EcuState::Probing:         return "PROBING";
    case EcuState::Connected:       return "CONNECTED";
    case EcuState::Requesting:      return "REQUESTING";
    case EcuState::WaitingResponse: return "WAITING";
    case EcuState::Error:           return "ERROR";
    case EcuState::Reconnecting:    return "RECONNECTING";
    default:                        return "UNKNOWN";
  }
}

static const char* uiModeText(UiMode mode) {
  switch (mode) {
    case UiMode::Dashboard:     return "DASHBOARD";
    case UiMode::EcuMapping:    return "ECU MAPPING";
    case UiMode::Diagnostic:    return "DIAGNOSTIC";
    case UiMode::SensorMonitor: return "SENSOR MON";
    default:                    return "UNKNOWN";
  }
}

static void formatSensorValue(SensorId id, const SensorSample& s, char* out, size_t size) {
  const char* unit = SENSOR_UNITS[static_cast<size_t>(id)];
  switch (id) {
    case SensorId::FuelPump:
    case SensorId::RadiatorFan:
      snprintf(out, size, "%s", (s.converted > 0.5f) ? "ON" : "OFF");
      break;
    case SensorId::DtcCode:
      if (static_cast<uint16_t>(s.converted) == 0) snprintf(out, size, "NONE");
      else snprintf(out, size, "0x%04X", static_cast<unsigned>(static_cast<uint16_t>(s.converted)));
      break;
    case SensorId::GearPosition:
      if (s.status == SensorStatus::Offline) snprintf(out, size, "--");
      else if (s.converted < 0.5f) snprintf(out, size, "N");
      else snprintf(out, size, "%.0f", s.converted);
      break;
    default:
      if (strlen(unit) == 0) {
        snprintf(out, size, "%.1f", s.filtered);
      } else if (id == SensorId::Afr) {
        snprintf(out, size, "%.2f %s", s.filtered, unit);
      } else if (id == SensorId::Battery || id == SensorId::FuelLevel || id == SensorId::Tps) {
        snprintf(out, size, "%.1f %s", s.filtered, unit);
      } else if (id == SensorId::Rpm) {
        snprintf(out, size, "%.0f %s", s.filtered, unit);
      } else if (id == SensorId::InjectorPulse || id == SensorId::IgnitionTiming || id == SensorId::LeanAngle || id == SensorId::OilPressure) {
        snprintf(out, size, "%.2f %s", s.filtered, unit);
      } else {
        snprintf(out, size, "%.1f %s", s.filtered, unit);
      }
      break;
  }
}

static void formatRawValue(SensorId id, const SensorSample& s, char* out, size_t size) {
  switch (id) {
    case SensorId::DtcCode:
      if (static_cast<uint16_t>(s.raw) == 0) snprintf(out, size, "NONE");
      else snprintf(out, size, "0x%04X", static_cast<unsigned>(static_cast<uint16_t>(s.raw)));
      break;
    case SensorId::FuelPump:
    case SensorId::RadiatorFan:
      snprintf(out, size, "%s", (s.raw > 0.5f) ? "1" : "0");
      break;
    default:
      snprintf(out, size, "%.1f", s.raw);
      break;
  }
}

static void formatAge(char* out, size_t size, uint32_t ageMs) {
  if (ageMs < 1000UL) {
    snprintf(out, size, "%lu ms", static_cast<unsigned long>(ageMs));
  } else {
    snprintf(out, size, "%.1f s", ageMs / 1000.0f);
  }
}

// ============================================================
// Ring history helper
// ============================================================
template <typename T, size_t N>
class HistoryWindow {
public:
  HistoryWindow() { reset(); }

  void reset() {
    _count = 0;
    _head = 0;
    for (size_t i = 0; i < N; ++i) _values[i] = T{};
  }

  void push(T value) {
    _values[_head] = value;
    _head = (_head + 1) % N;
    if (_count < N) ++_count;
  }

  size_t count() const { return _count; }
  const T* data() const { return _values; }

  T at(size_t index) const {
    if (_count == 0 || index >= _count) return T{};
    size_t start = (_head + N - _count) % N;
    return _values[(start + index) % N];
  }

  T latest() const {
    if (_count == 0) return T{};
    return at(_count - 1);
  }

  T mean() const {
    return movingAverage(_values, _count);
  }

  float stddev() const {
    if (_count < 2) return 0.0f;
    float m = static_cast<float>(mean());
    float acc = 0.0f;
    for (size_t i = 0; i < _count; ++i) {
      float d = static_cast<float>(at(i)) - m;
      acc += d * d;
    }
    return sqrtf(acc / static_cast<float>(_count));
  }

private:
  T _values[N];
  size_t _count;
  size_t _head;
};

// ============================================================
// ADC helper
// ============================================================
class AnalogSampler {
public:
  static uint16_t averageRaw(int16_t pin, uint8_t samples = 6) {
    if (!pinActive(pin)) return 0;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; ++i) {
      sum += analogRead(static_cast<uint8_t>(pin));
    }
    return static_cast<uint16_t>(sum / samples);
  }

  static float rawToVoltage(uint16_t raw) {
    return (static_cast<float>(raw) / cfg::ADC_MAX_RAW) * cfg::ADC_REF_VOLT;
  }
};

// ============================================================
// Sensor hub
// ============================================================
class SensorHub {
public:
  SensorHub()
    : _simulationMode(cfg::SIMULATION_MODE),
      _ecuEnabled(cfg::ECU_MANAGER_ENABLED),
      _ecuOnline(false),
      _lastFastMs(0),
      _lastSlowMs(0),
      _lastSimMs(0),
      _lastDistanceMs(0),
      _lastRpmCalcMs(0),
      _lastSpeedCalcMs(0),
      _prevRpmCount(0),
      _prevSpeedCount(0),
      _distanceMeters(0.0f),
      _simPhase(0.0f) {
    for (size_t i = 0; i < SENSOR_COUNT; ++i) {
      _samples[i].timeoutMs = SENSOR_TIMEOUTS[i];
    }
  }

  bool begin() {
    analogReadResolution(12);

    if (pinActive(cfg::PIN_AFR_ADC)) analogSetPinAttenuation(static_cast<uint8_t>(cfg::PIN_AFR_ADC), ADC_11db);
    if (pinActive(cfg::PIN_TEMP_ADC)) analogSetPinAttenuation(static_cast<uint8_t>(cfg::PIN_TEMP_ADC), ADC_11db);
    if (pinActive(cfg::PIN_BATT_ADC)) analogSetPinAttenuation(static_cast<uint8_t>(cfg::PIN_BATT_ADC), ADC_11db);
    if (pinActive(cfg::PIN_FUEL_ADC)) analogSetPinAttenuation(static_cast<uint8_t>(cfg::PIN_FUEL_ADC), ADC_11db);
    if (pinActive(cfg::PIN_TPS_ADC)) analogSetPinAttenuation(static_cast<uint8_t>(cfg::PIN_TPS_ADC), ADC_11db);
    if (pinActive(cfg::PIN_MAP_ADC)) analogSetPinAttenuation(static_cast<uint8_t>(cfg::PIN_MAP_ADC), ADC_11db);

    if (pinActive(cfg::PIN_RPM)) {
      pinMode(static_cast<uint8_t>(cfg::PIN_RPM), INPUT);
      attachInterrupt(digitalPinToInterrupt(static_cast<uint8_t>(cfg::PIN_RPM)), isrRpm, FALLING);
    }
    if (pinActive(cfg::PIN_SPEED)) {
      pinMode(static_cast<uint8_t>(cfg::PIN_SPEED), INPUT);
      attachInterrupt(digitalPinToInterrupt(static_cast<uint8_t>(cfg::PIN_SPEED)), isrSpeed, FALLING);
    }

    if (cfg::DEBUG_MODE) {
      Serial.println("[SENSOR] ADC, interrupts, and filter chain initialized");
    }
    return true;
  }

  void setSimulationMode(bool enabled) { _simulationMode = enabled; }
  void setEcuEnabled(bool enabled) { _ecuEnabled = enabled; }
  void setEcuOnline(bool online) { _ecuOnline = online; }

  bool ecuEnabled() const { return _ecuEnabled; }
  bool ecuOnline() const { return _ecuOnline; }
  bool simulationMode() const { return _simulationMode; }

  const DashboardSnapshot& snapshot() const { return _snapshot; }
  const SensorSample& sample(SensorId id) const { return _samples[indexOf(id)]; }
  size_t sensorCount() const { return SENSOR_COUNT; }
  const char* sensorName(SensorId id) const { return SENSOR_NAMES[indexOf(id)]; }
  const char* sensorUnit(SensorId id) const { return SENSOR_UNITS[indexOf(id)]; }

  void injectSensor(SensorId id, float value, bool valid = true, SensorSource source = SensorSource::Ecu) {
    const size_t idx = indexOf(id);
    _external[idx].value = value;
    _external[idx].valid = valid;
    _external[idx].lastMs = millis();
    _external[idx].source = source;
  }

  void injectSpeed(float value, bool valid = true) { injectSensor(SensorId::Speed, value, valid, SensorSource::Ecu); }
  void injectRpm(float value, bool valid = true) { injectSensor(SensorId::Rpm, value, valid, SensorSource::Ecu); }
  void injectAfr(float value, bool valid = true) { injectSensor(SensorId::Afr, value, valid, SensorSource::Ecu); }
  void injectEngineTemp(float value, bool valid = true) { injectSensor(SensorId::EngineTemp, value, valid, SensorSource::Ecu); }
  void injectBattery(float value, bool valid = true) { injectSensor(SensorId::Battery, value, valid, SensorSource::Ecu); }
  void injectFuel(float value, bool valid = true) { injectSensor(SensorId::FuelLevel, value, valid, SensorSource::Ecu); }
  void injectTps(float value, bool valid = true) { injectSensor(SensorId::Tps, value, valid, SensorSource::Ecu); }
  void injectMap(float value, bool valid = true) { injectSensor(SensorId::Map, value, valid, SensorSource::Ecu); }
  void injectIat(float value, bool valid = true) { injectSensor(SensorId::Iat, value, valid, SensorSource::Ecu); }
  void injectEot(float value, bool valid = true) { injectSensor(SensorId::Eot, value, valid, SensorSource::Ecu); }
  void injectInjectorPulse(float value, bool valid = true, SensorSource source = SensorSource::Ecu) { injectSensor(SensorId::InjectorPulse, value, valid, source); }
  void injectIgnitionTiming(float value, bool valid = true, SensorSource source = SensorSource::Ecu) { injectSensor(SensorId::IgnitionTiming, value, valid, source); }
  void injectCkp(float value, bool valid = true, SensorSource source = SensorSource::Ecu) { injectSensor(SensorId::Ckp, value, valid, source); }
  void injectFuelPump(bool on, bool valid = true, SensorSource source = SensorSource::Ecu) { injectSensor(SensorId::FuelPump, on ? 1.0f : 0.0f, valid, source); }
  void injectRadiatorFan(bool on, bool valid = true, SensorSource source = SensorSource::Ecu) { injectSensor(SensorId::RadiatorFan, on ? 1.0f : 0.0f, valid, source); }
  void injectGear(float gear, bool valid = true, SensorSource source = SensorSource::Ecu) { injectSensor(SensorId::GearPosition, gear, valid, source); }
  void injectLeanAngle(float angle, bool valid = true, SensorSource source = SensorSource::Ecu) { injectSensor(SensorId::LeanAngle, angle, valid, source); }
  void injectOilPressure(float bar, bool valid = true, SensorSource source = SensorSource::Ecu) { injectSensor(SensorId::OilPressure, bar, valid, source); }
  void injectKnock(float knock, bool valid = true, SensorSource source = SensorSource::Ecu) { injectSensor(SensorId::Knock, knock, valid, source); }
  void injectAmbientTemp(float c, bool valid = true, SensorSource source = SensorSource::Ecu) { injectSensor(SensorId::AmbientTemp, c, valid, source); }
  void injectHumidity(float pct, bool valid = true, SensorSource source = SensorSource::Ecu) { injectSensor(SensorId::Humidity, pct, valid, source); }
  void injectBarometricPressure(float kpa, bool valid = true, SensorSource source = SensorSource::Ecu) { injectSensor(SensorId::BarometricPressure, kpa, valid, source); }
  void injectDtc(uint16_t code, bool valid = true, SensorSource source = SensorSource::Ecu) { injectSensor(SensorId::DtcCode, static_cast<float>(code), valid, source); }

  // Required API names.
  ReadResult readSpeed() { return readSpeed(millis()); }
  ReadResult readRPM(uint32_t now) { return readRpm(now); }
  ReadResult readRPM() { return readRpm(millis()); }
  ReadResult readAFR() { return readAfr(); }
  ReadResult readTPS() { return readTps(); }
  ReadResult readMAP() { return readMap(); }
  ReadResult readIAT() { return readIat(); }
  ReadResult readInjectorPulse() { return readInjectorPulse(millis()); }

  void update() {
    const uint32_t now = millis();

    if (_simulationMode) {
      updateSimulation(now);
      buildSnapshot(now);
      return;
    }

    if ((now - _lastFastMs) >= cfg::SENSOR_FAST_MS) {
      _lastFastMs = now;
      updateFastSensors(now);
    }
    if ((now - _lastSlowMs) >= cfg::SENSOR_SLOW_MS) {
      _lastSlowMs = now;
      updateSlowSensors(now);
    }

    updateTimeouts(now);
    updateDistance(now);
    buildSnapshot(now);
  }

private:
  struct ExternalFeed {
    float value = 0.0f;
    bool valid = false;
    uint32_t lastMs = 0;
    SensorSource source = SensorSource::Ecu;
  };

  static volatile uint32_t s_rpmPulseCount;
  static volatile uint32_t s_rpmLastEdgeUs;
  static volatile uint32_t s_speedPulseCount;
  static volatile uint32_t s_speedLastEdgeUs;

  bool _simulationMode;
  bool _ecuEnabled;
  bool _ecuOnline;

  uint32_t _lastFastMs;
  uint32_t _lastSlowMs;
  uint32_t _lastSimMs;
  uint32_t _lastDistanceMs;
  uint32_t _lastRpmCalcMs;
  uint32_t _lastSpeedCalcMs;
  uint32_t _prevRpmCount;
  uint32_t _prevSpeedCount;
  float _distanceMeters;
  float _simPhase;

  SensorSample _samples[SENSOR_COUNT];
  HistoryWindow<float, 8> _history[SENSOR_COUNT];
  ExternalFeed _external[SENSOR_COUNT];
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

  static size_t indexOf(SensorId id) { return static_cast<size_t>(id); }

  ReadResult readSpeed(uint32_t now) {
    ReadResult rr;
    rr.source = SensorSource::Hardware;
    noInterrupts();
    uint32_t pulseCount = s_speedPulseCount;
    uint32_t lastEdgeUs = s_speedLastEdgeUs;
    interrupts();

    if (_lastSpeedCalcMs == 0) {
      _lastSpeedCalcMs = now;
      rr.valid = false;
      rr.value = _samples[indexOf(SensorId::Speed)].lastGood;
      rr.status = SensorStatus::Offline;
      return rr;
    }

    const uint32_t elapsedMs = now - _lastSpeedCalcMs;
    _lastSpeedCalcMs = now;

    if ((micros() - lastEdgeUs) > cfg::SPEED_TIMEOUT_US) {
      rr.valid = false;
      rr.status = SensorStatus::Offline;
      rr.value = _samples[indexOf(SensorId::Speed)].lastGood;
      return rr;
    }

    noInterrupts();
    uint32_t delta = pulseCount - _prevSpeedCount;
    _prevSpeedCount = pulseCount;
    interrupts();

    float rotations = static_cast<float>(delta) / cfg::SPEED_PULSES_PER_REV;
    float meters = rotations * cfg::WHEEL_CIRCUMFERENCE_M;
    float kmh = (elapsedMs > 0) ? ((meters / (elapsedMs / 1000.0f)) * 3.6f * cfg::SPEED_CAL_FACTOR) : 0.0f;
    kmh = clampValue(kmh, 0.0f, 220.0f);

    rr.raw = static_cast<float>(delta);
    rr.value = kmh;
    rr.valid = true;
    rr.status = SensorStatus::Ok;
    return rr;
  }

  ReadResult readRpm(uint32_t now) {
    ReadResult rr;
    rr.source = SensorSource::Hardware;
    noInterrupts();
    uint32_t pulseCount = s_rpmPulseCount;
    uint32_t lastEdgeUs = s_rpmLastEdgeUs;
    interrupts();

    if (_lastRpmCalcMs == 0) {
      _lastRpmCalcMs = now;
      _prevRpmCount = pulseCount;
      rr.valid = false;
      rr.value = _samples[indexOf(SensorId::Rpm)].lastGood;
      rr.status = SensorStatus::Offline;
      return rr;
    }

    const uint32_t elapsedMs = now - _lastRpmCalcMs;
    _lastRpmCalcMs = now;

    if ((micros() - lastEdgeUs) > cfg::RPM_TIMEOUT_US) {
      rr.valid = false;
      rr.status = SensorStatus::Offline;
      rr.value = _samples[indexOf(SensorId::Rpm)].lastGood;
      return rr;
    }

    noInterrupts();
    uint32_t delta = pulseCount - _prevRpmCount;
    _prevRpmCount = pulseCount;
    interrupts();

    float rpm = (elapsedMs > 0) ? ((static_cast<float>(delta) / cfg::RPM_PULSES_PER_REV) * (60000.0f / elapsedMs) * cfg::RPM_CAL_FACTOR) : 0.0f;
    rpm = clampValue(rpm, 0.0f, 14000.0f);

    rr.raw = static_cast<float>(delta);
    rr.value = rpm;
    rr.valid = true;
    rr.status = SensorStatus::Ok;
    return rr;
  }

  ReadResult readAfr() {
    ReadResult rr;
    rr.source = SensorSource::Hardware;
    if (!pinActive(cfg::PIN_AFR_ADC)) return rr;

    uint16_t raw = AnalogSampler::averageRaw(cfg::PIN_AFR_ADC, 6);
    float adcVolt = AnalogSampler::rawToVoltage(raw);
    float sensorVolt = adcVolt * cfg::AFR_DIVIDER_RATIO;

    rr.raw = sensorVolt;
    if (sensorVolt < 0.20f || sensorVolt > 5.20f) {
      rr.valid = false;
      rr.status = SensorStatus::Error;
      rr.value = _samples[indexOf(SensorId::Afr)].lastGood;
      return rr;
    }

    float afr = mapFloat(sensorVolt, cfg::AFR_SENSOR_V_MIN, cfg::AFR_SENSOR_V_MAX, cfg::AFR_MIN_VALUE, cfg::AFR_MAX_VALUE);
    afr = clampValue(afr, 8.0f, 22.0f);
    rr.value = afr;
    rr.valid = true;
    rr.status = (sensorVolt < (cfg::AFR_SENSOR_V_MIN + 0.12f) || sensorVolt > (cfg::AFR_SENSOR_V_MAX - 0.12f)) ? SensorStatus::Warning : SensorStatus::Ok;
    return rr;
  }

  ReadResult readEngineTemp() {
    ReadResult rr;
    rr.source = SensorSource::Hardware;
    if (!pinActive(cfg::PIN_TEMP_ADC)) return rr;

    uint16_t raw = AnalogSampler::averageRaw(cfg::PIN_TEMP_ADC, 6);
    float v = AnalogSampler::rawToVoltage(raw);
    rr.raw = v;
    if (v < 0.02f || v > (cfg::ADC_REF_VOLT - 0.02f)) {
      rr.valid = false;
      rr.status = SensorStatus::Error;
      rr.value = _samples[indexOf(SensorId::EngineTemp)].lastGood;
      return rr;
    }

    float r = cfg::TEMP_NTC_FIXED_OHM * (v / (cfg::ADC_REF_VOLT - v));
    float invT = (1.0f / 298.15f) + (1.0f / cfg::TEMP_NTC_BETA) * logf(r / cfg::TEMP_NTC_R25_OHM);
    float tempC = (1.0f / invT) - 273.15f + cfg::TEMP_OFFSET_C;
    tempC = clampValue(tempC, -20.0f, 160.0f);
    rr.value = tempC;
    rr.valid = true;
    rr.status = (tempC > 105.0f) ? SensorStatus::Warning : SensorStatus::Ok;
    return rr;
  }

  ReadResult readBatteryVoltage() {
    ReadResult rr;
    rr.source = SensorSource::Hardware;
    if (!pinActive(cfg::PIN_BATT_ADC)) return rr;

    uint16_t raw = AnalogSampler::averageRaw(cfg::PIN_BATT_ADC, 8);
    float v = AnalogSampler::rawToVoltage(raw) * cfg::BATT_DIVIDER_RATIO;
    rr.raw = static_cast<float>(raw);
    if (v < 6.0f || v > 16.5f) {
      rr.valid = false;
      rr.status = SensorStatus::Error;
      rr.value = _samples[indexOf(SensorId::Battery)].lastGood;
      return rr;
    }

    rr.value = v;
    rr.valid = true;
    rr.status = (v < 11.4f) ? SensorStatus::Warning : SensorStatus::Ok;
    return rr;
  }

  ReadResult readFuelLevel() {
    ReadResult rr;
    rr.source = SensorSource::Hardware;
    if (!pinActive(cfg::PIN_FUEL_ADC)) return rr;

    uint16_t raw = AnalogSampler::averageRaw(cfg::PIN_FUEL_ADC, 8);
    float pct = mapFloat(static_cast<float>(raw), 3800.0f, 400.0f, 0.0f, 100.0f);
    pct = clampValue(pct, 0.0f, 100.0f);
    rr.raw = static_cast<float>(raw);
    rr.value = pct;
    rr.valid = (raw > 0 && raw < 4095);
    rr.status = rr.valid ? ((pct < 10.0f) ? SensorStatus::Warning : SensorStatus::Ok) : SensorStatus::Error;
    return rr;
  }

  ReadResult readTps() {
    ReadResult rr;
    rr.source = SensorSource::Hardware;
    if (!pinActive(cfg::PIN_TPS_ADC)) return rr;

    uint16_t raw = AnalogSampler::averageRaw(cfg::PIN_TPS_ADC, 6);
    float pct = mapFloat(static_cast<float>(raw), 250.0f, 3800.0f, 0.0f, 100.0f);
    pct = clampValue(pct, 0.0f, 100.0f);
    rr.raw = static_cast<float>(raw);
    rr.value = pct;
    rr.valid = (raw > 0 && raw < 4095);
    rr.status = rr.valid ? SensorStatus::Ok : SensorStatus::Error;
    return rr;
  }

  ReadResult readMap() {
    ReadResult rr;
    rr.source = SensorSource::Hardware;
    if (!pinActive(cfg::PIN_MAP_ADC)) return rr;

    uint16_t raw = AnalogSampler::averageRaw(cfg::PIN_MAP_ADC, 6);
    float kpa = mapFloat(static_cast<float>(raw), 120.0f, 3900.0f, 20.0f, 140.0f);
    kpa = clampValue(kpa, 0.0f, 300.0f);
    rr.raw = static_cast<float>(raw);
    rr.value = kpa;
    rr.valid = (raw > 0 && raw < 4095);
    rr.status = rr.valid ? SensorStatus::Ok : SensorStatus::Error;
    return rr;
  }

  ReadResult readExternalOrOffline(SensorId id, SensorSource source = SensorSource::Ecu) {
    ReadResult rr;
    const size_t idx = indexOf(id);
    const ExternalFeed& feed = _external[idx];
    rr.source = source;
    if (feed.valid && ((millis() - feed.lastMs) <= cfg::ECU_EXTERNAL_TIMEOUT_MS)) {
      rr.raw = feed.value;
      rr.value = feed.value;
      rr.valid = true;
      rr.status = SensorStatus::Ok;
      rr.source = feed.source;
      return rr;
    }

    rr.raw = _samples[idx].lastGood;
    rr.value = _samples[idx].lastGood;
    rr.valid = false;
    rr.status = SensorStatus::Offline;
    rr.source = SensorSource::Offline;
    return rr;
  }

  ReadResult readInjectorPulse(uint32_t now) {
    ReadResult rr = readExternalOrOffline(SensorId::InjectorPulse, SensorSource::Ecu);
    if (rr.valid) return rr;

    const float rpm = _samples[indexOf(SensorId::Rpm)].filtered;
    const float tps = _samples[indexOf(SensorId::Tps)].filtered;
    const float map = _samples[indexOf(SensorId::Map)].filtered;
    const float tempC = _samples[indexOf(SensorId::EngineTemp)].filtered;
    if (rpm < 100.0f) {
      rr.raw = 0.0f;
      rr.value = 0.0f;
      rr.valid = false;
      rr.status = SensorStatus::Offline;
      rr.source = SensorSource::Offline;
      return rr;
    }

    float pulse = estimateInjectorPulseMs(rpm, tps, map, tempC);
    rr.raw = pulse;
    rr.value = pulse;
    rr.valid = true;
    rr.status = SensorStatus::Warning;
    rr.source = SensorSource::Estimated;
    (void)now;
    return rr;
  }

  ReadResult readIgnitionTiming() {
    ReadResult rr = readExternalOrOffline(SensorId::IgnitionTiming, SensorSource::Ecu);
    if (rr.valid) return rr;

    const float rpm = _samples[indexOf(SensorId::Rpm)].filtered;
    const float tps = _samples[indexOf(SensorId::Tps)].filtered;
    const float map = _samples[indexOf(SensorId::Map)].filtered;
    if (rpm < 100.0f) {
      rr.valid = false;
      rr.status = SensorStatus::Offline;
      return rr;
    }

    float timing = clampValue(35.0f - (rpm / 600.0f) - (tps * 0.08f) - (map / 30.0f), 0.0f, 45.0f);
    rr.raw = timing;
    rr.value = timing;
    rr.valid = true;
    rr.status = SensorStatus::Warning;
    rr.source = SensorSource::Estimated;
    return rr;
  }

  ReadResult readCkp() {
    ReadResult rr;
    rr.source = SensorSource::Hardware;
    const float rpm = _samples[indexOf(SensorId::Rpm)].filtered;
    if (rpm < 10.0f) {
      rr.valid = false;
      rr.status = SensorStatus::Offline;
      rr.raw = 0.0f;
      rr.value = 0.0f;
      return rr;
    }

    float hz = (rpm / 60.0f) * cfg::RPM_PULSES_PER_REV;
    rr.raw = hz;
    rr.value = hz;
    rr.valid = true;
    rr.status = SensorStatus::Ok;
    return rr;
  }

  ReadResult readFuelPumpStatus() { return readExternalOrOffline(SensorId::FuelPump); }
  ReadResult readRadiatorFanStatus() { return readExternalOrOffline(SensorId::RadiatorFan); }
  ReadResult readGearPosition() {
    ReadResult rr = readExternalOrOffline(SensorId::GearPosition);
    if (rr.valid) return rr;
    const float speed = _samples[indexOf(SensorId::Speed)].filtered;
    const float rpm = _samples[indexOf(SensorId::Rpm)].filtered;
    if (speed < 1.0f || rpm < 100.0f) {
      rr.valid = false;
      rr.status = SensorStatus::Offline;
      return rr;
    }
    float ratio = rpm / clampValue(speed, 1.0f, 200.0f);
    float gear = 6.0f - clampValue(ratio / 120.0f, 0.0f, 5.0f);
    gear = clampValue(roundf(gear), 1.0f, 6.0f);
    rr.raw = gear;
    rr.value = gear;
    rr.valid = true;
    rr.status = SensorStatus::Warning;
    rr.source = SensorSource::Estimated;
    return rr;
  }
  ReadResult readLeanAngle() { return readExternalOrOffline(SensorId::LeanAngle); }
  ReadResult readOilPressure() { return readExternalOrOffline(SensorId::OilPressure); }
  ReadResult readKnock() { return readExternalOrOffline(SensorId::Knock); }
  ReadResult readAmbientTemp() { return readExternalOrOffline(SensorId::AmbientTemp); }
  ReadResult readHumidity() { return readExternalOrOffline(SensorId::Humidity); }
  ReadResult readBarometricPressure() { return readExternalOrOffline(SensorId::BarometricPressure); }
  ReadResult readDtcCode() { return readExternalOrOffline(SensorId::DtcCode); }
  ReadResult readIat() { return readExternalOrOffline(SensorId::Iat); }
  ReadResult readEot() { return readExternalOrOffline(SensorId::Eot); }

  void updateFastSensors(uint32_t now) {
    writeSample(SensorId::Speed, readSpeed(now), now);
    writeSample(SensorId::Rpm, readRPM(now), now);
    writeSample(SensorId::Afr, readAFR(), now);
    writeSample(SensorId::Tps, readTPS(), now);
    writeSample(SensorId::Map, readMAP(), now);
    writeSample(SensorId::InjectorPulse, readInjectorPulse(now), now);
    writeSample(SensorId::IgnitionTiming, readIgnitionTiming(), now);
    writeSample(SensorId::Ckp, readCkp(), now);
  }

  void updateSlowSensors(uint32_t now) {
    writeSample(SensorId::EngineTemp, readEngineTemp(), now);
    writeSample(SensorId::Battery, readBatteryVoltage(), now);
    writeSample(SensorId::FuelLevel, readFuelLevel(), now);
    writeSample(SensorId::Iat, readIAT(), now);
    writeSample(SensorId::Eot, readEot(), now);
    writeSample(SensorId::FuelPump, readFuelPumpStatus(), now);
    writeSample(SensorId::RadiatorFan, readRadiatorFanStatus(), now);
    writeSample(SensorId::GearPosition, readGearPosition(), now);
    writeSample(SensorId::LeanAngle, readLeanAngle(), now);
    writeSample(SensorId::OilPressure, readOilPressure(), now);
    writeSample(SensorId::Knock, readKnock(), now);
    writeSample(SensorId::AmbientTemp, readAmbientTemp(), now);
    writeSample(SensorId::Humidity, readHumidity(), now);
    writeSample(SensorId::BarometricPressure, readBarometricPressure(), now);
    writeSample(SensorId::DtcCode, readDtcCode(), now);
  }

  void updateSimulation(uint32_t now) {
    if (_lastSimMs == 0) _lastSimMs = now;
    _lastSimMs = now;
    _simPhase += 0.015f;

    float speed = 58.0f + 22.0f * sinf(_simPhase * 0.8f);
    float rpm = 3200.0f + 1800.0f * sinf(_simPhase * 1.7f) + 200.0f * sinf(_simPhase * 5.1f);
    float afr = 14.6f + 0.9f * sinf(_simPhase * 2.3f);
    float temp = 82.0f + 11.0f * sinf(_simPhase * 0.45f);
    float batt = 13.9f + 0.3f * sinf(_simPhase * 0.2f);
    float fuel = 62.0f - fmodf(_simPhase * 0.04f, 6.0f);
    float tps = 18.0f + 25.0f * (0.5f + 0.5f * sinf(_simPhase * 1.1f));
    float map = 66.0f + 24.0f * sinf(_simPhase * 0.9f);
    float iat = 31.0f + 2.0f * sinf(_simPhase * 0.25f);
    float eot = 86.0f + 8.0f * sinf(_simPhase * 0.35f);
    float inj = estimateInjectorPulseMs(rpm, tps, map, temp);
    float ign = clampValue(30.0f - (rpm / 700.0f) - (tps * 0.07f), 5.0f, 40.0f);
    float gear = clampValue(6.0f - (rpm / 2200.0f), 1.0f, 6.0f);
    float lean = 0.8f * sinf(_simPhase * 0.6f);
    float oil = 2.2f + 0.02f * (rpm / 100.0f);
    float knock = 1.0f + 0.3f * sinf(_simPhase * 3.7f);
    float ambient = 30.5f + 1.2f * sinf(_simPhase * 0.15f);
    float hum = 53.0f + 7.0f * sinf(_simPhase * 0.21f);
    float baro = 100.7f + 0.6f * sinf(_simPhase * 0.17f);
    bool pump = true;
    bool fan = temp > 90.0f;
    uint16_t dtc = 0;

    writeSample(SensorId::Speed, {speed, speed, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::Rpm, {rpm, rpm, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::Afr, {afr, afr, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::EngineTemp, {temp, temp, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::Battery, {batt, batt, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::FuelLevel, {fuel, fuel, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::Tps, {tps, tps, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::Map, {map, map, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::Iat, {iat, iat, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::Eot, {eot, eot, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::InjectorPulse, {inj, inj, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::IgnitionTiming, {ign, ign, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::Ckp, {rpm / 60.0f, rpm / 60.0f, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::FuelPump, {pump ? 1.0f : 0.0f, pump ? 1.0f : 0.0f, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::RadiatorFan, {fan ? 1.0f : 0.0f, fan ? 1.0f : 0.0f, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::GearPosition, {gear, gear, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::LeanAngle, {lean, lean, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::OilPressure, {oil, oil, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::Knock, {knock, knock, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::AmbientTemp, {ambient, ambient, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::Humidity, {hum, hum, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::BarometricPressure, {baro, baro, true, SensorStatus::Ok, SensorSource::Simulation}, now);
    writeSample(SensorId::DtcCode, {static_cast<float>(dtc), static_cast<float>(dtc), true, SensorStatus::Ok, SensorSource::Simulation}, now);
  }

  void writeSample(SensorId id, const ReadResult& rr, uint32_t now) {
    const size_t idx = indexOf(id);
    SensorSample& s = _samples[idx];
    s.raw = rr.raw;
    s.lastUpdateMs = now;
    if (rr.valid && isFiniteNumber(rr.value)) {
      s.valid = true;
      s.source = rr.source;
      s.status = rr.status;
      s.converted = rr.value;
      if (!s.hasValue) {
        s.filtered = rr.value;
        s.hasValue = true;
      } else {
        s.filtered = lowPassFilter(s.filtered, rr.value, 0.22f);
      }
      _history[idx].push(s.filtered);
      s.avg = _history[idx].mean();
      if (s.min == 0.0f && s.max == 0.0f && s.lastGoodMs == 0) {
        s.min = s.max = s.filtered;
      } else {
        if (s.filtered < s.min) s.min = s.filtered;
        if (s.filtered > s.max) s.max = s.filtered;
      }
      s.lastGood = s.filtered;
      s.lastGoodMs = now;
      return;
    }

    s.valid = false;
    s.source = rr.source;
    s.errorCount++;
    s.converted = s.lastGood;
    s.filtered = s.lastGood;
    s.avg = _history[idx].mean();
    if (rr.status == SensorStatus::Error) {
      s.status = SensorStatus::Error;
    } else if (s.lastGoodMs == 0 || (now - s.lastGoodMs) > s.timeoutMs) {
      s.status = SensorStatus::Offline;
    } else {
      s.status = SensorStatus::Warning;
    }
  }

  void updateTimeouts(uint32_t now) {
    for (size_t i = 0; i < SENSOR_COUNT; ++i) {
      SensorSample& s = _samples[i];
      if (s.lastGoodMs == 0) continue;
      if ((now - s.lastGoodMs) > s.timeoutMs) {
        s.status = SensorStatus::Offline;
        s.valid = false;
        s.converted = s.lastGood;
        s.filtered = s.lastGood;
      }
    }
  }

  void updateDistance(uint32_t now) {
    if (_lastDistanceMs == 0) {
      _lastDistanceMs = now;
      return;
    }
    const uint32_t dtMs = now - _lastDistanceMs;
    _lastDistanceMs = now;
    const float hours = dtMs / 3600000.0f;
    _distanceMeters += _samples[indexOf(SensorId::Speed)].filtered * hours * 1000.0f;
  }

  float estimateInjectorPulseMs(float rpm, float throttlePct, float mapKpa, float tempC) const {
    if (rpm < 100.0f) return 0.0f;
    float load = 0.55f * (throttlePct / 100.0f) + 0.45f * clampValue(mapKpa / 101.3f, 0.0f, 1.6f);
    float warmup = (tempC < 60.0f) ? 1.12f : 1.0f;
    float pulse = (1.0f + load * 5.5f + (rpm / 10000.0f) * 1.8f) * warmup;
    return clampValue(pulse, 0.8f, 16.0f);
  }

  void buildSnapshot(uint32_t now) {
    _snapshot.timestampMs = now;
    _snapshot.speedKmh = _samples[indexOf(SensorId::Speed)].filtered;
    _snapshot.rpm = static_cast<uint16_t>(roundf(_samples[indexOf(SensorId::Rpm)].filtered));
    _snapshot.afr = _samples[indexOf(SensorId::Afr)].filtered;
    _snapshot.engineTempC = _samples[indexOf(SensorId::EngineTemp)].filtered;
    _snapshot.batteryVolt = _samples[indexOf(SensorId::Battery)].filtered;
    _snapshot.fuelPercent = _samples[indexOf(SensorId::FuelLevel)].filtered;
    _snapshot.throttlePct = _samples[indexOf(SensorId::Tps)].filtered;
    _snapshot.mapKpa = _samples[indexOf(SensorId::Map)].filtered;
    _snapshot.iatC = _samples[indexOf(SensorId::Iat)].filtered;
    _snapshot.eotC = _samples[indexOf(SensorId::Eot)].filtered;
    _snapshot.injectorPulseMs = _samples[indexOf(SensorId::InjectorPulse)].filtered;
    _snapshot.ignitionTimingDeg = _samples[indexOf(SensorId::IgnitionTiming)].filtered;
    _snapshot.ckpHz = _samples[indexOf(SensorId::Ckp)].filtered;
    _snapshot.gearPosition = _samples[indexOf(SensorId::GearPosition)].filtered;
    _snapshot.leanAngleDeg = _samples[indexOf(SensorId::LeanAngle)].filtered;
    _snapshot.oilPressureBar = _samples[indexOf(SensorId::OilPressure)].filtered;
    _snapshot.knockLevel = _samples[indexOf(SensorId::Knock)].filtered;
    _snapshot.ambientTempC = _samples[indexOf(SensorId::AmbientTemp)].filtered;
    _snapshot.humidityPct = _samples[indexOf(SensorId::Humidity)].filtered;
    _snapshot.baroKpa = _samples[indexOf(SensorId::BarometricPressure)].filtered;
    _snapshot.fuelPumpOn = (_samples[indexOf(SensorId::FuelPump)].filtered > 0.5f);
    _snapshot.radiatorFanOn = (_samples[indexOf(SensorId::RadiatorFan)].filtered > 0.5f);
    _snapshot.dtcCode = static_cast<uint16_t>(roundf(_samples[indexOf(SensorId::DtcCode)].filtered));
    _snapshot.ecuEnabled = _ecuEnabled;
    _snapshot.ecuOnline = _ecuOnline;
    _snapshot.engineRunning = (_snapshot.rpm > 300U);
  }
};

volatile uint32_t SensorHub::s_rpmPulseCount = 0;
volatile uint32_t SensorHub::s_rpmLastEdgeUs = 0;
volatile uint32_t SensorHub::s_speedPulseCount = 0;
volatile uint32_t SensorHub::s_speedLastEdgeUs = 0;

// ============================================================
// Fuel estimator
// ============================================================
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
    _history.reset();
  }

  void calculateFuelConsumption(DashboardSnapshot& s) {
    uint32_t now = millis();
    if (_lastUpdateMs == 0) _lastUpdateMs = now;
    uint32_t dtMs = now - _lastUpdateMs;
    _lastUpdateMs = now;

    float dtHours = dtMs / 3600000.0f;
    float speed = s.speedKmh;
    float rpm = static_cast<float>(s.rpm);
    float throttle = s.throttlePct;
    float map = s.mapKpa;

    float injectorPulseMs = s.injectorPulseMs;
    if (injectorPulseMs <= 0.0f) {
      injectorPulseMs = estimateInjectorPulseMs(rpm, throttle, map, s.engineTempC);
      s.injectorPulseMs = injectorPulseMs;
    }

    if (speed < 1.0f || rpm < 250.0f) {
      _instantKmL = 0.0f;
    } else {
      float duty = injectorPulseMs / (60000.0f / clampValue(rpm, 1.0f, 14000.0f));
      duty = clampValue(duty, 0.01f, 0.95f);
      float fuelFlowCcMin = cfg::INJECTOR_FLOW_CC_MIN * duty;
      float fuelFlowLh = fuelFlowCcMin / 1000.0f * 60.0f;
      _instantKmL = (fuelFlowLh > 0.01f) ? clampValue(speed / fuelFlowLh, 0.1f, 99.9f) : 0.0f;
    }

    if (dtHours > 0.0f) {
      _totalDistanceKm += speed * dtHours;
      if (_instantKmL > 0.01f && speed > 0.1f) {
        float fuelFlowLh = speed / _instantKmL;
        _totalFuelL += fuelFlowLh * dtHours;
      }
    }

    _history.push(_instantKmL);
    _averageKmL = _history.mean();
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
  HistoryWindow<float, 30> _history;

  float estimateInjectorPulseMs(float rpm, float throttlePct, float mapKpa, float tempC) const {
    if (rpm < 100.0f) return 0.0f;
    float load = 0.55f * (throttlePct / 100.0f) + 0.45f * clampValue(mapKpa / 101.3f, 0.0f, 1.6f);
    float warmup = (tempC < 60.0f) ? 1.12f : 1.0f;
    float pulse = (1.0f + load * 5.5f + (rpm / 10000.0f) * 1.8f) * warmup;
    return clampValue(pulse, 0.8f, 16.0f);
  }
};

// ============================================================
// Engine analyzer
// ============================================================
class EngineAnalyzer {
public:
  EngineAnalyzer() : _health(100.0f) {}

  void reset() {
    _health = 100.0f;
    _rpmHistory.reset();
    _afrHistory.reset();
    _tempHistory.reset();
    _battHistory.reset();
  }

  float analyzeAFR(float afr, SensorStatus status, float stability = 0.0f) {
    float diff = fabsf(afr - 14.7f);
    float base = clampValue(100.0f - (diff * 18.0f), 0.0f, 100.0f);
    if (status == SensorStatus::Warning) base -= 5.0f;
    if (status == SensorStatus::Error || status == SensorStatus::Offline) base *= 0.55f;
    return clampValue((base * 0.75f) + (stability * 0.25f), 0.0f, 100.0f);
  }

  float analyzeTemperature(float tempC) {
    if (tempC < 0.0f) return 25.0f;
    if (tempC < 40.0f) return 60.0f;
    if (tempC <= 95.0f) return 100.0f;
    if (tempC <= 110.0f) return mapFloat(tempC, 95.0f, 110.0f, 100.0f, 35.0f);
    if (tempC <= 120.0f) return mapFloat(tempC, 110.0f, 120.0f, 35.0f, 10.0f);
    return 0.0f;
  }

  float analyzeBattery(float volt) {
    if (volt < 10.0f) return 0.0f;
    if (volt < 11.5f) return 25.0f;
    if (volt < 12.2f) return 60.0f;
    if (volt <= 14.8f) return 100.0f;
    if (volt <= 15.2f) return 75.0f;
    return 20.0f;
  }

  float analyzeRPM(float rpm, float jitter) {
    float base = 100.0f;
    if (rpm < 100.0f) base = 85.0f;
    else if (rpm < 700.0f) base = 60.0f;
    else if (rpm > 10500.0f) base = 50.0f;
    float smooth = clampValue(100.0f - (jitter * 0.25f), 0.0f, 100.0f);
    return (base * 0.5f) + (smooth * 0.5f);
  }

  float calculateEngineHealth(const DashboardSnapshot& s, const SensorHub& hub) {
    _rpmHistory.push(static_cast<float>(s.rpm));
    _afrHistory.push(s.afr);
    _tempHistory.push(s.engineTempC);
    _battHistory.push(s.batteryVolt);

    float tempScore = analyzeTemperature(s.engineTempC);
    float afrScore  = analyzeAFR(s.afr, hub.sample(SensorId::Afr).status, clampValue(100.0f - (_afrHistory.stddev() * 18.0f), 0.0f, 100.0f));
    float voltScore = analyzeBattery(s.batteryVolt);
    float rpmScore  = analyzeRPM(static_cast<float>(s.rpm), _rpmHistory.stddev());
    float fuelScore = analyzeFuelEconomy(s.fuelAverageKmL, s.fuelLPer100Km);

    float health = (tempScore * cfg::HEALTH_W_TEMP) +
                   (afrScore  * cfg::HEALTH_W_AFR) +
                   (voltScore * cfg::HEALTH_W_VOLT) +
                   (rpmScore  * cfg::HEALTH_W_RPM) +
                   (fuelScore * cfg::HEALTH_W_FUEL);

    if (s.engineRunning) {
      if (hub.sample(SensorId::EngineTemp).status != SensorStatus::Ok) health -= 3.0f;
      if (hub.sample(SensorId::Battery).status != SensorStatus::Ok) health -= 4.0f;
      if (hub.sample(SensorId::Afr).status != SensorStatus::Ok) health -= 4.0f;
      if (hub.sample(SensorId::Rpm).status != SensorStatus::Ok) health -= 2.0f;
    }

    if (s.ecuEnabled && !s.ecuOnline) health -= 5.0f;
    _health = clampValue(health, 0.0f, 100.0f);
    return _health;
  }

  float health() const { return _health; }

private:
  float _health;
  HistoryWindow<float, 24> _rpmHistory;
  HistoryWindow<float, 20> _afrHistory;
  HistoryWindow<float, 20> _tempHistory;
  HistoryWindow<float, 20> _battHistory;

  float analyzeFuelEconomy(float kmL, float l100) {
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

// ============================================================
// K-Line / KWP2000 scaffolding
// ============================================================
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

class PacketValidator {
public:
  static uint8_t checksum(const uint8_t* data, size_t length) {
    uint16_t sum = 0;
    for (size_t i = 0; i < length; ++i) sum += data[i];
    return static_cast<uint8_t>(sum & 0xFF);
  }

  static bool isHeaderValid(uint8_t header) {
    return (header == 0x68 || header == 0x80 || header == 0xC2 || header == 0x02 || header == 0x0E);
  }

  static bool validatePacket(const uint8_t* raw, size_t rawLen) {
    if (!raw || rawLen < 5) return false;
    if (!isHeaderValid(raw[0])) return false;
    uint8_t len = raw[3];
    if (len > cfg::MAX_ECU_PAYLOAD) return false;
    size_t total = static_cast<size_t>(4) + static_cast<size_t>(len) + 1;
    if (rawLen < total) return false;
    return checksum(raw, 4 + len) == raw[4 + len];
  }
};

class KWP2000Handler {
public:
  static size_t buildRequestFrame(uint8_t* out,
                                  size_t outSize,
                                  uint8_t header,
                                  uint8_t target,
                                  uint8_t source,
                                  uint8_t service,
                                  uint8_t pid) {
    if (!out || outSize < 7) return 0;
    out[0] = header;
    out[1] = target;
    out[2] = source;
    out[3] = 0x02;
    out[4] = service;
    out[5] = pid;
    out[6] = PacketValidator::checksum(out, 6);
    return 7;
  }

  static bool parseFrame(const uint8_t* raw, size_t rawLen, ECUFrame& frame) {
    if (!PacketValidator::validatePacket(raw, rawLen)) return false;
    const uint8_t len = raw[3];
    frame.header = raw[0];
    frame.target = raw[1];
    frame.source = raw[2];
    frame.length = len;
    frame.payloadLen = len;
    frame.checksum = raw[4 + len];
    frame.timestampMs = millis();
    frame.valid = true;
    for (size_t i = 0; i < len; ++i) {
      frame.payload[i] = raw[4 + i];
    }
    return true;
  }
};

class KLineManager {
public:
  KLineManager()
    : _serial(Serial2),
      _state(EcuState::Disabled),
      _protocol(ProtocolType::Unknown),
      _enabled(false),
      _connected(false),
      _requestPending(false),
      _ecuAddress(cfg::ECU_ADDR_DEFAULT),
      _testerAddress(cfg::TESTER_ADDR_DEFAULT),
      _header(cfg::KLINE_HEADER_DEFAULT),
      _lastProbeMs(0),
      _lastRxMs(0),
      _lastRetryMs(0),
      _lastRequestMs(0),
      _pendingService(0),
      _pendingPid(0),
      _errorCount(0),
      _rxLen(0),
      _haveFrame(false) {}

  bool begin(int16_t rxPin = cfg::PIN_KLINE_RX,
             int16_t txPin = cfg::PIN_KLINE_TX,
             uint32_t baud = cfg::KLINE_BAUD) {
    _rxPin = rxPin;
    _txPin = txPin;
    _baud = baud;

    if (pinActive(_txPin)) {
      pinMode(static_cast<uint8_t>(_txPin), OUTPUT);
      digitalWrite(static_cast<uint8_t>(_txPin), HIGH);
    }

    if (pinActive(_rxPin) && pinActive(_txPin)) {
      _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
    }
    _serial.setTimeout(cfg::KLINE_RX_TIMEOUT_MS);
    _state = EcuState::Idle;

    if (cfg::DEBUG_MODE) {
      Serial.printf("[KLINE] UART ready RX=%d TX=%d BAUD=%lu\n",
                    _rxPin, _txPin, static_cast<unsigned long>(_baud));
      Serial.println("[KLINE] Use MC33290 / L9637 transceiver and protected DLC wiring");
    }
    return true;
  }

  bool initKLine(int16_t rxPin = cfg::PIN_KLINE_RX,
                 int16_t txPin = cfg::PIN_KLINE_TX,
                 uint32_t baud = cfg::KLINE_BAUD) {
    return begin(rxPin, txPin, baud);
  }

  void enable(bool enabled) {
    _enabled = enabled;
    if (!enabled) {
      _state = EcuState::Disabled;
      _connected = false;
      _requestPending = false;
    } else {
      _state = EcuState::Reconnecting;
      _lastRetryMs = 0;
    }
  }

  void update() {
    if (!_enabled) return;
    readIncoming();

    const uint32_t now = millis();
    if (_requestPending && ((now - _lastRequestMs) > cfg::KLINE_RX_TIMEOUT_MS)) {
      handleECUTimeout();
    }

    if (_connected && ((now - _lastRxMs) > cfg::KLINE_RX_TIMEOUT_MS)) {
      _connected = false;
      _state = EcuState::Reconnecting;
      _protocol = ProtocolType::Unknown;
      if (cfg::DEBUG_MODE) Serial.println("[KLINE] ECU timeout, reconnect scheduled");
    }

    if (!_connected && ((now - _lastRetryMs) > cfg::KLINE_RETRY_MS)) {
      _lastRetryMs = now;
      _state = EcuState::Probing;
      sendKWPRequest(cfg::OBD_MODE_CURRENT, cfg::PID_SUPPORTED_01, true);
    }
  }

  bool sendKWPRequest(uint8_t service, uint8_t pid, bool force = false) {
    if (!_enabled) return false;
    if (!_connected && !force) return false;
    if (_requestPending && !force) return false;

    uint8_t frame[8] = {};
    size_t frameLen = KWP2000Handler::buildRequestFrame(frame, sizeof(frame),
                                                        _header, _ecuAddress, _testerAddress,
                                                        service, pid);
    if (frameLen == 0) return false;

    _serial.write(frame, frameLen);
    _pendingService = service;
    _pendingPid = pid;
    _lastRequestMs = millis();
    _requestPending = true;
    _state = EcuState::WaitingResponse;

    if (cfg::DEBUG_MODE) {
      Serial.printf("[KLINE] TX service=0x%02X pid=0x%02X len=%u\n",
                    service, pid, static_cast<unsigned>(frameLen));
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
    _state = EcuState::Connected;
    _lastRxMs = millis();
    if (cfg::DEBUG_MODE) Serial.println("[KLINE] ECU connected");
  }

  void handleECUTimeout() {
    _requestPending = false;
    _state = EcuState::Reconnecting;
    _errorCount++;
  }

  void reset() {
    _connected = false;
    _requestPending = false;
    _state = _enabled ? EcuState::Reconnecting : EcuState::Disabled;
    _protocol = ProtocolType::Unknown;
    _rxLen = 0;
    _haveFrame = false;
  }

  bool isEnabled() const { return _enabled; }
  bool isConnected() const { return _connected; }
  bool waitingResponse() const { return _requestPending; }
  EcuState state() const { return _state; }
  ProtocolType protocol() const { return _protocol; }
  uint32_t errorCount() const { return _errorCount; }
  const char* stateString() const {
    return ecuStateText(_state);
  }

private:
  HardwareSerial& _serial;
  EcuState _state;
  ProtocolType _protocol;
  bool _enabled;
  bool _connected;
  bool _requestPending;

  int16_t _rxPin;
  int16_t _txPin;
  uint32_t _baud;
  uint8_t _ecuAddress;
  uint8_t _testerAddress;
  uint8_t _header;

  uint32_t _lastProbeMs;
  uint32_t _lastRxMs;
  uint32_t _lastRetryMs;
  uint32_t _lastRequestMs;
  uint8_t _pendingService;
  uint8_t _pendingPid;
  uint32_t _errorCount;

  uint8_t _rxBuffer[cfg::MAX_ECU_FRAME];
  size_t _rxLen;
  ECUFrame _pendingFrame;
  bool _haveFrame;

  void readIncoming() {
    while (_serial.available()) {
      uint8_t b = static_cast<uint8_t>(_serial.read());
      if (_rxLen < sizeof(_rxBuffer)) {
        _rxBuffer[_rxLen++] = b;
      } else {
        memmove(_rxBuffer, _rxBuffer + 1, sizeof(_rxBuffer) - 1);
        _rxBuffer[sizeof(_rxBuffer) - 1] = b;
        _errorCount++;
      }
    }

    while (_rxLen >= 5) {
      if (!PacketValidator::isHeaderValid(_rxBuffer[0])) {
        shiftBuffer(1);
        continue;
      }

      size_t frameLen = static_cast<size_t>(4) + static_cast<size_t>(_rxBuffer[3]) + 1;
      if (frameLen > sizeof(_rxBuffer)) {
        shiftBuffer(1);
        _errorCount++;
        continue;
      }

      if (_rxLen < frameLen) return;

      ECUFrame frame;
      if (KWP2000Handler::parseFrame(_rxBuffer, frameLen, frame)) {
        _pendingFrame = frame;
        _haveFrame = true;
        _requestPending = false;
        _lastRxMs = millis();
        if (!_connected) markConnected(ProtocolType::ISO9141);
        if (cfg::DEBUG_MODE) {
          Serial.printf("[KLINE] RX frame len=%u payload=%u\n",
                        static_cast<unsigned>(frameLen),
                        static_cast<unsigned>(frame.payloadLen));
        }
      } else {
        _errorCount++;
        if (cfg::DEBUG_MODE) Serial.println("[KLINE] Invalid ECU frame");
        shiftBuffer(1);
        continue;
      }

      if (_rxLen > frameLen) {
        memmove(_rxBuffer, _rxBuffer + frameLen, _rxLen - frameLen);
        _rxLen -= frameLen;
      } else {
        _rxLen = 0;
      }
    }
  }

  void shiftBuffer(size_t n) {
    if (n >= _rxLen) {
      _rxLen = 0;
      return;
    }
    memmove(_rxBuffer, _rxBuffer + n, _rxLen - n);
    _rxLen -= n;
  }
};

class ECUDataParser {
public:
  static void parseECUResponse(const ECUFrame& frame, SensorHub& hub) {
    if (!frame.valid || frame.payloadLen < 2) return;
    const uint8_t service = frame.payload[0];
    const uint8_t pid = frame.payload[1];
    if (service < 0x40) return;

    switch (pid) {
      case cfg::PID_ENGINE_RPM:
        if (frame.payloadLen >= 4) {
          float rpm = ((static_cast<uint16_t>(frame.payload[2]) << 8) | frame.payload[3]) / 4.0f;
          hub.injectRpm(rpm, true);
        }
        break;

      case cfg::PID_SPEED:
        if (frame.payloadLen >= 3) hub.injectSpeed(static_cast<float>(frame.payload[2]), true);
        break;

      case cfg::PID_COOLANT_TEMP:
        if (frame.payloadLen >= 3) hub.injectEngineTemp(static_cast<float>(frame.payload[2]) - 40.0f, true);
        break;

      case cfg::PID_THROTTLE_POS:
        if (frame.payloadLen >= 3) hub.injectTps((static_cast<float>(frame.payload[2]) * 100.0f) / 255.0f, true);
        break;

      case cfg::PID_O2_SENSOR_1:
        if (frame.payloadLen >= 3) {
          float voltage = static_cast<float>(frame.payload[2]) * 0.005f;
          float afr = clampValue(14.7f + ((voltage - 0.45f) * 6.0f), 8.0f, 22.0f);
          hub.injectAfr(afr, true);
        }
        break;

      case cfg::PID_MAP:
        if (frame.payloadLen >= 3) hub.injectMap(static_cast<float>(frame.payload[2]), true);
        break;

      case cfg::PID_IAT:
        if (frame.payloadLen >= 3) hub.injectIat(static_cast<float>(frame.payload[2]) - 40.0f, true);
        break;

      case cfg::PID_BATTERY_VOLT:
        if (frame.payloadLen >= 3) hub.injectBattery(static_cast<float>(frame.payload[2]) * 0.1f, true);
        break;

      case cfg::PID_DTC:
        if (frame.payloadLen >= 4) {
          uint16_t code = (static_cast<uint16_t>(frame.payload[2]) << 8) | frame.payload[3];
          hub.injectDtc(code, true);
        }
        break;

      default:
        break;
    }
  }
};

struct RequestSlot {
  uint8_t service = 0;
  uint8_t pid = 0;
  uint32_t intervalMs = 0;
  uint32_t lastMs = 0;
};

class ECURequestManager {
public:
  void begin(bool enabled) {
    _enabled = enabled;
    _slotCount = 0;
    _cursor = 0;
    _lastProcessMs = 0;

    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_ENGINE_RPM,   100);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_SPEED,        100);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_COOLANT_TEMP, 500);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_THROTTLE_POS,  200);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_O2_SENSOR_1,   250);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_MAP,           200);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_IAT,          500);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_BATTERY_VOLT, 1000);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_STFT,         1000);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_LTFT,         1000);
    addSlot(cfg::OBD_MODE_CURRENT, cfg::PID_DTC,          5000);
  }

  void update(KLineManager& kline, SensorHub& hub, UiMode mode) {
    if (!_enabled) return;

    ECUFrame frame;
    while (kline.pollFrame(frame)) {
      ECUDataParser::parseECUResponse(frame, hub);
    }

    hub.setEcuOnline(kline.isConnected());

    if (!kline.isConnected()) return;
    if (kline.waitingResponse()) return;

    const uint32_t now = millis();
    if ((now - _lastProcessMs) < cfg::KLINE_POLL_MS) return;
    _lastProcessMs = now;

    if (_slotCount == 0) return;
    RequestSlot& slot = _slots[_cursor];
    if ((now - slot.lastMs) >= slot.intervalMs) {
      slot.lastMs = now;
      kline.sendKWPRequest(slot.service, slot.pid, false);
    }

    _cursor = (_cursor + 1) % _slotCount;

    if (mode == UiMode::Diagnostic) {
      // Keep diagnostic mode focused on fast ECU visibility.
      // Future enhancement can raise DTC polling frequency here.
    }
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
};

// ============================================================
// ECU mapping reference data
// ============================================================
static const uint16_t ECU_RPM_BANDS[8] = { 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000 };
static const uint8_t ECU_TPS_BANDS[5] = { 0, 25, 50, 75, 100 };
static const float ECU_MAP_DUMMY[5][8] = {
  {15.2f, 15.0f, 14.8f, 14.6f, 14.4f, 14.1f, 13.8f, 13.5f},
  {15.0f, 14.8f, 14.6f, 14.4f, 14.1f, 13.9f, 13.6f, 13.3f},
  {14.8f, 14.6f, 14.4f, 14.2f, 14.0f, 13.8f, 13.5f, 13.2f},
  {14.6f, 14.4f, 14.2f, 14.0f, 13.8f, 13.6f, 13.3f, 13.0f},
  {14.4f, 14.2f, 14.0f, 13.8f, 13.6f, 13.4f, 13.1f, 12.9f}
};

// ============================================================
// Dashboard UI
// ============================================================
class DashboardUI {
public:
  DashboardUI()
    : _mode(UiMode::Dashboard),
      _layoutDirty(true),
      _warningBlink(true),
      _forceDashboardRender(true),
      _lastBlinkMs(0),
      _lastRenderMs(0),
      _lastGraphMs(0),
      _lastStatusMs(0),
      _lastFullRedrawMs(0),
      _fps(0),
      _frameCounter(0),
      _fpsWindowMs(0) {}

  bool begin() {
    _tft.init();
    _tft.setRotation(cfg::DISPLAY_ROTATION);
    _tft.fillScreen(cfg::COL_BG);
    _tft.setTextWrap(false);

    if (cfg::DEBUG_MODE) {
      Serial.println("[UI] TFT initialized");
      if (!psramFound()) {
        Serial.println("[UI][WARN] PSRAM not detected; direct rendering will be used");
      }
    }

    runBootAnimation();
    drawStaticLayout();
    _layoutDirty = false;
    _lastFullRedrawMs = millis();
    _fpsWindowMs = millis();
    return true;
  }

  void setMode(UiMode mode) {
    if (_mode != mode) {
      _mode = mode;
      _layoutDirty = true;
      _forceDashboardRender = true;
    }
  }

  UiMode mode() const { return _mode; }

  void update(const DashboardSnapshot& frame, const SensorHub& hub) {
    const uint32_t now = millis();

    if ((now - _lastBlinkMs) >= 350) {
      _lastBlinkMs = now;
      _warningBlink = !_warningBlink;
    }

    if (_layoutDirty || ((now - _lastFullRedrawMs) >= cfg::FULL_REDRAW_MS)) {
      drawStaticLayout();
      _layoutDirty = false;
      _lastFullRedrawMs = now;
    }

    bool shouldRender = false;
    uint32_t interval = cfg::UI_DASH_MS;
    switch (_mode) {
      case UiMode::Dashboard:     interval = cfg::UI_DASH_MS; break;
      case UiMode::EcuMapping:    interval = cfg::UI_TABLE_MS; break;
      case UiMode::Diagnostic:    interval = cfg::UI_TABLE_MS; break;
      case UiMode::SensorMonitor: interval = cfg::UI_TABLE_MS; break;
    }

    if ((now - _lastRenderMs) >= interval) {
      _lastRenderMs = now;
      shouldRender = true;
    }

    if (!shouldRender) return;

    _tft.startWrite();
    drawStatusPanel(frame);
    switch (_mode) {
      case UiMode::Dashboard:
        drawDashboardMode(frame, hub, now);
        break;
      case UiMode::EcuMapping:
        drawECUMappingMode(frame, hub, now);
        break;
      case UiMode::Diagnostic:
        drawDiagnosticMode(frame, hub, now);
        break;
      case UiMode::SensorMonitor:
        drawSensorMonitorMode(frame, hub, now);
        break;
    }
    _tft.endWrite();

    _frameCounter++;
    if ((now - _fpsWindowMs) >= 1000UL) {
      _fps = _frameCounter;
      _frameCounter = 0;
      _fpsWindowMs = now;
    }
  }

private:
  TFT_eSPI _tft;
  UiMode _mode;
  bool _layoutDirty;
  bool _warningBlink;
  bool _forceDashboardRender;
  uint32_t _lastBlinkMs;
  uint32_t _lastRenderMs;
  uint32_t _lastGraphMs;
  uint32_t _lastStatusMs;
  uint32_t _lastFullRedrawMs;
  uint32_t _fps;
  uint32_t _frameCounter;
  uint32_t _fpsWindowMs;
  DashboardSnapshot _prev;

  HistoryWindow<float, 96> _rpmGraph;
  HistoryWindow<float, 96> _afrGraph;
  HistoryWindow<float, 96> _tempGraph;
  HistoryWindow<float, 96> _battGraph;

  void runBootAnimation() {
    _tft.fillScreen(cfg::COL_BG);
    _tft.setTextDatum(MC_DATUM);
    _tft.setTextColor(cfg::COL_ACCENT, cfg::COL_BG);
    _tft.drawString("MOTOR ECU DASHBOARD", cfg::DISPLAY_W / 2, cfg::DISPLAY_H / 2 - 38, 4);
    _tft.setTextColor(cfg::COL_GRAY, cfg::COL_BG);
    _tft.drawString("ESP32 / ESP32-S3 + TFT_eSPI + K-Line scaffold", cfg::DISPLAY_W / 2, cfg::DISPLAY_H / 2 - 6, 2);
    _tft.drawRoundRect(120, cfg::DISPLAY_H / 2 + 24, cfg::DISPLAY_W - 240, 18, 8, cfg::COL_DARK);

    uint32_t start = millis();
    uint32_t lastFrame = 0;
    while ((millis() - start) < cfg::BOOT_MS) {
      if ((millis() - lastFrame) < 16) {
        yield();
        continue;
      }
      lastFrame = millis();
      float p = clampValue((millis() - start) / static_cast<float>(cfg::BOOT_MS), 0.0f, 1.0f);
      int fillW = static_cast<int>((cfg::DISPLAY_W - 244) * p);
      _tft.fillRoundRect(122, cfg::DISPLAY_H / 2 + 26, fillW, 14, 6, cfg::COL_ACCENT2);
      _tft.fillCircle(122 + fillW, cfg::DISPLAY_H / 2 + 33, 5, cfg::COL_YELLOW);
      yield();
    }
    _tft.fillScreen(cfg::COL_BG);
  }

  void drawStaticLayout() {
    _tft.fillScreen(cfg::COL_BG);
    drawStatusPanel(_prev);

    switch (_mode) {
      case UiMode::Dashboard:
        drawDashboardLayout();
        break;
      case UiMode::EcuMapping:
        drawEcuMappingLayout();
        break;
      case UiMode::Diagnostic:
        drawDiagnosticLayout();
        break;
      case UiMode::SensorMonitor:
        drawSensorMonitorLayout();
        break;
    }
  }

  void drawStatusPanel(const DashboardSnapshot& s) {
    _tft.fillRect(0, 0, cfg::DISPLAY_W, 48, cfg::COL_PANEL);
    _tft.drawFastHLine(0, 47, cfg::DISPLAY_W, cfg::COL_DARK);
    drawModeTabs();
    _tft.setTextDatum(TL_DATUM);

    char left[64];
    snprintf(left, sizeof(left), "ECU %s", ecuStateText(s.ecuState));
    _tft.setTextDatum(TL_DATUM);
    _tft.setTextColor(s.ecuOnline ? cfg::COL_GREEN : cfg::COL_GRAY, cfg::COL_PANEL);
    _tft.drawString(left, 560, 8, 2);

    char right[64];
    snprintf(right, sizeof(right), "FPS %lu  HEAP %lu",
             static_cast<unsigned long>(_fps),
             static_cast<unsigned long>(ESP.getFreeHeap()));
    _tft.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    _tft.drawString(right, 560, 26, 1);

    uint16_t warnCol = warningColor(s.warningLevel, s.warningReason);
    _tft.setTextColor(warnCol, cfg::COL_PANEL);
    _tft.drawRightString(warningReasonText(s.warningReason), cfg::DISPLAY_W - 10, 8, 2);

    char up[16];
    formatUptime(up, sizeof(up), millis() / 1000UL);
    _tft.setTextColor(cfg::COL_MUTED, cfg::COL_PANEL);
    _tft.drawRightString(up, cfg::DISPLAY_W - 10, 26, 1);
  }

  void drawModeTabs() {
    const int tabY = 6;
    const int tabH = 32;
    const int tabW = 132;
    const int tabGap = 6;
    const int startX = 12;

    const char* labels[4] = { "DASH", "MAP", "DIAG", "MON" };
    _tft.setTextDatum(MC_DATUM);
    for (int i = 0; i < 4; ++i) {
      int x = startX + i * (tabW + tabGap);
      bool active = (static_cast<int>(_mode) == i);
      uint16_t bg = active ? cfg::COL_ACCENT2 : cfg::COL_PANEL_2;
      uint16_t fg = active ? cfg::COL_WHITE : cfg::COL_GRAY;
      uint16_t border = active ? cfg::COL_ACCENT : cfg::COL_DARK;
      _tft.fillRoundRect(x, tabY, tabW, tabH, 6, bg);
      _tft.drawRoundRect(x, tabY, tabW, tabH, 6, border);
      _tft.setTextDatum(MC_DATUM);
      _tft.setTextColor(fg, bg);
      _tft.drawString(labels[i], x + tabW / 2, tabY + tabH / 2, 2);
    }
  }

  void drawPanelFrame(int x, int y, int w, int h, const char* title, uint16_t accent) {
    _tft.fillRoundRect(x, y, w, h, 8, cfg::COL_PANEL);
    _tft.drawRoundRect(x, y, w, h, 8, accent);
    _tft.fillRect(x + 1, y + 1, w - 2, 10, cfg::COL_PANEL_2);
    _tft.setTextDatum(TL_DATUM);
    _tft.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL_2);
    _tft.drawString(title, x + 8, y + 2, 2);
  }

  void clearPanelInner(int x, int y, int w, int h) {
    _tft.fillRect(x + 2, y + 12, w - 4, h - 14, cfg::COL_PANEL);
  }

  uint16_t rpmColor(float rpm) const {
    if (rpm >= 10000.0f) return cfg::COL_RED;
    if (rpm >= 8000.0f)  return cfg::COL_ORANGE;
    if (rpm >= 6000.0f)  return cfg::COL_YELLOW;
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

  void drawProgressBar(int x, int y, int w, int h, float pct, uint16_t color) {
    pct = clampValue(pct, 0.0f, 100.0f);
    int fillW = static_cast<int>((w - 2) * pct / 100.0f);
    _tft.drawRoundRect(x, y, w, h, 3, cfg::COL_DARK);
    if (fillW > 0) {
      _tft.fillRoundRect(x + 1, y + 1, fillW, h - 2, 3, color);
    }
  }

  void drawVerticalGauge(int x, int y, int w, int h, float pct, uint16_t color) {
    pct = clampValue(pct, 0.0f, 100.0f);
    int fillH = static_cast<int>((h - 2) * pct / 100.0f);
    _tft.drawRoundRect(x, y, w, h, 4, cfg::COL_DARK);
    if (fillH > 0) {
      _tft.fillRect(x + 1, y + h - 1 - fillH, w - 2, fillH, color);
    }
  }

  void drawSensorChip(int x, int y, const char* label, SensorStatus status) {
    uint16_t col = sensorStatusColor(status);
    _tft.fillRoundRect(x, y, 40, 16, 4, cfg::COL_PANEL_2);
    _tft.drawRoundRect(x, y, 40, 16, 4, col);
    _tft.setTextDatum(MC_DATUM);
    _tft.setTextColor(col, cfg::COL_PANEL_2);
    _tft.drawString(label, x + 20, y + 8, 1);
  }

  void drawDashboardLayout() {
    drawPanelFrame(10, 56, 370, 220, "SPEED / STATUS", cfg::COL_ACCENT);
    drawPanelFrame(392, 56, 398, 220, "ECU / SENSOR ANALYSIS", cfg::COL_ACCENT2);
    drawPanelFrame(10, 292, 780, 180, "REAL-TIME MINI GRAPHS", cfg::COL_ACCENT);
  }

  void drawEcuMappingLayout() {
    drawPanelFrame(10, 56, 780, 416, "ECU MAPPING TABLE", cfg::COL_ACCENT);
  }

  void drawDiagnosticLayout() {
    drawPanelFrame(10, 56, 780, 416, "DIAGNOSTIC VIEW", cfg::COL_ACCENT2);
  }

  void drawSensorMonitorLayout() {
    drawPanelFrame(10, 56, 780, 416, "SENSOR MONITOR", cfg::COL_ACCENT);
  }

  void drawDashboardMode(const DashboardSnapshot& s, const SensorHub& hub, uint32_t now) {
    bool force = _forceDashboardRender;
    bool speedDirty = force ||
                      fabsf(s.speedKmh - _prev.speedKmh) >= 0.5f ||
                      (s.engineHealth != _prev.engineHealth) ||
                      (s.engineRunning != _prev.engineRunning) ||
                      fabsf(s.fuelInstantKmL - _prev.fuelInstantKmL) >= 0.2f ||
                      fabsf(s.fuelAverageKmL - _prev.fuelAverageKmL) >= 0.2f ||
                      fabsf(s.fuelLPer100Km - _prev.fuelLPer100Km) >= 0.1f;
    bool rpmDirty = force || (abs(static_cast<int>(s.rpm) - static_cast<int>(_prev.rpm)) > 25);
    bool sensorDirty = force ||
                       fabsf(s.afr - _prev.afr) >= 0.05f ||
                       fabsf(s.engineTempC - _prev.engineTempC) >= 0.5f ||
                       fabsf(s.batteryVolt - _prev.batteryVolt) >= 0.05f ||
                       fabsf(s.fuelPercent - _prev.fuelPercent) >= 0.5f ||
                       fabsf(s.fuelInstantKmL - _prev.fuelInstantKmL) >= 0.2f ||
                       fabsf(s.fuelAverageKmL - _prev.fuelAverageKmL) >= 0.2f ||
                       fabsf(s.fuelLPer100Km - _prev.fuelLPer100Km) >= 0.1f ||
                       fabsf(s.mapKpa - _prev.mapKpa) >= 1.0f ||
                       fabsf(s.throttlePct - _prev.throttlePct) >= 1.0f;
    bool graphsDirty = force || ((now - _lastGraphMs) >= cfg::UI_GRAPH_MS);
    bool statusDirty = force ||
                       (s.ecuState != _prev.ecuState) ||
                       (s.ecuOnline != _prev.ecuOnline) ||
                       (s.warningLevel != _prev.warningLevel) ||
                       (s.warningReason != _prev.warningReason) ||
                       (s.warningLevel != WarningLevel::None) ||
                       ((now - _lastStatusMs) >= 500UL);

    if (graphsDirty) {
      _lastGraphMs = now;
      _rpmGraph.push(s.rpm);
      _afrGraph.push(s.afr);
      _tempGraph.push(s.engineTempC);
      _battGraph.push(s.batteryVolt);
      drawGraphs(s, true);
    }

    if (speedDirty) drawSpeedTile(s, hub);
    if (rpmDirty || sensorDirty) drawEcuSensorBlock(s, hub);
    if (statusDirty) {
      drawWarningPanel(s);
      _lastStatusMs = now;
    }

    _prev = s;
    _layoutDirty = false;
    _forceDashboardRender = false;
  }

  void drawSpeedTile(const DashboardSnapshot& s, const SensorHub& hub) {
    const int x = 10;
    const int y = 56;
    const int w = 370;
    const int h = 220;

    clearPanelInner(x, y, w, h);
    _tft.setTextDatum(MC_DATUM);

    char speed[16];
    snprintf(speed, sizeof(speed), "%.0f", s.speedKmh);
    _tft.setTextColor(cfg::COL_WHITE, cfg::COL_PANEL);
    _tft.drawCentreString(speed, x + w / 2, y + 74, 7);
    _tft.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    _tft.drawCentreString("km/h", x + w / 2, y + 145, 4);

    const char* driveText = s.engineRunning ? "ENGINE RUNNING" : "ENGINE OFF";
    _tft.setTextColor(cfg::COL_MUTED, cfg::COL_PANEL);
    _tft.drawCentreString(driveText, x + w / 2, y + 170, 2);

    float healthPct = s.engineHealth;
    drawProgressBar(x + 20, y + 188, 330, 10, healthPct, healthColor(healthPct));
    _tft.setTextDatum(TL_DATUM);
    _tft.setTextColor(healthColor(healthPct), cfg::COL_PANEL);
    _tft.drawString("ENGINE HEALTH", x + 20, y + 174, 2);
    char healthTxt[12];
    snprintf(healthTxt, sizeof(healthTxt), "%.0f%%", healthPct);
    _tft.drawRightString(healthTxt, x + w - 18, y + 172, 2);

    char fuelTxt[32];
    snprintf(fuelTxt, sizeof(fuelTxt), "%.1f km/L", s.fuelAverageKmL);
    _tft.setTextColor(cfg::COL_ACCENT, cfg::COL_PANEL);
    _tft.drawString(fuelTxt, x + 20, y + 196, 2);
    char l100Txt[24];
    snprintf(l100Txt, sizeof(l100Txt), "%.1f L/100", s.fuelLPer100Km);
    _tft.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    _tft.drawRightString(l100Txt, x + w - 18, y + 196, 2);

    // Sensor status chips
    drawSensorChip(x + 8, y + 12, "SPD", hub.sample(SensorId::Speed).status);
    drawSensorChip(x + 52, y + 12, "RPM", hub.sample(SensorId::Rpm).status);
    drawSensorChip(x + 96, y + 12, "AFR", hub.sample(SensorId::Afr).status);
    drawSensorChip(x + 140, y + 12, "TMP", hub.sample(SensorId::EngineTemp).status);
    drawSensorChip(x + 184, y + 12, "BAT", hub.sample(SensorId::Battery).status);
    drawSensorChip(x + 228, y + 12, "FUEL", hub.sample(SensorId::FuelLevel).status);
    drawSensorChip(x + 282, y + 12, "TPS", hub.sample(SensorId::Tps).status);
    drawSensorChip(x + 326, y + 12, "MAP", hub.sample(SensorId::Map).status);
  }

  void drawRPMBar(int x, int y, int w, const DashboardSnapshot& s, const SensorHub& hub) {
    (void)hub;
    const int h = 62;
    clearPanelInner(x, y, w, h);

    float pct = clampValue(s.rpm / 12000.0f * 100.0f, 0.0f, 100.0f);
    uint16_t col = rpmColor(s.rpm);
    _tft.setTextDatum(TL_DATUM);
    _tft.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    _tft.drawString("RPM", x + 12, y + 6, 2);

    char rpmTxt[16];
    snprintf(rpmTxt, sizeof(rpmTxt), "%u", static_cast<unsigned>(s.rpm));
    _tft.setTextColor(col, cfg::COL_PANEL);
    _tft.drawRightString(rpmTxt, x + w - 12, y + 4, 4);

    drawProgressBar(x + 12, y + 38, w - 24, 10, pct, col);
    char ckpTxt[24];
    snprintf(ckpTxt, sizeof(ckpTxt), "CKP %.1f Hz", s.ckpHz);
    _tft.setTextColor(cfg::COL_MUTED, cfg::COL_PANEL);
    _tft.drawString(ckpTxt, x + 12, y + 48, 1);
  }

  void drawEcuSensorBlock(const DashboardSnapshot& s, const SensorHub& hub) {
    const int x = 392;
    const int y = 56;
    const int w = 398;
    const int h = 220;
    clearPanelInner(x, y, w, h);

    drawRPMBar(x + 4, y + 4, w - 8, s, hub);

    drawValueCard(x + 10, y + 76, 184, 64, "AFR", afrColor(s.afr), s.afr, "AFR");
    drawValueCard(x + 204, y + 76, 184, 64, "TEMP", tempColor(s.engineTempC), s.engineTempC, "C");
    drawValueCard(x + 10, y + 146, 184, 64, "BATT", voltColor(s.batteryVolt), s.batteryVolt, "V");
    drawValueCard(x + 204, y + 146, 184, 64, "FUEL", fuelColor(s.fuelPercent), s.fuelPercent, "%");
  }

  void drawValueCard(int x, int y, int w, int h, const char* title, uint16_t color, float value, const char* unit) {
    _tft.fillRoundRect(x, y, w, h, 6, cfg::COL_PANEL_2);
    _tft.drawRoundRect(x, y, w, h, 6, color);
    _tft.setTextDatum(TL_DATUM);
    _tft.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL_2);
    _tft.drawString(title, x + 8, y + 4, 2);

    char val[24];
    if (strcmp(unit, "AFR") == 0) {
      snprintf(val, sizeof(val), "%.2f", value);
    } else if (strcmp(unit, "%") == 0) {
      snprintf(val, sizeof(val), "%.0f%%", value);
    } else if (strcmp(unit, "V") == 0) {
      snprintf(val, sizeof(val), "%.2fV", value);
    } else if (strcmp(unit, "C") == 0) {
      snprintf(val, sizeof(val), "%.1fC", value);
    } else {
      snprintf(val, sizeof(val), "%.1f", value);
    }

    _tft.setTextDatum(MC_DATUM);
    _tft.setTextColor(color, cfg::COL_PANEL_2);
    _tft.drawCentreString(val, x + w / 2, y + 34, 4);
  }

  template <size_t N>
  void drawMiniGraph(int x, int y, int w, int h, const HistoryWindow<float, N>& history,
                     const char* label, float minY, float maxY, uint16_t color, const char* valueText) {
    _tft.fillRoundRect(x, y, w, h, 6, cfg::COL_PANEL);
    _tft.drawRoundRect(x, y, w, h, 6, cfg::COL_DARK);
    _tft.setTextDatum(TL_DATUM);
    _tft.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    _tft.drawString(label, x + 8, y + 4, 2);
    _tft.setTextColor(cfg::COL_MUTED, cfg::COL_PANEL);
    _tft.drawRightString(valueText, x + w - 8, y + 4, 1);

    int gx = x + 6;
    int gy = y + 18;
    int gw = w - 12;
    int gh = h - 24;

    for (int i = 0; i <= 4; ++i) {
      int yy = gy + (gh * i) / 4;
      _tft.drawFastHLine(gx, yy, gw, cfg::COL_DARK);
    }
    for (int i = 0; i <= 5; ++i) {
      int xx = gx + (gw * i) / 5;
      _tft.drawFastVLine(xx, gy, gh, cfg::COL_DARK);
    }

    if (history.count() < 2) return;
    float span = maxY - minY;
    if (fabsf(span) < 0.0001f) span = 1.0f;
    int lastX = gx;
    int lastY = gy + gh;
    size_t count = history.count();
    for (size_t i = 0; i < count; ++i) {
      float v = history.at(i);
      float t = (v - minY) / span;
      t = clampValue(t, 0.0f, 1.0f);
      int px = gx + static_cast<int>((gw - 1) * (i / static_cast<float>(count - 1)));
      int py = gy + gh - static_cast<int>(t * gh);
      if (i > 0) {
        _tft.drawLine(lastX, lastY, px, py, color);
      }
      lastX = px;
      lastY = py;
    }
  }

  void drawGraphs(const DashboardSnapshot& s, bool force) {
    (void)force;
    const int x1 = 10;
    const int x2 = 400;
    const int w = 380;
    const int h = 82;
    const int yTop = 292;
    const int yBottom = 382;

    char rpmTxt[24];
    char afrTxt[24];
    char tempTxt[24];
    char battTxt[24];
    snprintf(rpmTxt, sizeof(rpmTxt), "%u rpm", static_cast<unsigned>(s.rpm));
    snprintf(afrTxt, sizeof(afrTxt), "%.2f AFR", s.afr);
    snprintf(tempTxt, sizeof(tempTxt), "%.1f C", s.engineTempC);
    snprintf(battTxt, sizeof(battTxt), "%.2f V", s.batteryVolt);

    drawMiniGraph(x1, yTop, w, h, _rpmGraph, "RPM", 0.0f, 12000.0f, rpmColor(s.rpm), rpmTxt);
    drawMiniGraph(x2, yTop, w, h, _afrGraph, "AFR", 10.0f, 18.5f, afrColor(s.afr), afrTxt);
    drawMiniGraph(x1, yBottom, w, h, _tempGraph, "TEMP", 0.0f, 120.0f, tempColor(s.engineTempC), tempTxt);
    drawMiniGraph(x2, yBottom, w, h, _battGraph, "BATT", 11.0f, 15.8f, voltColor(s.batteryVolt), battTxt);
  }

  void drawWarningPanel(const DashboardSnapshot& s) {
    const int x = 250;
    const int y = 270;
    const int w = 300;
    const int h = 20;
    uint16_t col = warningColor(s.warningLevel, s.warningReason);

    if (s.warningLevel != WarningLevel::None && !_warningBlink) {
      _tft.fillRoundRect(x, y, w, h, 6, cfg::COL_PANEL);
      _tft.drawRoundRect(x, y, w, h, 6, cfg::COL_PANEL);
      return;
    }

    _tft.fillRoundRect(x, y, w, h, 6, cfg::COL_PANEL_2);
    _tft.drawRoundRect(x, y, w, h, 6, col);
    _tft.setTextDatum(MC_DATUM);
    _tft.setTextColor(col, cfg::COL_PANEL_2);
    _tft.drawString(warningReasonText(s.warningReason), x + w / 2, y + 10, 2);
  }

  void drawECUMappingMode(const DashboardSnapshot& s, const SensorHub& hub, uint32_t now) {
    (void)hub;
    clearPanelInner(10, 56, 780, 416);

    const int startX = 76;
    const int startY = 92;
    const int cellW = 80;
    const int cellH = 56;

    _tft.setTextDatum(MC_DATUM);
    _tft.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    for (int c = 0; c < 8; ++c) {
      char rpmTxt[12];
      snprintf(rpmTxt, sizeof(rpmTxt), "%u", static_cast<unsigned>(ECU_RPM_BANDS[c]));
      _tft.drawCentreString(rpmTxt, startX + c * cellW + cellW / 2, startY - 22, 2);
    }
    for (int r = 0; r < 5; ++r) {
      char tpsTxt[8];
      snprintf(tpsTxt, sizeof(tpsTxt), "%u%%", ECU_TPS_BANDS[r]);
      _tft.drawCentreString(tpsTxt, startX - 34, startY + r * cellH + cellH / 2, 2);
    }

    int activeCol = 0;
    int activeRow = 0;
    float rpm = static_cast<float>(s.rpm);
    float tps = s.throttlePct;
    for (int i = 0; i < 7; ++i) {
      if (rpm >= ECU_RPM_BANDS[i + 1]) activeCol = i + 1;
    }
    if (tps >= 90.0f) activeRow = 4;
    else if (tps >= 70.0f) activeRow = 3;
    else if (tps >= 45.0f) activeRow = 2;
    else if (tps >= 15.0f) activeRow = 1;
    else activeRow = 0;

    for (int r = 0; r < 5; ++r) {
      for (int c = 0; c < 8; ++c) {
        float afrTarget = ECU_MAP_DUMMY[r][c];
        uint16_t col = afrColor(afrTarget);
        int x = startX + c * cellW;
        int y = startY + r * cellH;
        bool active = (r == activeRow && c == activeCol);
        _tft.fillRoundRect(x, y, cellW - 4, cellH - 4, 5, active ? cfg::COL_ACCENT2 : cfg::COL_PANEL_2);
        _tft.drawRoundRect(x, y, cellW - 4, cellH - 4, 5, active ? cfg::COL_WHITE : col);
        _tft.setTextColor(active ? cfg::COL_WHITE : col, active ? cfg::COL_ACCENT2 : cfg::COL_PANEL_2);
        char cellTxt[16];
        snprintf(cellTxt, sizeof(cellTxt), "%.1f", afrTarget);
        _tft.drawCentreString(cellTxt, x + (cellW - 4) / 2, y + 16, 4);
        _tft.setTextColor(cfg::COL_GRAY, active ? cfg::COL_ACCENT2 : cfg::COL_PANEL_2);
        _tft.drawCentreString(active ? "ACTIVE" : "TARGET", x + (cellW - 4) / 2, y + 38, 1);
      }
    }

    _tft.setTextDatum(TL_DATUM);
    _tft.fillRoundRect(712, 92, 72, 280, 6, cfg::COL_PANEL_2);
    _tft.drawRoundRect(712, 92, 72, 280, 6, cfg::COL_DARK);
    _tft.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL_2);
    _tft.drawString("LIVE", 720, 100, 2);
    char buf[24];
    snprintf(buf, sizeof(buf), "RPM %u", static_cast<unsigned>(s.rpm));
    _tft.drawString(buf, 720, 130, 2);
    snprintf(buf, sizeof(buf), "TPS %.0f%%", s.throttlePct);
    _tft.drawString(buf, 720, 158, 2);
    snprintf(buf, sizeof(buf), "AFR %.2f", s.afr);
    _tft.drawString(buf, 720, 186, 2);
    snprintf(buf, sizeof(buf), "ECU %s", s.ecuOnline ? "ON" : "OFF");
    _tft.drawString(buf, 720, 214, 2);
    _tft.drawString("Dummy map data", 720, 252, 1);
    _tft.drawString("ready for ECU", 720, 268, 1);
    _tft.drawString("packet parse", 720, 284, 1);
    _tft.drawString("extension", 720, 300, 1);

    char modeTxt[32];
    snprintf(modeTxt, sizeof(modeTxt), "MODE %s", uiModeText(_mode));
    _tft.setTextColor(cfg::COL_WHITE, cfg::COL_PANEL);
    _tft.drawString(modeTxt, 18, 74, 2);
    char liveTxt[32];
    snprintf(liveTxt, sizeof(liveTxt), "ACTIVE %u / %u", activeRow + 1, activeCol + 1);
    _tft.drawString(liveTxt, 18, 100, 2);

    (void)now;
  }

  void drawDiagnosticMode(const DashboardSnapshot& s, const SensorHub& hub, uint32_t now) {
    clearPanelInner(10, 56, 780, 416);
    const int rowsPerPage = 14;
    const size_t count = hub.sensorCount();
    const size_t pageCount = (count + rowsPerPage - 1) / rowsPerPage;
    const size_t page = pageCount ? ((now / cfg::PAGE_SWITCH_MS) % pageCount) : 0;
    const size_t start = page * rowsPerPage;

    _tft.setTextDatum(TL_DATUM);
    _tft.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    _tft.drawString("SENSOR", 20, 78, 2);
    _tft.drawString("VALUE", 200, 78, 2);
    _tft.drawString("STATUS", 330, 78, 2);
    _tft.drawString("AGE", 430, 78, 2);
    _tft.drawString("TIMEOUT", 530, 78, 2);
    _tft.drawString("SRC", 650, 78, 2);
    _tft.drawString("PAGE", 720, 78, 2);

    char pageTxt[16];
    snprintf(pageTxt, sizeof(pageTxt), "%u/%u", static_cast<unsigned>(page + 1), static_cast<unsigned>(pageCount ? pageCount : 1));
    _tft.drawString(pageTxt, 720, 100, 2);

    for (int row = 0; row < rowsPerPage; ++row) {
      size_t idx = start + row;
      if (idx >= count) break;
      SensorId id = static_cast<SensorId>(idx);
      const SensorSample& smp = hub.sample(id);
      int y = 112 + row * 22;
      _tft.fillRect(16, y - 2, 760, 20, (row % 2 == 0) ? cfg::COL_PANEL : cfg::COL_PANEL_2);

      char value[32];
      char age[16];
      char timeout[16];
      formatSensorValue(id, smp, value, sizeof(value));
      formatAge(age, sizeof(age), (smp.lastGoodMs == 0) ? 0 : (now - smp.lastGoodMs));
      formatDuration(timeout, sizeof(timeout), smp.timeoutMs);

      _tft.setTextColor(cfg::COL_WHITE, (row % 2 == 0) ? cfg::COL_PANEL : cfg::COL_PANEL_2);
      _tft.drawString(hub.sensorName(id), 20, y, 2);
      _tft.drawString(value, 200, y, 2);
      _tft.setTextColor(sensorStatusColor(smp.status), (row % 2 == 0) ? cfg::COL_PANEL : cfg::COL_PANEL_2);
      _tft.drawString(sensorStatusText(smp.status), 330, y, 2);
      _tft.setTextColor(cfg::COL_GRAY, (row % 2 == 0) ? cfg::COL_PANEL : cfg::COL_PANEL_2);
      _tft.drawString(age, 430, y, 2);
      _tft.drawString(timeout, 530, y, 2);
      _tft.drawString(sensorSourceText(smp.source), 650, y, 2);
    }

    _tft.setTextColor(cfg::COL_MUTED, cfg::COL_PANEL);
    _tft.drawString("ECU communication status and packet validation remain visible in top bar.", 20, 396, 1);
    _tft.drawString("Rows auto-page every few seconds for all sensors.", 20, 412, 1);

    (void)s;
  }

  void drawSensorMonitorMode(const DashboardSnapshot& s, const SensorHub& hub, uint32_t now) {
    clearPanelInner(10, 56, 780, 416);
    const int rowsPerPage = 12;
    const size_t count = hub.sensorCount();
    const size_t pageCount = (count + rowsPerPage - 1) / rowsPerPage;
    const size_t page = pageCount ? ((now / cfg::PAGE_SWITCH_MS) % pageCount) : 0;
    const size_t start = page * rowsPerPage;

    _tft.setTextDatum(TL_DATUM);
    _tft.setTextColor(cfg::COL_GRAY, cfg::COL_PANEL);
    _tft.drawString("SENSOR", 18, 78, 2);
    _tft.drawString("RAW", 176, 78, 2);
    _tft.drawString("CONV", 266, 78, 2);
    _tft.drawString("MIN", 356, 78, 2);
    _tft.drawString("MAX", 446, 78, 2);
    _tft.drawString("AVG", 536, 78, 2);
    _tft.drawString("FIL", 626, 78, 2);
    _tft.drawString("ST", 720, 78, 2);

    char pageTxt[16];
    snprintf(pageTxt, sizeof(pageTxt), "%u/%u", static_cast<unsigned>(page + 1), static_cast<unsigned>(pageCount ? pageCount : 1));
    _tft.drawString(pageTxt, 720, 100, 2);

    for (int row = 0; row < rowsPerPage; ++row) {
      size_t idx = start + row;
      if (idx >= count) break;
      SensorId id = static_cast<SensorId>(idx);
      const SensorSample& smp = hub.sample(id);
      int y = 112 + row * 22;
      _tft.fillRect(16, y - 2, 760, 20, (row % 2 == 0) ? cfg::COL_PANEL : cfg::COL_PANEL_2);

      char rawTxt[24];
      char convTxt[24];
      char minTxt[24];
      char maxTxt[24];
      char avgTxt[24];
      char filTxt[24];
      formatRawValue(id, smp, rawTxt, sizeof(rawTxt));
      formatSensorValue(id, smp, convTxt, sizeof(convTxt));
      snprintf(minTxt, sizeof(minTxt), "%.1f", smp.min);
      snprintf(maxTxt, sizeof(maxTxt), "%.1f", smp.max);
      snprintf(avgTxt, sizeof(avgTxt), "%.1f", smp.avg);
      snprintf(filTxt, sizeof(filTxt), "%.1f", smp.filtered);

      _tft.setTextColor(cfg::COL_WHITE, (row % 2 == 0) ? cfg::COL_PANEL : cfg::COL_PANEL_2);
      _tft.drawString(hub.sensorName(id), 18, y, 2);
      _tft.drawString(rawTxt, 176, y, 2);
      _tft.drawString(convTxt, 266, y, 2);
      _tft.drawString(minTxt, 356, y, 2);
      _tft.drawString(maxTxt, 446, y, 2);
      _tft.drawString(avgTxt, 536, y, 2);
      _tft.drawString(filTxt, 626, y, 2);
      _tft.setTextColor(sensorStatusColor(smp.status), (row % 2 == 0) ? cfg::COL_PANEL : cfg::COL_PANEL_2);
      _tft.drawString(sensorStatusText(smp.status), 720, y, 2);
    }

    _tft.setTextColor(cfg::COL_MUTED, cfg::COL_PANEL);
    _tft.drawString("Raw ADC / raw counter / filtered and statistics shown for each channel.", 20, 396, 1);
    _tft.drawString("Use this page to tune calibration and check noise rejection.", 20, 412, 1);

    (void)s;
  }
};

// ============================================================
// Application orchestrator
// ============================================================
class DashboardApp {
public:
  bool begin() {
    Serial.begin(115200);
    while (!Serial && (millis() < 2000UL)) {
      yield();
    }

    if (pinActive(cfg::PIN_MODE_BUTTON)) {
      pinMode(static_cast<uint8_t>(cfg::PIN_MODE_BUTTON), INPUT_PULLUP);
    }

    if (cfg::DEBUG_MODE) {
      Serial.println();
      Serial.println("==================================================");
      Serial.println("ESP32 / ESP32-S3 Motorcycle ECU Dashboard v2");
      Serial.println("Single-file production base");
      Serial.println("==================================================");
      Serial.println("[SEC] Wi-Fi credentials are intentionally blank");
      Serial.println("[SEC] Use provisioning / NVS / secure storage later");
    }

    initWatchdog();

    _sensorHub.setSimulationMode(cfg::SIMULATION_MODE);
    _sensorHub.setEcuEnabled(cfg::ECU_MANAGER_ENABLED);
    _sensorHub.begin();

    _kline.begin(cfg::PIN_KLINE_RX, cfg::PIN_KLINE_TX, cfg::KLINE_BAUD);
    _kline.enable(cfg::ECU_MANAGER_ENABLED);
    _ecuRequests.begin(cfg::ECU_MANAGER_ENABLED);

    _fuelEstimator.reset();
    _engineAnalyzer.reset();

    _ui.begin();
    _ui.setMode(_mode);

    printConfig();
    return true;
  }

  void update() {
    feedWatchdog();
    pollModeInput();
    pollTouchPlaceholder();

    uint32_t startUs = micros();
    updateSensors();
    uint32_t sensorUs = micros() - startUs;

    startUs = micros();
    updateEngineAnalysis();
    uint32_t analysisUs = micros() - startUs;

    startUs = micros();
    updateWarnings(_frame);
    uint32_t warningUs = micros() - startUs;

    startUs = micros();
    updateDisplay(_frame);
    uint32_t uiUs = micros() - startUs;

    _framesSinceDebug++;
    if (cfg::DEBUG_MODE && ((millis() - _lastDebugMs) >= cfg::DEBUG_MS)) {
      _lastDebugMs = millis();
      printDebug(_frame, sensorUs, analysisUs, warningUs, uiUs);
      _framesSinceDebug = 0;
    }
  }

private:
  SensorHub _sensorHub;
  FuelEstimator _fuelEstimator;
  EngineAnalyzer _engineAnalyzer;
  KLineManager _kline;
  ECURequestManager _ecuRequests;
  DashboardUI _ui;

  UiMode _mode = UiMode::Dashboard;
  WarningLevel _warningLevel = WarningLevel::None;
  WarningReason _warningReason = WarningReason::None;
  DashboardSnapshot _frame;

  uint32_t _lastDebugMs = 0;
  uint32_t _framesSinceDebug = 0;
  DebounceTracker _modeButtonDebounce;

  void printConfig() {
    if (!cfg::DEBUG_MODE) return;
    Serial.printf("[CFG] MODE_BUTTON=%d KLINE_RX=%d KLINE_TX=%d\n",
                  cfg::PIN_MODE_BUTTON, cfg::PIN_KLINE_RX, cfg::PIN_KLINE_TX);
    Serial.printf("[CFG] TFT %ux%u rotation=%u\n",
                  cfg::DISPLAY_W, cfg::DISPLAY_H, cfg::DISPLAY_ROTATION);
    Serial.printf("[CFG] SIMULATION=%s ECU_MANAGER=%s\n",
                  yesNo(cfg::SIMULATION_MODE), yesNo(cfg::ECU_MANAGER_ENABLED));
    Serial.println("[NOTE] RPM and injector inputs require optocoupler / isolation");
    Serial.println("[NOTE] Battery ADC must use divider + protection + TVS");
    Serial.println("[NOTE] K-Line must use MC33290 or L9637 transceiver");
  }

  void pollModeInput() {
    if (!pinActive(cfg::PIN_MODE_BUTTON)) return;
    bool rawPressed = (digitalRead(static_cast<uint8_t>(cfg::PIN_MODE_BUTTON)) == LOW);
    if (debounceFilter(rawPressed, _modeButtonDebounce, millis(), 40)) {
      if (_modeButtonDebounce.stableState) {
        cycleMode();
      }
    }
  }

  void pollTouchPlaceholder() {
    // Placeholder for future touch controller integration.
    // Keep non-blocking; return immediately until touch hardware is wired.
  }

  void cycleMode() {
    switch (_mode) {
      case UiMode::Dashboard:     _mode = UiMode::EcuMapping; break;
      case UiMode::EcuMapping:    _mode = UiMode::Diagnostic; break;
      case UiMode::Diagnostic:    _mode = UiMode::SensorMonitor; break;
      case UiMode::SensorMonitor: _mode = UiMode::Dashboard; break;
    }
    _ui.setMode(_mode);
  }

  void updateSensors() {
    _kline.update();
    _sensorHub.setEcuEnabled(cfg::ECU_MANAGER_ENABLED);
    _sensorHub.setEcuOnline(_kline.isConnected());
    _ecuRequests.update(_kline, _sensorHub, _mode);
    _sensorHub.update();

    _frame = _sensorHub.snapshot();
    _frame.mode = _mode;
    _frame.ecuEnabled = cfg::ECU_MANAGER_ENABLED;
    _frame.ecuOnline = _kline.isConnected();
    _frame.ecuState = _kline.state();
    _frame.timestampMs = millis();
  }

  void updateEngineAnalysis() {
    _fuelEstimator.calculateFuelConsumption(_frame);
    _engineAnalyzer.calculateEngineHealth(_frame, _sensorHub);
    _frame.engineHealth = _engineAnalyzer.health();
  }

  void updateWarnings(const DashboardSnapshot& s) {
    _warningLevel = WarningLevel::None;
    _warningReason = WarningReason::None;

    if (s.ecuEnabled && !s.ecuOnline && s.engineRunning) {
      _warningLevel = WarningLevel::Critical;
      _warningReason = WarningReason::EcuTimeout;
      return;
    }

    if (s.engineTempC > 110.0f) {
      _warningLevel = WarningLevel::Critical;
      _warningReason = WarningReason::Overheat;
      return;
    }

    if (s.batteryVolt < 11.4f && s.batteryVolt > 0.1f) {
      _warningLevel = WarningLevel::Warning;
      _warningReason = WarningReason::BatteryLow;
    }

    if (s.rpm > 1000U && s.afr > 17.0f) {
      _warningLevel = WarningLevel::Warning;
      _warningReason = WarningReason::AfrLean;
    }
    if (s.rpm > 1000U && s.afr > 0.0f && s.afr < 12.0f) {
      _warningLevel = WarningLevel::Warning;
      _warningReason = WarningReason::AfrRich;
    }

    if (s.engineRunning) {
      if (_sensorHub.sample(SensorId::Speed).status == SensorStatus::Offline ||
          _sensorHub.sample(SensorId::Rpm).status == SensorStatus::Offline ||
          _sensorHub.sample(SensorId::Afr).status == SensorStatus::Offline ||
          _sensorHub.sample(SensorId::EngineTemp).status == SensorStatus::Offline ||
          _sensorHub.sample(SensorId::Battery).status == SensorStatus::Offline) {
        _warningLevel = WarningLevel::Warning;
        _warningReason = WarningReason::SensorFault;
      }
    }
  }

  void updateDisplay(const DashboardSnapshot& frame) {
    _ui.setMode(_mode);
    DashboardSnapshot copy = frame;
    copy.mode = _mode;
    copy.warningLevel = _warningLevel;
    copy.warningReason = _warningReason;
    _ui.update(copy, _sensorHub);
  }

  void printDebug(const DashboardSnapshot& s,
                  uint32_t sensorUs,
                  uint32_t analysisUs,
                  uint32_t warningUs,
                  uint32_t uiUs) {
    if (!cfg::DEBUG_MODE) return;

    Serial.printf("[DATA] SPD:%5.1f RPM:%5u AFR:%5.2f TMP:%5.1f BATT:%5.2f FUEL:%5.0f%% HEALTH:%5.0f ECU:%s ONLINE:%s MODE:%s\n",
                  s.speedKmh,
                  static_cast<unsigned>(s.rpm),
                  s.afr,
                  s.engineTempC,
                  s.batteryVolt,
                  s.fuelPercent,
                  s.engineHealth,
                  ecuStateText(s.ecuState),
                  yesNo(s.ecuOnline),
                  uiModeText(s.mode));

    Serial.printf("[PERF] sensor=%luus analysis=%luus warning=%luus ui=%luus fps=%lu heap=%lu psram=%lu\n",
                  static_cast<unsigned long>(sensorUs),
                  static_cast<unsigned long>(analysisUs),
                  static_cast<unsigned long>(warningUs),
                  static_cast<unsigned long>(uiUs),
                  static_cast<unsigned long>(_framesSinceDebug),
                  static_cast<unsigned long>(ESP.getFreeHeap()),
                  static_cast<unsigned long>(ESP.getFreePsram()));
  }

  void initWatchdog() {
#if defined(ARDUINO_ARCH_ESP32)
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    esp_task_wdt_config_t config;
    config.timeout_ms = cfg::WDT_TIMEOUT_MS;
    config.idle_core_mask = 0;
    config.trigger_panic = true;
    esp_task_wdt_init(&config);
  #else
    esp_task_wdt_init(cfg::WDT_TIMEOUT_MS / 1000, true);
  #endif
    esp_task_wdt_add(nullptr);
#endif
  }

  void feedWatchdog() {
#if defined(ARDUINO_ARCH_ESP32)
    esp_task_wdt_reset();
#endif
  }
};

// ============================================================
// Global application instance
// ============================================================
static DashboardApp g_app;

void setup() {
  g_app.begin();
}

void loop() {
  g_app.update();
}
