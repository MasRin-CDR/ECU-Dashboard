#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>

// ============================================================================
// Configuration constants
// ============================================================================
namespace cfg {
constexpr bool DEBUG_MODE      = true;
constexpr bool SIMULATION_MODE = false;

constexpr uint16_t DISPLAY_W        = 800;
constexpr uint16_t DISPLAY_H        = 480;
constexpr uint8_t  DISPLAY_ROTATION = 1;

constexpr uint32_t SENSOR_FAST_MS  = 20;
constexpr uint32_t SENSOR_SLOW_MS  = 120;
constexpr uint32_t UI_DASH_MS      = 33;
constexpr uint32_t UI_TABLE_MS     = 160;
constexpr uint32_t UI_GRAPH_MS     = 100;
constexpr uint32_t DEBUG_MS        = 1000;
constexpr uint32_t BOOT_MS         = 900;
constexpr uint32_t PAGE_SWITCH_MS  = 4500;
constexpr uint32_t FULL_REDRAW_MS  = 5000;
constexpr uint32_t WDT_TIMEOUT_MS  = 8000;
constexpr uint32_t ECU_EXT_TIMEOUT_MS = 2000;
constexpr uint32_t ECU_RECONNECT_DELAY_MS = 8000;

constexpr uint32_t RPM_MIN_EDGE_US   = 650;
constexpr uint32_t SPEED_MIN_EDGE_US = 2500;
constexpr uint32_t RPM_TIMEOUT_US    = 650000;
constexpr uint32_t SPEED_TIMEOUT_US  = 900000;

constexpr uint32_t KLINE_BAUD               = 10400;
constexpr uint32_t KLINE_RX_TIMEOUT_MS      = 500;
constexpr uint32_t KLINE_RETRY_MS           = 5000;
constexpr uint32_t KLINE_POLL_MS            = 25;
constexpr uint32_t KLINE_FAST_LOW_MS        = 25;
constexpr uint32_t KLINE_FAST_HIGH_MS       = 25;
constexpr uint32_t KLINE_5BAUD_BIT_MS       = 200;
constexpr uint32_t KLINE_W4_MS              = 300;
constexpr uint32_t KLINE_STARTCOMM_TMO_MS   = 2000;
constexpr uint32_t KLINE_INTER_BYTE_MS      = 10;
constexpr uint32_t KLINE_ECHO_WINDOW_MS     = 150;
constexpr uint32_t RESCAN_REINIT_DELAY_MS   = 60;

constexpr uint32_t SCAN_PHASE_TMO_MS  = 3000;
constexpr uint32_t SCAN_TOTAL_TMO_MS  = 30000;
constexpr uint32_t CAN_LISTEN_MS      = 2000;

constexpr int16_t PIN_RPM          = 34;
constexpr int16_t PIN_SPEED        = 35;
constexpr int16_t PIN_AFR_ADC      = 36;
constexpr int16_t PIN_TEMP_ADC     = 39;
constexpr int16_t PIN_BATT_ADC     = 32;
constexpr int16_t PIN_FUEL_ADC     = 33;
constexpr int16_t PIN_TPS_ADC      = 25;
constexpr int16_t PIN_MAP_ADC      = 26;
constexpr int16_t PIN_KLINE_RX     = 16;
constexpr int16_t PIN_KLINE_TX     = 17;
constexpr int16_t PIN_CAN_RX       = 22;
constexpr int16_t PIN_CAN_TX       = 21;
constexpr int16_t PIN_MODE_BUTTON  = 27;
constexpr int16_t PIN_SCAN_BUTTON  = 14;

constexpr float ADC_REF_VOLT       = 3.30f;
constexpr float ADC_MAX_RAW        = 4095.0f;
constexpr float BATT_R1_OHM        = 100000.0f;
constexpr float BATT_R2_OHM        = 22000.0f;
constexpr float BATT_DIVIDER_RATIO = (BATT_R1_OHM + BATT_R2_OHM) / BATT_R2_OHM;
constexpr float AFR_DIVIDER_RATIO  = 2.447f;
constexpr float AFR_SENSOR_V_MIN   = 0.05f;
constexpr float AFR_SENSOR_V_MAX   = 1.00f;
constexpr float AFR_MIN_VALUE      = 12.2f;
constexpr float AFR_MAX_VALUE      = 20.0f;
constexpr float TEMP_NTC_FIXED_OHM = 10000.0f;
constexpr float TEMP_NTC_R25_OHM   = 10000.0f;
constexpr float TEMP_NTC_BETA      = 3950.0f;
constexpr float TEMP_OFFSET_C      = 0.0f;

constexpr float WHEEL_CIRCUMFERENCE_M = 1.720f;
constexpr float SPEED_PULSES_PER_REV  = 1.0f;
constexpr float RPM_PULSES_PER_REV    = 1.0f;
constexpr float SPEED_CAL_FACTOR      = 1.0f;
constexpr float RPM_CAL_FACTOR        = 1.0f;
constexpr float INJECTOR_FLOW_CC_MIN  = 125.0f;
constexpr float TANK_CAPACITY_L       = 3.5f;

constexpr float HEALTH_W_TEMP = 0.30f;
constexpr float HEALTH_W_AFR  = 0.25f;
constexpr float HEALTH_W_VOLT = 0.20f;
constexpr float HEALTH_W_RPM  = 0.15f;
constexpr float HEALTH_W_FUEL = 0.10f;

constexpr uint8_t OBD_MODE_LIVE   = 0x01;
constexpr uint8_t OBD_MODE_DTC    = 0x03;
constexpr uint8_t OBD_MODE_CLEAR  = 0x04;
constexpr uint8_t PID_SUPPORTED   = 0x00;
constexpr uint8_t PID_ENGINE_RPM  = 0x0C;
constexpr uint8_t PID_SPEED       = 0x0D;
constexpr uint8_t PID_COOLANT     = 0x05;
constexpr uint8_t PID_TPS         = 0x11;
constexpr uint8_t PID_O2_1        = 0x14;
constexpr uint8_t PID_MAP         = 0x0B;
constexpr uint8_t PID_IAT         = 0x0F;
constexpr uint8_t PID_BATT        = 0x42;
constexpr uint8_t PID_STFT        = 0x06;
constexpr uint8_t PID_LTFT        = 0x07;
constexpr uint8_t KWP_START_COMM  = 0x81;
constexpr uint8_t KWP_READ_DTC    = 0x18;
constexpr uint8_t KWP_CLEAR_DTC   = 0x14;

constexpr uint8_t ECU_ADDR_HONDA    = 0x6A;
constexpr uint8_t ECU_ADDR_YAMAHA   = 0x85;
constexpr uint8_t ECU_ADDR_YAMAHA2  = 0x17;
constexpr uint8_t ECU_ADDR_SUZUKI   = 0x6B;
constexpr uint8_t ECU_ADDR_KAWASAKI = 0x84;
constexpr uint8_t ECU_ADDR_GENERIC  = 0x33;
constexpr uint8_t TESTER_ADDR       = 0xF1;
constexpr uint8_t KLINE_HEADER      = 0x68;

constexpr size_t MAX_ECU_PAYLOAD = 48;
constexpr size_t MAX_ECU_FRAME   = 64;
constexpr size_t MAX_DTC_COUNT   = 20;
constexpr size_t MAX_PIDS        = 16;

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
constexpr uint16_t COL_SCAN_OK   = 0x07E0;
constexpr uint16_t COL_SCAN_TRY  = 0xFFE0;
constexpr uint16_t COL_SCAN_FAIL = 0xF800;

} // namespace cfg

// ============================================================================
// Types and platform data
// ============================================================================
enum class SensorStatus  : uint8_t { Ok=0, Warning, Error, Offline };
enum class SensorSource  : uint8_t { Hardware=0, Ecu, Estimated, Simulation, Offline };
enum class WarningLevel  : uint8_t { None=0, Info, Warning, Critical };
enum class WarningReason : uint8_t { None=0, EcuTimeout, BatteryLow, Overheat, AfrLean, AfrRich, SensorFault };
enum class EcuState      : uint8_t { Disabled=0, Idle, Probing, Connected, Requesting, WaitingResponse, Error, Reconnecting };
enum class MotorModel    : uint8_t {
  Unknown=0, SensorOnly, HondaVario125Kzr, GenericKLine,
  YamahaGeneric, YamahaAlt, SuzukiGeneric, KawasakiGeneric,
  DucatiCan, BmwCan, KtmCan
};
enum class ProtocolType  : uint8_t {
  None=0, HondaKLine, GenericOBD2KLine, CAN,
  YamahaKLine, SuzukiKLine, KawasakiKLine, Unknown,
  ISO9141_5Baud, KWP2000_FastInit, CanBus=CAN
};
enum class ChecksumMode  : uint8_t { Sum8=0, TwosComplement };
enum class InitMethod    : uint8_t { None=0, FastInit, FiveBaudInit, FiveBaud=FiveBaudInit };

enum class BrandId : uint8_t {
  Unknown=0, Honda, Yamaha, Suzuki, Kawasaki, Ducati, BMW, KTM, GenericObd2
};

enum class ScanPhase : uint8_t {
  Idle=0, CanBus125k, CanBus250k, CanBus500k, CanBus1M,
  KLineFastHonda, KLineFastYamaha, KLineFastKawasaki, KLineFastGeneric,
  KLine5BaudHonda, KLine5BaudYamaha, KLine5BaudYamahaAlt, KLine5BaudSuzuki,
  Done, Failed
};

enum class UiMode : uint8_t {
  Scanning=0, ScanResult, Dashboard, EcuMapping, Diagnostic, SensorMonitor
};

enum class SensorId : uint8_t {
  Speed=0, Rpm, Afr, EngineTemp, Battery, FuelLevel,
  Tps, Map, Iat, Eot, InjectorPulse, IgnitionTiming,
  Ckp, FuelPump, RadiatorFan, GearPosition, LeanAngle,
  OilPressure, Knock, AmbientTemp, Humidity, BarometricPressure,
  DtcCode,
  Count
};
static constexpr size_t SENSOR_COUNT = static_cast<size_t>(SensorId::Count);

struct DtcEntry {
  uint16_t code  = 0;
  bool     active= false;
  char     desc[48] = {};
};

struct ScanResult {
  bool         detected       = false;
  BrandId      brand          = BrandId::Unknown;
  ProtocolType protocol       = ProtocolType::Unknown;
  InitMethod   initMethod     = InitMethod::None;
  uint8_t      ecuAddress     = 0;
  uint32_t     baudRate       = 0;
  uint8_t      pidsFound      = 0;
  uint8_t      confidence     = 0;
  char         profileId[24]  = "sensor-only";
  char         brandName[24]  = "UNKNOWN";
  char         protoName[32]  = "---";
  uint32_t     scanDurationMs = 0;
};

struct ReadResult {
  float        raw    = 0.0f;
  float        value  = 0.0f;
  bool         valid  = false;
  SensorStatus status = SensorStatus::Offline;
  SensorSource source = SensorSource::Offline;
};

struct SensorSample {
  float        raw          = 0.0f;
  float        converted    = 0.0f;
  float        filtered     = 0.0f;
  float        avg          = 0.0f;
  float        min          = 0.0f;
  float        max          = 0.0f;
  float        lastGood     = 0.0f;
  uint32_t     lastUpdateMs = 0;
  uint32_t     lastGoodMs   = 0;
  uint32_t     timeoutMs    = 1000;
  uint32_t     errorCount   = 0;
  SensorStatus status       = SensorStatus::Offline;
  SensorSource source       = SensorSource::Offline;
  bool         valid        = false;
  bool         hasValue     = false;
};

struct DashboardSnapshot {
  float        speedKmh         = 0.0f;
  uint16_t     rpm              = 0;
  float        afr              = 14.7f;
  float        engineTempC      = 25.0f;
  float        batteryVolt      = 12.6f;
  float        fuelPercent      = 100.0f;
  float        fuelInstantKmL   = 0.0f;
  float        fuelAverageKmL   = 0.0f;
  float        fuelLPer100Km    = 0.0f;
  float        fuelRangeKm      = 0.0f;
  float        tankCapacityL    = 0.0f;
  float        engineHealth     = 100.0f;
  float        throttlePct      = 0.0f;
  float        mapKpa           = 101.3f;
  float        iatC             = 25.0f;
  float        eotC             = 25.0f;
  float        injectorPulseMs  = 0.0f;
  float        ignitionTimingDeg= 0.0f;
  float        ckpHz            = 0.0f;
  float        leanAngleDeg     = 0.0f;
  float        oilPressureBar   = 0.0f;
  float        knockLevel       = 0.0f;
  float        ambientTempC     = 25.0f;
  float        humidityPct      = 50.0f;
  float        baroKpa          = 101.3f;
  float        gearPosition     = 0.0f;
  bool         fuelPumpOn       = false;
  bool         radiatorFanOn    = false;
  uint16_t     dtcCode          = 0;
  bool         ecuEnabled       = false;
  bool         ecuOnline        = false;
  EcuState     ecuState         = EcuState::Disabled;
  BrandId      brand            = BrandId::Unknown;
  UiMode       mode             = UiMode::Scanning;
  WarningLevel warningLevel     = WarningLevel::None;
  WarningReason warningReason   = WarningReason::None;
  bool         engineRunning    = false;
  bool         fuelConsumptionEstimated = true;
  MotorModel   motorModel       = MotorModel::Unknown;
  char         motorName[32]    = "UNKNOWN";
  uint32_t     timestampMs      = 0;
};

struct ECUFrame {
  uint8_t  header     = 0;
  uint8_t  target     = 0;
  uint8_t  source     = 0;
  uint8_t  length     = 0;
  uint8_t  payload[cfg::MAX_ECU_PAYLOAD] = {};
  uint8_t  payloadLen = 0;
  uint8_t  checksum   = 0;
  bool     valid      = false;
  uint32_t timestampMs= 0;
};

static constexpr const char* SENSOR_NAMES[SENSOR_COUNT] = {
  "SPEED","RPM","AFR","ENGINE TEMP","BATT VOLT","FUEL LEVEL",
  "TPS","MAP","IAT","EOT","INJ PULSE","IGN TIMING",
  "CKP","FUEL PUMP","RAD FAN","GEAR","LEAN ANG",
  "OIL PRES","KNOCK","AMBIENT","HUMIDITY","BARO","DTC"
};
static constexpr const char* SENSOR_UNITS[SENSOR_COUNT] = {
  "km/h","rpm","AFR","C","V","%","%","kPa","C","C",
  "ms","deg","Hz","","","","deg","bar","lvl","C","%","kPa",""
};
static constexpr uint32_t SENSOR_TIMEOUTS[SENSOR_COUNT] = {
  cfg::SPEED_TIMEOUT_US/1000UL, cfg::RPM_TIMEOUT_US/1000UL,
  1200,2000,2000,3000,1000,1000,2000,2000,
  1000,1000,1000,1000,1000,1000,1000,1000,1000,3000,3000,3000,5000
};

static const char* sensorStatusText(SensorStatus s) {
  switch(s){ case SensorStatus::Ok: return "OK"; case SensorStatus::Warning: return "WARN";
    case SensorStatus::Error: return "ERR"; default: return "OFF"; }
}
static uint16_t sensorStatusColor(SensorStatus s) {
  switch(s){ case SensorStatus::Ok: return cfg::COL_GREEN; case SensorStatus::Warning: return cfg::COL_YELLOW;
    case SensorStatus::Error: return cfg::COL_ORANGE; default: return cfg::COL_RED; }
}
static const char* sensorSourceText(SensorSource s) {
  switch(s){ case SensorSource::Hardware: return "HW"; case SensorSource::Ecu: return "ECU";
    case SensorSource::Estimated: return "EST"; case SensorSource::Simulation: return "SIM"; default: return "OFF"; }
}
static const char* warningReasonText(WarningReason w) {
  switch(w){ case WarningReason::EcuTimeout: return "ECU TIMEOUT"; case WarningReason::BatteryLow: return "BATTERY LOW";
    case WarningReason::Overheat: return "OVERHEAT"; case WarningReason::AfrLean: return "AFR LEAN";
    case WarningReason::AfrRich: return "AFR RICH"; case WarningReason::SensorFault: return "SENSOR FAULT"; default: return "SYSTEM OK"; }
}
static uint16_t warningColor(WarningLevel lv, WarningReason r) {
  switch(lv){ case WarningLevel::Info: return cfg::COL_ACCENT;
    case WarningLevel::Warning: return (r==WarningReason::AfrLean||r==WarningReason::AfrRich)?cfg::COL_ORANGE:cfg::COL_YELLOW;
    case WarningLevel::Critical: return cfg::COL_RED; default: return cfg::COL_GREEN; }
}
static const char* ecuStateText(EcuState s) {
  switch(s){ case EcuState::Disabled: return "DISABLED"; case EcuState::Idle: return "IDLE";
    case EcuState::Probing: return "PROBING"; case EcuState::Connected: return "CONNECTED";
    case EcuState::Requesting: return "REQUESTING"; case EcuState::WaitingResponse: return "WAITING";
    case EcuState::Error: return "ERROR"; case EcuState::Reconnecting: return "RECONNECTING"; default: return "UNKNOWN"; }
}
static const char* uiModeText(UiMode m) {
  switch(m){ case UiMode::Scanning: return "SCANNING"; case UiMode::ScanResult: return "SCAN RESULT";
    case UiMode::Dashboard: return "DASHBOARD"; case UiMode::EcuMapping: return "ECU MAPPING";
    case UiMode::Diagnostic: return "DIAGNOSTIC"; case UiMode::SensorMonitor: return "SENSOR MON"; default: return "UNKNOWN"; }
}
static const char* brandText(BrandId b) {
  switch(b){ case BrandId::Honda: return "HONDA"; case BrandId::Yamaha: return "YAMAHA";
    case BrandId::Suzuki: return "SUZUKI"; case BrandId::Kawasaki: return "KAWASAKI";
    case BrandId::Ducati: return "DUCATI"; case BrandId::BMW: return "BMW";
    case BrandId::KTM: return "KTM"; case BrandId::GenericObd2: return "GENERIC OBD2"; default: return "UNKNOWN"; }
}
static const char* scanPhaseText(ScanPhase p) {
  switch(p){
    case ScanPhase::CanBus125k:      return "CAN 125k";
    case ScanPhase::CanBus250k:      return "CAN 250k";
    case ScanPhase::CanBus500k:      return "CAN 500k";
    case ScanPhase::CanBus1M:        return "CAN 1M";
    case ScanPhase::KLineFastHonda:  return "K-LINE FAST -> HONDA (0x6A)";
    case ScanPhase::KLineFastYamaha: return "K-LINE FAST -> YAMAHA (0x85)";
    case ScanPhase::KLineFastKawasaki:return "K-LINE FAST -> KAWASAKI (0x84)";
    case ScanPhase::KLineFastGeneric:return "K-LINE FAST -> GENERIC (0x33)";
    case ScanPhase::KLine5BaudHonda: return "K-LINE 5BAUD -> HONDA (0x6A)";
    case ScanPhase::KLine5BaudYamaha:return "K-LINE 5BAUD -> YAMAHA (0x85)";
    case ScanPhase::KLine5BaudYamahaAlt:return "K-LINE 5BAUD -> YAMAHA ALT (0x17)";
    case ScanPhase::KLine5BaudSuzuki:return "K-LINE 5BAUD -> SUZUKI (0x6B)";
    case ScanPhase::Done:            return "DETECTED";
    case ScanPhase::Failed:          return "NOT FOUND";
    default:                         return "IDLE";
  }
}

static void formatSensorValueSimple(SensorId id, const SensorSample& s, char* out, size_t sz) {
  const char* unit = SENSOR_UNITS[static_cast<size_t>(id)];
  switch(id) {
    case SensorId::FuelPump: case SensorId::RadiatorFan:
      snprintf(out, sz, "%s", s.converted > 0.5f ? "ON" : "OFF"); break;
    case SensorId::DtcCode:
      if (!static_cast<uint16_t>(s.converted)) snprintf(out, sz, "NONE");
      else snprintf(out, sz, "0x%04X", static_cast<unsigned>(static_cast<uint16_t>(s.converted))); break;
    default:
      if (id == SensorId::Rpm) snprintf(out, sz, "%.0f %s", s.filtered, unit);
      else if (id == SensorId::Afr) snprintf(out, sz, "%.2f %s", s.filtered, unit);
      else snprintf(out, sz, "%.1f %s", s.filtered, unit);
      break;
  }
}

struct AfrLookupPoint {
  float voltage;
  float afr;
};

static constexpr AfrLookupPoint NARROWBAND_AFR_TABLE[] = {
  { 0.05f, 20.0f },
  { 0.10f, 18.5f },
  { 0.20f, 16.2f },
  { 0.30f, 15.3f },
  { 0.45f, 14.7f },
  { 0.60f, 14.2f },
  { 0.75f, 13.5f },
  { 0.90f, 12.8f },
  { 1.00f, 12.2f },
};

static float afrFromNarrowbandVoltage(float voltage) {
  constexpr size_t count = sizeof(NARROWBAND_AFR_TABLE) / sizeof(NARROWBAND_AFR_TABLE[0]);
  if (voltage <= NARROWBAND_AFR_TABLE[0].voltage) return NARROWBAND_AFR_TABLE[0].afr;
  for (size_t i = 1; i < count; ++i) {
    const AfrLookupPoint& lo = NARROWBAND_AFR_TABLE[i - 1];
    const AfrLookupPoint& hi = NARROWBAND_AFR_TABLE[i];
    if (voltage <= hi.voltage) {
      const float span = hi.voltage - lo.voltage;
      const float t = span > 0.0f ? (voltage - lo.voltage) / span : 0.0f;
      return lo.afr + (hi.afr - lo.afr) * t;
    }
  }
  return NARROWBAND_AFR_TABLE[count - 1].afr;
}

struct PidSlot {
  uint8_t  service;
  uint8_t  pid;
  uint32_t intervalMs;
};

struct TempLimits {
  float normalMin;
  float normalMax;
  float warningTemp;
  float dangerTemp;
};

struct MotorProfile {
  MotorModel   model;
  BrandId      brandId;
  const char*  id;
  const char*  name;
  ProtocolType protocol;
  ChecksumMode checksumMode;
  InitMethod   initMethod;
  float        tankCapacityL;
  float        wheelCircumferenceM;
  float        rpmPulsePerRev;
  float        speedPulsePerRev;
  uint16_t     fuelAdcEmpty;
  uint16_t     fuelAdcFull;
  float        tpsVoltClosed;
  float        tpsVoltOpen;
  TempLimits   temp;
  uint8_t      ecuAddress;
  uint8_t      testerAddress;
  uint8_t      klineHeader;
  uint32_t     baudRate;
  uint8_t      startCommBytes[4];
  uint8_t      startCommLen;
  uint8_t      ackPrefix[2];
  uint8_t      ackPrefixLen;
  PidSlot      pids[cfg::MAX_PIDS];
  uint8_t      pidCount;
  bool         supportsEcuRpm;
  bool         supportsEcuTemp;
  bool         supportsEcuTps;
  bool         supportsEcuVoltage;
  bool         supportsDtc;
  uint32_t     canSpeed;
  uint32_t     canTxId;
  uint32_t     canRxId;
  bool         isCanBus;
};

static const MotorProfile PROFILE_SENSOR_ONLY = {
  MotorModel::SensorOnly, BrandId::Unknown, "sensor-only", "Manual Sensor Mode",
  ProtocolType::None, ChecksumMode::Sum8, InitMethod::None,
  3.5f, cfg::WHEEL_CIRCUMFERENCE_M, cfg::RPM_PULSES_PER_REV, cfg::SPEED_PULSES_PER_REV,
  3800, 400, 0.50f, 4.50f, { 70.0f, 95.0f, 105.0f, 120.0f },
  0, 0, 0, 0,
  {}, 0, {}, 0, {}, 0,
  false, false, false, false, false,
  0, 0, 0, false
};

static const MotorProfile PROFILE_HONDA_VARIO125_KZR = {
  MotorModel::HondaVario125Kzr, BrandId::Honda, "vario125kzr", "Honda Vario 125 KZR",
  ProtocolType::HondaKLine, ChecksumMode::TwosComplement, InitMethod::FastInit,
  5.5f, 1.720f, 1.0f, 1.0f,
  3600, 550, 0.50f, 4.50f, { 70.0f, 95.0f, 100.0f, 115.0f },
  cfg::ECU_ADDR_HONDA, cfg::TESTER_ADDR, cfg::KLINE_HEADER, cfg::KLINE_BAUD,
  { 0xC1, 0xD1, 0x8F }, 3,
  { 0x83 }, 1,
  {
    { cfg::OBD_MODE_LIVE, cfg::PID_ENGINE_RPM, 80  },
    { cfg::OBD_MODE_LIVE, cfg::PID_SPEED,      100 },
    { cfg::OBD_MODE_LIVE, cfg::PID_COOLANT,    400 },
    { cfg::OBD_MODE_LIVE, cfg::PID_TPS,        150 },
    { cfg::OBD_MODE_LIVE, cfg::PID_O2_1,       250 },
    { cfg::OBD_MODE_LIVE, cfg::PID_BATT,       800 },
    { cfg::OBD_MODE_DTC,  0x00,                5000 },
  },
  7,
  true, true, true, true, true,
  0, 0, 0, false
};

static const MotorProfile PROFILE_GENERIC_KLINE = {
  MotorModel::GenericKLine, BrandId::GenericObd2, "generic-kline", "Generic K-Line OBD2",
  ProtocolType::GenericOBD2KLine, ChecksumMode::Sum8, InitMethod::FastInit,
  4.0f, cfg::WHEEL_CIRCUMFERENCE_M, cfg::RPM_PULSES_PER_REV, cfg::SPEED_PULSES_PER_REV,
  3800, 400, 0.50f, 4.50f, { 70.0f, 95.0f, 105.0f, 120.0f },
  cfg::ECU_ADDR_GENERIC, cfg::TESTER_ADDR, cfg::KLINE_HEADER, cfg::KLINE_BAUD,
  { 0xC1, 0xD1, 0x8F }, 3,
  { 0x83 }, 1,
  {
    { cfg::OBD_MODE_LIVE, cfg::PID_ENGINE_RPM, 100 },
    { cfg::OBD_MODE_LIVE, cfg::PID_SPEED,      100 },
    { cfg::OBD_MODE_LIVE, cfg::PID_TPS,        200 },
    { cfg::OBD_MODE_LIVE, cfg::PID_COOLANT,    500 },
    { cfg::OBD_MODE_LIVE, cfg::PID_BATT,       800 },
    { cfg::OBD_MODE_DTC,  0x00,                5000 },
  },
  6,
  true, true, true, true, true,
  0, 0, 0, false
};

static const MotorProfile PROFILE_YAMAHA = {
  MotorModel::YamahaGeneric, BrandId::Yamaha, "yamaha-kline", "Yamaha K-Line",
  ProtocolType::YamahaKLine, ChecksumMode::Sum8, InitMethod::FiveBaudInit,
  4.2f, cfg::WHEEL_CIRCUMFERENCE_M, cfg::RPM_PULSES_PER_REV, cfg::SPEED_PULSES_PER_REV,
  3800, 400, 0.50f, 4.50f, { 70.0f, 95.0f, 105.0f, 120.0f },
  cfg::ECU_ADDR_YAMAHA, cfg::TESTER_ADDR, 0x80, cfg::KLINE_BAUD,
  { 0x81 }, 1, { 0x83 }, 1,
  {
    { 0x21, 0x01, 100 }, { 0x21, 0x02, 150 }, { 0x21, 0x03, 400 },
    { 0x21, 0x04, 800 }, { 0x21, 0x20, 200 }, { 0x21, 0x21, 500 },
    { 0x21, 0x30, 200 }, { cfg::OBD_MODE_DTC, 0x00, 5000 },
  },
  8,
  true, true, true, true, true,
  0, 0, 0, false
};

static const MotorProfile PROFILE_YAMAHA_ALT = {
  MotorModel::YamahaAlt, BrandId::Yamaha, "yamaha-alt-kline", "Yamaha K-Line ALT",
  ProtocolType::YamahaKLine, ChecksumMode::Sum8, InitMethod::FiveBaudInit,
  4.2f, cfg::WHEEL_CIRCUMFERENCE_M, cfg::RPM_PULSES_PER_REV, cfg::SPEED_PULSES_PER_REV,
  3800, 400, 0.50f, 4.50f, { 70.0f, 95.0f, 105.0f, 120.0f },
  cfg::ECU_ADDR_YAMAHA2, cfg::TESTER_ADDR, 0x80, cfg::KLINE_BAUD,
  { 0x81 }, 1, { 0x83 }, 1,
  {
    { 0x21, 0x01, 100 }, { 0x21, 0x02, 150 }, { 0x21, 0x03, 400 },
    { 0x21, 0x04, 800 }, { 0x21, 0x20, 200 }, { cfg::OBD_MODE_DTC, 0x00, 5000 },
  },
  6,
  true, true, true, true, true,
  0, 0, 0, false
};

static const MotorProfile PROFILE_SUZUKI = {
  MotorModel::SuzukiGeneric, BrandId::Suzuki, "suzuki-kline", "Suzuki K-Line",
  ProtocolType::SuzukiKLine, ChecksumMode::Sum8, InitMethod::FiveBaudInit,
  4.0f, cfg::WHEEL_CIRCUMFERENCE_M, cfg::RPM_PULSES_PER_REV, cfg::SPEED_PULSES_PER_REV,
  3800, 400, 0.50f, 4.50f, { 70.0f, 95.0f, 105.0f, 120.0f },
  cfg::ECU_ADDR_SUZUKI, cfg::TESTER_ADDR, cfg::KLINE_HEADER, cfg::KLINE_BAUD,
  { 0x81 }, 1, { 0x83 }, 1,
  {
    { cfg::OBD_MODE_LIVE, cfg::PID_ENGINE_RPM, 100 },
    { cfg::OBD_MODE_LIVE, cfg::PID_SPEED,      100 },
    { cfg::OBD_MODE_LIVE, cfg::PID_COOLANT,    500 },
    { cfg::OBD_MODE_LIVE, cfg::PID_TPS,        200 },
    { cfg::OBD_MODE_LIVE, cfg::PID_MAP,        200 },
    { cfg::OBD_MODE_LIVE, cfg::PID_IAT,        500 },
    { cfg::OBD_MODE_DTC,  0x00,                5000 },
  },
  7,
  true, true, true, false, true,
  0, 0, 0, false
};

static const MotorProfile PROFILE_KAWASAKI = {
  MotorModel::KawasakiGeneric, BrandId::Kawasaki, "kawasaki-kline", "Kawasaki K-Line",
  ProtocolType::KawasakiKLine, ChecksumMode::Sum8, InitMethod::FastInit,
  4.0f, cfg::WHEEL_CIRCUMFERENCE_M, cfg::RPM_PULSES_PER_REV, cfg::SPEED_PULSES_PER_REV,
  3800, 400, 0.50f, 4.50f, { 70.0f, 95.0f, 105.0f, 120.0f },
  cfg::ECU_ADDR_KAWASAKI, cfg::TESTER_ADDR, cfg::KLINE_HEADER, cfg::KLINE_BAUD,
  { 0xC1, 0xD1, 0x8F }, 3, { 0x83 }, 1,
  {
    { cfg::OBD_MODE_LIVE, cfg::PID_ENGINE_RPM, 100 },
    { cfg::OBD_MODE_LIVE, cfg::PID_SPEED,      100 },
    { cfg::OBD_MODE_LIVE, cfg::PID_COOLANT,    500 },
    { cfg::OBD_MODE_LIVE, cfg::PID_TPS,        200 },
    { cfg::OBD_MODE_LIVE, cfg::PID_IAT,        500 },
    { cfg::OBD_MODE_LIVE, cfg::PID_BATT,       800 },
    { cfg::OBD_MODE_DTC,  0x00,                5000 },
  },
  7,
  true, true, true, true, true,
  0, 0, 0, false
};

static const MotorProfile PROFILE_DUCATI = {
  MotorModel::DucatiCan, BrandId::Ducati, "ducati-can", "Ducati CAN Scaffold",
  ProtocolType::CAN, ChecksumMode::Sum8, InitMethod::None,
  4.0f, cfg::WHEEL_CIRCUMFERENCE_M, 1.0f, 1.0f,
  3800, 400, 0.50f, 4.50f, { 70.0f, 95.0f, 105.0f, 120.0f },
  0, 0, 0, 0, {}, 0, {}, 0, {}, 0,
  false, false, false, false, false,
  1000000UL, 0x143, 0x144, true
};

static const MotorProfile PROFILE_BMW = {
  MotorModel::BmwCan, BrandId::BMW, "bmw-can", "BMW CAN Scaffold",
  ProtocolType::CAN, ChecksumMode::Sum8, InitMethod::None,
  4.0f, cfg::WHEEL_CIRCUMFERENCE_M, 1.0f, 1.0f,
  3800, 400, 0.50f, 4.50f, { 70.0f, 95.0f, 105.0f, 120.0f },
  0, 0, 0, 0, {}, 0, {}, 0, {}, 0,
  false, false, false, false, false,
  500000UL, 0x612, 0x613, true
};

static const MotorProfile PROFILE_KTM = {
  MotorModel::KtmCan, BrandId::KTM, "ktm-can", "KTM CAN Scaffold",
  ProtocolType::CAN, ChecksumMode::Sum8, InitMethod::None,
  4.0f, cfg::WHEEL_CIRCUMFERENCE_M, 1.0f, 1.0f,
  3800, 400, 0.50f, 4.50f, { 70.0f, 95.0f, 105.0f, 120.0f },
  0, 0, 0, 0, {}, 0, {}, 0, {}, 0,
  false, false, false, false, false,
  250000UL, 0x201, 0x211, true
};

struct ScanCandidate {
  ScanPhase          phase;
  const MotorProfile* profile;
};

static const ScanCandidate SCAN_CANDIDATES[] = {
  { ScanPhase::KLineFastHonda,       &PROFILE_HONDA_VARIO125_KZR },
  { ScanPhase::KLineFastKawasaki,    &PROFILE_KAWASAKI           },
  { ScanPhase::KLineFastYamaha,      &PROFILE_YAMAHA             },
  { ScanPhase::KLine5BaudYamaha,     &PROFILE_YAMAHA             },
  { ScanPhase::KLine5BaudYamahaAlt,  &PROFILE_YAMAHA_ALT         },
  { ScanPhase::KLine5BaudSuzuki,     &PROFILE_SUZUKI             },
  { ScanPhase::KLine5BaudHonda,      &PROFILE_HONDA_VARIO125_KZR },
  { ScanPhase::KLineFastGeneric,     &PROFILE_GENERIC_KLINE      },
  { ScanPhase::CanBus250k,           &PROFILE_KTM                },
  { ScanPhase::CanBus500k,           &PROFILE_BMW                },
  { ScanPhase::CanBus1M,             &PROFILE_DUCATI             },
};
static constexpr size_t SCAN_CANDIDATE_COUNT = sizeof(SCAN_CANDIDATES) / sizeof(SCAN_CANDIDATES[0]);

static const MotorProfile* const USER_PROFILES[] = {
  &PROFILE_HONDA_VARIO125_KZR,
  &PROFILE_GENERIC_KLINE,
  &PROFILE_SENSOR_ONLY,
  &PROFILE_YAMAHA,
  &PROFILE_YAMAHA_ALT,
  &PROFILE_SUZUKI,
  &PROFILE_KAWASAKI,
};
static constexpr size_t USER_PROFILE_COUNT = sizeof(USER_PROFILES) / sizeof(USER_PROFILES[0]);

static const MotorProfile* activeProfile = &PROFILE_SENSOR_ONLY;

static const MotorProfile* getDefaultProfile() {
  return &PROFILE_SENSOR_ONLY;
}

static const char* getProfileName(const MotorProfile* profile) {
  return profile ? profile->name : "UNKNOWN";
}

static const MotorProfile* findProfile(const char* id) {
  if (!id || !*id) return nullptr;
  for (size_t i = 0; i < USER_PROFILE_COUNT; ++i) {
    if (strcmp(USER_PROFILES[i]->id, id) == 0) return USER_PROFILES[i];
  }
  return nullptr;
}

static const MotorProfile* profileForBrand(BrandId id) {
  switch(id) {
    case BrandId::Honda:     return &PROFILE_HONDA_VARIO125_KZR;
    case BrandId::Yamaha:    return &PROFILE_YAMAHA;
    case BrandId::Suzuki:    return &PROFILE_SUZUKI;
    case BrandId::Kawasaki:  return &PROFILE_KAWASAKI;
    case BrandId::Ducati:    return &PROFILE_DUCATI;
    case BrandId::BMW:       return &PROFILE_BMW;
    case BrandId::KTM:       return &PROFILE_KTM;
    default:                 return &PROFILE_GENERIC_KLINE;
  }
}

static const MotorProfile* profileForScanResult(const ScanResult& result) {
  for (size_t i = 0; i < SCAN_CANDIDATE_COUNT; ++i) {
    const MotorProfile* profile = SCAN_CANDIDATES[i].profile;
    if (!profile) continue;
    if (strcmp(profile->id, result.profileId) == 0) return profile;
    if (profile->brandId == result.brand &&
        profile->ecuAddress == result.ecuAddress &&
        profile->initMethod == result.initMethod) {
      return profile;
    }
  }
  return profileForBrand(result.brand);
}

static const char* protocolText(ProtocolType protocol) {
  switch (protocol) {
    case ProtocolType::None: return "None";
    case ProtocolType::HondaKLine: return "Honda K-Line";
    case ProtocolType::GenericOBD2KLine: return "Generic OBD2 K-Line";
    case ProtocolType::CAN: return "CAN";
    case ProtocolType::YamahaKLine: return "Yamaha K-Line";
    case ProtocolType::SuzukiKLine: return "Suzuki K-Line";
    case ProtocolType::KawasakiKLine: return "Kawasaki K-Line";
    case ProtocolType::ISO9141_5Baud: return "ISO9141 5-Baud";
    case ProtocolType::KWP2000_FastInit: return "KWP2000 Fast Init";
    default: return "Unknown";
  }
}

static const char* checksumText(ChecksumMode mode) {
  switch (mode) {
    case ChecksumMode::TwosComplement: return "TwosComplement";
    case ChecksumMode::Sum8:
    default: return "Sum8";
  }
}

static const char* initMethodText(InitMethod method) {
  switch (method) {
    case InitMethod::FastInit: return "FastInit";
    case InitMethod::FiveBaudInit: return "FiveBaudInit";
    default: return "None";
  }
}

struct ProfileDetectResult {
  const MotorProfile* profile = nullptr;
  uint8_t confidence = 0;
  bool confident = false;
};

// ============================================================================
// K-line helpers and protocol management
// ============================================================================
class PacketValidator {
public:
  static uint8_t checksum(const uint8_t* data, size_t len, ChecksumMode mode) {
    uint16_t s = 0;
    for (size_t i = 0; i < len; ++i) s += data[i];
    const uint8_t sum8 = static_cast<uint8_t>(s & 0xFF);
    if (mode == ChecksumMode::TwosComplement) {
      return static_cast<uint8_t>((0x100U - sum8) & 0xFFU);
    }
    return sum8;
  }
  static bool isHeaderValid(uint8_t h) {
    return h==0x68 || h==0x80 || h==0xC2 || h==0x02 || h==0x0E || h==0x83;
  }
  static bool validate(const uint8_t* raw, size_t rawLen, ChecksumMode mode) {
    if (!raw || rawLen < 5) return false;
    if (!isHeaderValid(raw[0])) return false;
    uint8_t len = raw[3];
    if (len > cfg::MAX_ECU_PAYLOAD) return false;
    size_t total = 4u + len + 1u;
    if (rawLen < total) return false;
    return checksum(raw, 4+len, mode) == raw[4+len];
  }
};

class KWP2000Handler {
public:
  static size_t buildRequest(uint8_t* out, size_t outSize,
                             uint8_t header, uint8_t target,
                             uint8_t source, uint8_t service, uint8_t pid,
                             ChecksumMode checksumMode) {
    if (!out || outSize < 7) return 0;
    out[0] = header; out[1] = target; out[2] = source;
    out[3] = 0x02;   out[4] = service; out[5] = pid;
    out[6] = PacketValidator::checksum(out, 6, checksumMode);
    return 7;
  }
  static bool parseFrame(const uint8_t* raw, size_t rawLen, ECUFrame& frame,
                         ChecksumMode checksumMode) {
    if (!PacketValidator::validate(raw, rawLen, checksumMode)) return false;
    frame.header     = raw[0]; frame.target = raw[1];
    frame.source     = raw[2]; frame.length = raw[3];
    frame.payloadLen = raw[3]; frame.checksum = raw[4+raw[3]];
    frame.timestampMs= millis(); frame.valid = true;
    for (size_t i = 0; i < raw[3]; ++i) frame.payload[i] = raw[4+i];
    return true;
  }
};

class KLineInitiator {
public:
  static bool fastInit(HardwareSerial& serial,
                       int16_t txPin, int16_t rxPin,
                       const MotorProfile& profile) {
    serial.end();
    pinMode(static_cast<uint8_t>(txPin), OUTPUT);
    digitalWrite(static_cast<uint8_t>(txPin), HIGH);
    delay(30);
    digitalWrite(static_cast<uint8_t>(txPin), LOW);
    delay(cfg::KLINE_FAST_LOW_MS);
    digitalWrite(static_cast<uint8_t>(txPin), HIGH);
    delay(cfg::KLINE_FAST_HIGH_MS);
    serial.begin(profile.baudRate, SERIAL_8N1,
                 static_cast<uint8_t>(rxPin), static_cast<uint8_t>(txPin));
    serial.setTimeout(cfg::KLINE_STARTCOMM_TMO_MS);
    delay(5);
    while (serial.available()) serial.read();
    for (uint8_t i = 0; i < profile.startCommLen; ++i) {
      serial.write(profile.startCommBytes[i]);
      delay(cfg::KLINE_INTER_BYTE_MS);
    }
    return waitForAck(serial, profile, cfg::KLINE_STARTCOMM_TMO_MS);
  }

  static bool fiveBaudInit(HardwareSerial& serial,
                            int16_t txPin, int16_t rxPin,
                            const MotorProfile& profile) {
    serial.end();
    pinMode(static_cast<uint8_t>(txPin), OUTPUT);
    digitalWrite(static_cast<uint8_t>(txPin), HIGH);
    delay(300);
    sendByte5Baud(txPin, profile.ecuAddress);
    delay(cfg::KLINE_W4_MS);
    serial.begin(profile.baudRate, SERIAL_8N1,
                 static_cast<uint8_t>(rxPin), static_cast<uint8_t>(txPin));
    serial.setTimeout(cfg::KLINE_STARTCOMM_TMO_MS);
    delay(5);
    while (serial.available()) serial.read();
    uint8_t sync = 0, key1 = 0, key2 = 0;
    if (!readByteWithin(serial, sync, 1500) || sync != 0x55) {
      if (cfg::DEBUG_MODE) Serial.printf("[KLINE] 5-baud sync fail: 0x%02X\n", sync);
      return false;
    }
    if (!readByteWithin(serial, key1, 300) || !readByteWithin(serial, key2, 300)) {
      if (cfg::DEBUG_MODE) Serial.println("[KLINE] 5-baud key read timeout");
      return false;
    }
    delay(20);
    const uint8_t inverseKey2 = static_cast<uint8_t>(~key2);
    serial.write(inverseKey2);
    delay(25);
    uint8_t finalAck = 0;
    const bool gotFinalAck = readByteWithin(serial, finalAck, 150);
    if (cfg::DEBUG_MODE) {
      Serial.printf("[KLINE] 5-baud sync=0x%02X key1=0x%02X key2=0x%02X sent=0x%02X",
                    sync, key1, key2, inverseKey2);
      if (gotFinalAck) Serial.printf(" ack=0x%02X\n", finalAck);
      else Serial.println(" ack=--");
    }
    for (uint8_t i = 0; i < profile.startCommLen; ++i) {
      serial.write(profile.startCommBytes[i]);
      delay(cfg::KLINE_INTER_BYTE_MS);
    }
    return waitForAck(serial, profile, cfg::KLINE_STARTCOMM_TMO_MS);
  }

private:
  static void sendByte5Baud(int16_t txPin, uint8_t data) {
    uint8_t pin = static_cast<uint8_t>(txPin);
    digitalWrite(pin, LOW);
    delay(cfg::KLINE_5BAUD_BIT_MS);
    for (uint8_t i = 0; i < 8; ++i) {
      digitalWrite(pin, (data >> i) & 0x01 ? HIGH : LOW);
      delay(cfg::KLINE_5BAUD_BIT_MS);
    }
    digitalWrite(pin, HIGH);
    delay(cfg::KLINE_5BAUD_BIT_MS);
  }

  static bool waitForAck(HardwareSerial& serial,
                          const MotorProfile& profile,
                          uint32_t timeoutMs) {
    uint32_t t0 = millis();
    uint8_t  rxBuf[8] = {};
    uint8_t  rxLen = 0;
    while ((millis()-t0) < timeoutMs) {
      if (serial.available()) {
        rxBuf[rxLen++] = static_cast<uint8_t>(serial.read());
        if (profile.ackPrefixLen > 0 && rxLen >= profile.ackPrefixLen) {
          bool match = true;
          for (uint8_t i = 0; i < profile.ackPrefixLen; ++i) {
            if (rxBuf[rxLen-profile.ackPrefixLen+i] != profile.ackPrefix[i]) { match = false; break; }
          }
          if (match) return true;
        }
        if (rxLen >= 8) return false;
      }
      yield();
    }
    return false;
  }

  static bool readByteWithin(HardwareSerial& serial, uint8_t& out, uint32_t timeoutMs) {
    const uint32_t t0 = millis();
    while ((millis() - t0) < timeoutMs) {
      if (serial.available()) {
        out = static_cast<uint8_t>(serial.read());
        return true;
      }
      yield();
    }
    return false;
  }
};

class KLineManager {
public:
  KLineManager()
    : _serial(Serial2), _state(EcuState::Disabled),
      _protocol(ProtocolType::Unknown), _enabled(false), _connected(false),
      _requestPending(false), _rxPin(-1), _txPin(-1),
      _baud(cfg::KLINE_BAUD), _lastRxMs(0), _lastRetryMs(0),
      _lastRequestMs(0), _errorCount(0), _checksumMode(ChecksumMode::Sum8),
      _rxLen(0), _haveFrame(false), _echoLen(0), _echoIdx(0), _echoStartMs(0) {}

  bool begin(int16_t rxPin, int16_t txPin, uint32_t baud) {
    _rxPin = rxPin; _txPin = txPin; _baud = baud;
    if (pinActive(_txPin)) { pinMode(static_cast<uint8_t>(_txPin), OUTPUT); digitalWrite(static_cast<uint8_t>(_txPin), HIGH); }
    if (pinActive(_rxPin) && pinActive(_txPin))
      _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
    _serial.setTimeout(cfg::KLINE_RX_TIMEOUT_MS);
    _state = EcuState::Idle;
    if (cfg::DEBUG_MODE) Serial.printf("[KLINE] UART RX=%d TX=%d BAUD=%lu\n", _rxPin, _txPin, (unsigned long)_baud);
    return true;
  }

  void markConnected(ProtocolType proto) {
    _connected = true; _protocol = proto;
    _state = EcuState::Connected;
    _lastRxMs = millis();
    if (cfg::DEBUG_MODE) Serial.println("[KLINE] Marked connected");
  }

  void enable(bool e) {
    _enabled = e;
    if (!e) { _state = EcuState::Disabled; _connected = false; _requestPending = false; }
    else    { _state = EcuState::Reconnecting; _lastRetryMs = 0; }
  }

  void reinit(int16_t rxPin, int16_t txPin, uint32_t baud) {
    _rxPin = rxPin; _txPin = txPin; _baud = baud;
    _serial.end();
    _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
    _serial.setTimeout(cfg::KLINE_RX_TIMEOUT_MS);
    clearEchoFilter();
  }

  void setProfile(const MotorProfile* profile) {
    if (!profile) profile = getDefaultProfile();
    _checksumMode = profile->checksumMode;
    if (profile->protocol == ProtocolType::None) {
      _protocol = ProtocolType::None;
    } else {
      _protocol = profile->protocol;
    }
  }

  void update() {
    if (!_enabled) return;
    readIncoming();
    const uint32_t now = millis();
    if (_requestPending && (now-_lastRequestMs) > cfg::KLINE_RX_TIMEOUT_MS) {
      _requestPending = false; _state = EcuState::Connected; _errorCount++;
    }
    if (_connected && (now-_lastRxMs) > 3000UL) {
      _connected = false; _state = EcuState::Reconnecting;
      _protocol = ProtocolType::Unknown;
      if (cfg::DEBUG_MODE) Serial.println("[KLINE] ECU timeout");
    }
  }

  bool sendRequest(uint8_t header, uint8_t target, uint8_t tester, uint8_t service, uint8_t pid) {
    if (!_enabled || !_connected || _requestPending) return false;
    uint8_t frame[8] = {};
    size_t len = KWP2000Handler::buildRequest(frame, sizeof(frame), header, target, tester, service, pid, _checksumMode);
    if (!len) return false;
    _serial.write(frame, len);
    armEchoFilter(frame, len);
    _lastRequestMs = millis(); _requestPending = true;
    _state = EcuState::WaitingResponse;
    return true;
  }

  bool pollFrame(ECUFrame& frame) {
    if (!_haveFrame) return false;
    frame = _pendingFrame; _haveFrame = false; return true;
  }

  bool isConnected()      const { return _connected; }
  bool isEnabled()        const { return _enabled; }
  bool waitingResponse()  const { return _requestPending; }
  EcuState state()        const { return _state; }
  uint32_t errorCount()   const { return _errorCount; }
  ProtocolType protocol() const { return _protocol; }
  int16_t rxPin()         const { return _rxPin; }
  int16_t txPin()         const { return _txPin; }
  uint32_t baud()         const { return _baud; }
  HardwareSerial& serial()      { return _serial; }

private:
  HardwareSerial& _serial;
  EcuState    _state;
  ProtocolType _protocol;
  bool _enabled, _connected, _requestPending;
  int16_t _rxPin, _txPin;
  uint32_t _baud;
  uint32_t _lastRxMs, _lastRetryMs, _lastRequestMs;
  uint32_t _errorCount;
  ChecksumMode _checksumMode;
  uint8_t  _rxBuffer[cfg::MAX_ECU_FRAME];
  size_t   _rxLen;
  ECUFrame _pendingFrame;
  bool     _haveFrame;
  uint8_t  _echoBuf[16];
  size_t   _echoLen;
  size_t   _echoIdx;
  uint32_t _echoStartMs;

  void readIncoming() {
    while (_serial.available()) {
      uint8_t b = static_cast<uint8_t>(_serial.read());
      if (filterEchoByte(b, millis())) continue;
      if (_rxLen < sizeof(_rxBuffer)) _rxBuffer[_rxLen++] = b;
      else { memmove(_rxBuffer, _rxBuffer+1, sizeof(_rxBuffer)-1); _rxBuffer[sizeof(_rxBuffer)-1]=b; _errorCount++; }
    }
    while (_rxLen >= 5) {
      if (!PacketValidator::isHeaderValid(_rxBuffer[0])) { shiftBuf(1); continue; }
      size_t flen = 4u + _rxBuffer[3] + 1u;
      if (flen > sizeof(_rxBuffer)) { shiftBuf(1); _errorCount++; continue; }
      if (_rxLen < flen) return;
      ECUFrame f;
      if (KWP2000Handler::parseFrame(_rxBuffer, flen, f, _checksumMode)) {
        _pendingFrame = f; _haveFrame = true;
        _requestPending = false; _lastRxMs = millis();
        if (cfg::DEBUG_MODE) Serial.printf("[KLINE] RX len=%u pay=%u\n", (unsigned)flen, (unsigned)f.payloadLen);
      } else { _errorCount++; shiftBuf(1); continue; }
      if (_rxLen > flen) { memmove(_rxBuffer, _rxBuffer+flen, _rxLen-flen); _rxLen -= flen; }
      else _rxLen = 0;
    }
  }

  void shiftBuf(size_t n) {
    if (n >= _rxLen) { _rxLen = 0; return; }
    memmove(_rxBuffer, _rxBuffer+n, _rxLen-n); _rxLen -= n;
  }
  void armEchoFilter(const uint8_t* data, size_t len) {
    _echoLen = (len < sizeof(_echoBuf)) ? len : sizeof(_echoBuf);
    for (size_t i = 0; i < _echoLen; ++i) _echoBuf[i] = data[i];
    _echoIdx = 0;
    _echoStartMs = millis();
  }
  void clearEchoFilter() {
    _echoLen = 0;
    _echoIdx = 0;
    _echoStartMs = 0;
  }
  bool filterEchoByte(uint8_t b, uint32_t now) {
    if (_echoIdx >= _echoLen || _echoLen == 0) return false;
    if ((now - _echoStartMs) > cfg::KLINE_ECHO_WINDOW_MS) {
      clearEchoFilter();
      return false;
    }
    if (b == _echoBuf[_echoIdx]) {
      _echoIdx++;
      if (_echoIdx >= _echoLen) clearEchoFilter();
      return true;
    }
    clearEchoFilter();
    return false;
  }
  static inline bool pinActive(int16_t p) { return p >= 0; }
};

class SensorHub;

class ECUDataParser {
public:
  static void parse(const ECUFrame& frame, SensorHub& hub, const MotorProfile& profile);
};

class ProtocolScanner {
public:
  void begin(KLineManager& kline, const MotorProfile* preferredProfile = nullptr) {
    _kline         = &kline;
    _candidateIdx  = 0;
    _phase         = ScanPhase::Idle;
    _phaseStart    = 0;
    _scanStart     = millis();
    _done          = false;
    _preferredOnly = preferredProfile != nullptr;
    if (_preferredOnly) {
      _preferredCandidate.phase = scanPhaseForProfile(*preferredProfile);
      _preferredCandidate.profile = preferredProfile;
    }
    _currentDesc[0]= '\0';
    _statusMsg[0]  = '\0';
    if (cfg::DEBUG_MODE) {
      Serial.println("[SCAN] Starting universal ECU scan...");
      Serial.printf("[SCAN] %u candidates\n", (unsigned)candidateCount());
    }
  }

  bool update() {
    if (_done) return true;
    if (!_kline) { _done = true; return true; }
    const uint32_t now = millis();
    if ((now - _scanStart) > cfg::SCAN_TOTAL_TMO_MS) { finishFailed(); return true; }
    if (_candidateIdx >= candidateCount()) { finishFailed(); return true; }
    const ScanCandidate& cand = candidateAt(_candidateIdx);
    if (!cand.profile) { finishFailed(); return true; }
    if (cand.profile->isCanBus || cand.phase == ScanPhase::CanBus125k ||
        cand.phase == ScanPhase::CanBus250k || cand.phase == ScanPhase::CanBus500k ||
        cand.phase == ScanPhase::CanBus1M) {
      if (cfg::DEBUG_MODE)
        Serial.printf("[SCAN] Skip CAN %s (Fase 3 HW required)\n", scanPhaseText(cand.phase));
      snprintf(_statusMsg, sizeof(_statusMsg), "%s  SKIP (no HW)", scanPhaseText(cand.phase));
      _candidateIdx++;
      return false;
    }
    if (_phaseStart == 0) {
      _phase      = cand.phase;
      _phaseStart = now;
      snprintf(_currentDesc, sizeof(_currentDesc), "%s", scanPhaseText(cand.phase));
      snprintf(_statusMsg, sizeof(_statusMsg), "%s  TRYING...", _currentDesc);
      if (cfg::DEBUG_MODE)
        Serial.printf("[SCAN] Trying %s ...\n", _currentDesc);
      bool ok = false;
      const MotorProfile& prof = *cand.profile;
      _kline->setProfile(&prof);
      if (prof.initMethod == InitMethod::FastInit) {
        ok = KLineInitiator::fastInit(_kline->serial(),
                                      _kline->txPin(), _kline->rxPin(), prof);
      } else if (prof.initMethod == InitMethod::FiveBaudInit) {
        ok = KLineInitiator::fiveBaudInit(_kline->serial(),
                                          _kline->txPin(), _kline->rxPin(), prof);
      } else {
        ok = false;
      }
      if (ok) {
        _kline->reinit(_kline->rxPin(), _kline->txPin(), prof.baudRate);
        ProtocolType proto = prof.protocol;
        _kline->markConnected(proto);
        finishSuccess(cand, proto);
        return true;
      } else {
        _kline->reinit(_kline->rxPin(), _kline->txPin(), cfg::KLINE_BAUD);
        snprintf(_statusMsg, sizeof(_statusMsg), "%s  NO RESPONSE", _currentDesc);
        if (cfg::DEBUG_MODE)
          Serial.printf("[SCAN] %s - no response\n", _currentDesc);
        _candidateIdx++;
        _phaseStart = 0;
      }
    }
    return false;
  }

  bool          isDone()   const { return _done; }
  ScanResult    result()   const { return _result; }
  ScanPhase     phase()    const { return _phase; }
  const char*   statusMsg()const { return _statusMsg; }
  const char*   currentDesc() const { return _currentDesc; }
  uint8_t       candidateIdx() const { return _candidateIdx; }
  uint8_t       candidateCount() const { return _preferredOnly ? 1 : static_cast<uint8_t>(SCAN_CANDIDATE_COUNT); }

private:
  KLineManager* _kline         = nullptr;
  uint8_t       _candidateIdx  = 0;
  ScanPhase     _phase         = ScanPhase::Idle;
  uint32_t      _phaseStart    = 0;
  uint32_t      _scanStart     = 0;
  bool          _done          = false;
  bool          _preferredOnly = false;
  ScanCandidate _preferredCandidate = { ScanPhase::Idle, nullptr };
  ScanResult    _result        = {};
  char          _currentDesc[48] = {};
  char          _statusMsg[64]   = {};

  const ScanCandidate& candidateAt(uint8_t idx) const {
    return _preferredOnly ? _preferredCandidate : SCAN_CANDIDATES[idx];
  }

  static ScanPhase scanPhaseForProfile(const MotorProfile& profile) {
    if (profile.protocol == ProtocolType::CAN) return ScanPhase::CanBus500k;
    if (profile.initMethod == InitMethod::FiveBaudInit) {
      if (profile.model == MotorModel::YamahaAlt) return ScanPhase::KLine5BaudYamahaAlt;
      if (profile.brandId == BrandId::Yamaha) return ScanPhase::KLine5BaudYamaha;
      if (profile.brandId == BrandId::Suzuki) return ScanPhase::KLine5BaudSuzuki;
      return ScanPhase::KLine5BaudHonda;
    }
    if (profile.brandId == BrandId::Kawasaki) return ScanPhase::KLineFastKawasaki;
    if (profile.brandId == BrandId::Yamaha) return ScanPhase::KLineFastYamaha;
    if (profile.protocol == ProtocolType::GenericOBD2KLine) return ScanPhase::KLineFastGeneric;
    return ScanPhase::KLineFastHonda;
  }

  static uint8_t confidenceForProfile(const MotorProfile& profile) {
    uint8_t score = 30; // K-Line init berhasil
    if (profile.ackPrefixLen > 0) score += 30; // response pattern sesuai kandidat
    if (profile.pidCount > 0) score += 10;     // ada PID dasar yang dikenal
    if (profile.checksumMode == ChecksumMode::Sum8 ||
        profile.checksumMode == ChecksumMode::TwosComplement) score += 10;
    if (profile.supportsEcuRpm || profile.supportsEcuTemp || profile.supportsEcuTps) score += 10;
    return score > 100 ? 100 : score;
  }

  void finishSuccess(const ScanCandidate& cand, ProtocolType proto) {
    _done = true;
    _phase = ScanPhase::Done;
    _result.detected       = true;
    _result.brand          = cand.profile->brandId;
    _result.protocol       = proto;
    _result.initMethod     = cand.profile->initMethod;
    _result.ecuAddress     = cand.profile->ecuAddress;
    _result.baudRate       = cand.profile->baudRate;
    _result.pidsFound      = cand.profile->pidCount;
    _result.confidence     = confidenceForProfile(*cand.profile);
    _result.scanDurationMs = millis() - _scanStart;
    strncpy(_result.profileId, cand.profile->id, sizeof(_result.profileId)-1);
    strncpy(_result.brandName, cand.profile->name, sizeof(_result.brandName)-1);
    strncpy(_result.protoName, protocolText(proto), sizeof(_result.protoName)-1);
    snprintf(_statusMsg, sizeof(_statusMsg), "DETECTED: %s %s", _result.brandName, _result.protoName);
    if (cfg::DEBUG_MODE)
      Serial.printf("[SCAN] OK %s via %s addr=0x%02X confidence=%u in %lums\n",
                    _result.brandName, _result.protoName,
                    _result.ecuAddress, _result.confidence,
                    (unsigned long)_result.scanDurationMs);
  }

  void finishFailed() {
    _done = true;
    _phase = ScanPhase::Failed;
    _result.detected = false;
    _result.scanDurationMs = millis() - _scanStart;
    snprintf(_statusMsg, sizeof(_statusMsg), "ECU NOT FOUND - check connector & ignition");
    if (cfg::DEBUG_MODE) Serial.println("[SCAN] FAIL No ECU detected");
  }
};

template <typename T, size_t N>
class HistoryWindow {
public:
  HistoryWindow() { reset(); }
  void reset() { _count=0; _head=0; for(size_t i=0;i<N;i++) _v[i]=T{}; }
  void push(T val) { _v[_head]=val; _head=(_head+1)%N; if(_count<N)_count++; }
  size_t count() const { return _count; }
  T at(size_t i) const {
    if(!_count||i>=_count) return T{};
    size_t s=(_head+N-_count)%N;
    return _v[(s+i)%N];
  }
  T latest() const { return _count?at(_count-1):T{}; }
  T mean() const {
    if(!_count) return T{};
    T s=T{};
    for(size_t i=0;i<_count;i++) s+=at(i);
    return s/static_cast<T>(_count);
  }
  float stddev() const {
    if(_count<2) return 0.f;
    float m=static_cast<float>(mean()), acc=0.f;
    for(size_t i=0;i<_count;i++){float d=static_cast<float>(at(i))-m; acc+=d*d;}
    return sqrtf(acc/static_cast<float>(_count));
  }
private:
  T _v[N]; size_t _count, _head;
};

class AnalogSampler {
public:
  static uint16_t averageRaw(int16_t pin, uint8_t n=6) {
    if(pin<0) return 0;
    uint32_t s=0;
    for(uint8_t i=0;i<n;i++) s+=analogRead(static_cast<uint8_t>(pin));
    return static_cast<uint16_t>(s/n);
  }
  static float toVolt(uint16_t raw) {
    return (raw/cfg::ADC_MAX_RAW)*cfg::ADC_REF_VOLT;
  }
};

class SensorHub {
public:
  SensorHub()
    : _profile(getDefaultProfile()), _simMode(cfg::SIMULATION_MODE), _ecuEnabled(false), _ecuOnline(false),
      _lastFastMs(0), _lastSlowMs(0), _lastSimMs(0),
      _lastDistMs(0), _lastRpmMs(0), _lastSpdMs(0),
      _prevRpm(0), _prevSpd(0), _distM(0.f), _simPh(0.f), _fuelAvgKmL(0.f) {
    for(size_t i=0;i<SENSOR_COUNT;i++) _s[i].timeoutMs=SENSOR_TIMEOUTS[i];
  }

  bool begin() {
    analogReadResolution(12);
    auto setAtt=[](int16_t p){ if(p>=0) analogSetPinAttenuation(static_cast<uint8_t>(p),ADC_11db); };
    setAtt(cfg::PIN_AFR_ADC); setAtt(cfg::PIN_TEMP_ADC);
    setAtt(cfg::PIN_BATT_ADC); setAtt(cfg::PIN_FUEL_ADC);
    setAtt(cfg::PIN_TPS_ADC); setAtt(cfg::PIN_MAP_ADC);
    // GPIO34/GPIO35 ESP32 tidak punya internal pull-up. RPM/speed wajib pakai
    // pull-up eksternal 10k ke 3.3V dan signal conditioner/opto, bukan pulser/coil langsung.
    if(cfg::PIN_RPM>=0){ pinMode(cfg::PIN_RPM,INPUT); attachInterrupt(digitalPinToInterrupt(cfg::PIN_RPM),isrRpm,FALLING); }
    if(cfg::PIN_SPEED>=0){ pinMode(cfg::PIN_SPEED,INPUT); attachInterrupt(digitalPinToInterrupt(cfg::PIN_SPEED),isrSpd,FALLING); }
    if(cfg::DEBUG_MODE) Serial.println("[SENSOR] Init OK");
    return true;
  }

  void setProfile(const MotorProfile* profile) {
    _profile = profile ? profile : getDefaultProfile();
    _fuelAvgKmL = 0.0f;
  }
  void setSimMode(bool v)   { _simMode=v; }
  void setEcuEnabled(bool v){ _ecuEnabled=v; }
  void setEcuOnline(bool v) { _ecuOnline=v; }
  bool ecuOnline()  const   { return _ecuOnline; }
  bool ecuEnabled() const   { return _ecuEnabled; }
  const DashboardSnapshot& snapshot() const { return _snap; }
  const SensorSample& sample(SensorId id) const { return _s[idx(id)]; }
  size_t sensorCount() const { return SENSOR_COUNT; }
  const char* sensorName(SensorId id) const { return SENSOR_NAMES[idx(id)]; }

  void inject(SensorId id, float v, bool valid=true, SensorSource src=SensorSource::Ecu) {
    auto& f=_ext[idx(id)]; f.value=v; f.valid=valid; f.lastMs=millis(); f.src=src;
  }
  void injectRpm(float v,bool ok=true)       { inject(SensorId::Rpm,v,ok); }
  void injectSpeed(float v,bool ok=true)     { inject(SensorId::Speed,v,ok); }
  void injectAfr(float v,bool ok=true)       { inject(SensorId::Afr,v,ok); }
  void injectEngineTemp(float v,bool ok=true){ inject(SensorId::EngineTemp,v,ok); }
  void injectBattery(float v,bool ok=true)   { inject(SensorId::Battery,v,ok); }
  void injectFuel(float v,bool ok=true)      { inject(SensorId::FuelLevel,v,ok); }
  void injectTps(float v,bool ok=true)       { inject(SensorId::Tps,v,ok); }
  void injectMap(float v,bool ok=true)       { inject(SensorId::Map,v,ok); }
  void injectIat(float v,bool ok=true)       { inject(SensorId::Iat,v,ok); }
  void injectEot(float v,bool ok=true)       { inject(SensorId::Eot,v,ok); }
  void injectDtc(uint16_t c,bool ok=true)    { inject(SensorId::DtcCode,static_cast<float>(c),ok); }

  void update() {
    const uint32_t now=millis();
    if(_simMode){ updateSim(now); buildSnap(now); return; }
    if((now-_lastFastMs)>=cfg::SENSOR_FAST_MS){ _lastFastMs=now; updateFast(now); }
    if((now-_lastSlowMs)>=cfg::SENSOR_SLOW_MS){ _lastSlowMs=now; updateSlow(now); }
    updateTimeouts(now); updateDist(now); buildSnap(now);
  }

private:
  struct ExtFeed { float value=0; bool valid=false; uint32_t lastMs=0; SensorSource src=SensorSource::Ecu; };

  static volatile uint32_t s_rpmCnt, s_rpmEdge, s_spdCnt, s_spdEdge;
  const MotorProfile* _profile;
  bool _simMode,_ecuEnabled,_ecuOnline;
  uint32_t _lastFastMs,_lastSlowMs,_lastSimMs,_lastDistMs,_lastRpmMs,_lastSpdMs;
  uint32_t _prevRpm,_prevSpd;
  float _distM,_simPh,_fuelAvgKmL;
  SensorSample _s[SENSOR_COUNT];
  HistoryWindow<float,8> _hist[SENSOR_COUNT];
  ExtFeed _ext[SENSOR_COUNT];
  DashboardSnapshot _snap;

  static void IRAM_ATTR isrRpm(){ uint32_t t=micros(); if(t-s_rpmEdge<cfg::RPM_MIN_EDGE_US)return; s_rpmEdge=t; s_rpmCnt++; }
  static void IRAM_ATTR isrSpd(){ uint32_t t=micros(); if(t-s_spdEdge<cfg::SPEED_MIN_EDGE_US)return; s_spdEdge=t; s_spdCnt++; }
  static size_t idx(SensorId id){ return static_cast<size_t>(id); }
  static inline float clamp(float v,float lo,float hi){ return v<lo?lo:(v>hi?hi:v); }
  static inline float mapf(float x,float a,float b,float c,float d){ return fabsf(b-a)<1e-4f?c:c+(x-a)/(b-a)*(d-c); }
  static inline bool finite_(float v){ return !isnan(v)&&!isinf(v); }
  static inline float lpf(float prev,float in,float a){ return prev+(in-prev)*clamp(a,0.f,1.f); }

  ReadResult readSpd(uint32_t now) {
    ReadResult r; r.source=SensorSource::Hardware;
    noInterrupts(); uint32_t cnt=s_spdCnt,edge=s_spdEdge; interrupts();
    if(!_lastSpdMs){ _lastSpdMs=now; r.status=SensorStatus::Offline; return r; }
    uint32_t dt=now-_lastSpdMs; _lastSpdMs=now;
    if((micros()-edge)>cfg::SPEED_TIMEOUT_US){ r.status=SensorStatus::Offline; r.value=_s[idx(SensorId::Speed)].lastGood; return r; }
    noInterrupts(); uint32_t d=cnt-_prevSpd; _prevSpd=cnt; interrupts();
    const float pulses = (_profile && _profile->speedPulsePerRev > 0.01f) ? _profile->speedPulsePerRev : cfg::SPEED_PULSES_PER_REV;
    const float wheel = (_profile && _profile->wheelCircumferenceM > 0.1f) ? _profile->wheelCircumferenceM : cfg::WHEEL_CIRCUMFERENCE_M;
    float km=(dt>0)?((d/pulses*wheel)/(dt/1000.f)*3.6f*cfg::SPEED_CAL_FACTOR):0.f;
    r.raw=d; r.value=clamp(km,0.f,220.f); r.valid=true; r.status=SensorStatus::Ok; return r;
  }

  ReadResult readRpm(uint32_t now) {
    ReadResult r; r.source=SensorSource::Hardware;
    noInterrupts(); uint32_t cnt=s_rpmCnt,edge=s_rpmEdge; interrupts();
    if(!_lastRpmMs){ _lastRpmMs=now; _prevRpm=cnt; r.status=SensorStatus::Offline; return r; }
    uint32_t dt=now-_lastRpmMs; _lastRpmMs=now;
    if((micros()-edge)>cfg::RPM_TIMEOUT_US){ r.status=SensorStatus::Offline; r.value=_s[idx(SensorId::Rpm)].lastGood; return r; }
    noInterrupts(); uint32_t d=cnt-_prevRpm; _prevRpm=cnt; interrupts();
    const float pulses = (_profile && _profile->rpmPulsePerRev > 0.01f) ? _profile->rpmPulsePerRev : cfg::RPM_PULSES_PER_REV;
    float rpm=(dt>0)?((d/pulses)*(60000.f/dt)*cfg::RPM_CAL_FACTOR):0.f;
    r.raw=d; r.value=clamp(rpm,0.f,14000.f); r.valid=true; r.status=SensorStatus::Ok; return r;
  }

  ReadResult readAfr() {
    ReadResult r; r.source=SensorSource::Hardware;
    if(cfg::PIN_AFR_ADC<0) return r;
    uint16_t raw=AnalogSampler::averageRaw(cfg::PIN_AFR_ADC);
    float sv=AnalogSampler::toVolt(raw)*cfg::AFR_DIVIDER_RATIO;
    r.raw=sv;
    if(sv<0.0f||sv>1.20f){ r.status=SensorStatus::Error; r.value=_s[idx(SensorId::Afr)].lastGood; return r; }
    float afr=afrFromNarrowbandVoltage(sv);
    r.value=clamp(afr,8.f,22.f); r.valid=true;
    r.status=(sv<cfg::AFR_SENSOR_V_MIN||sv>cfg::AFR_SENSOR_V_MAX)?SensorStatus::Warning:SensorStatus::Ok;
    return r;
  }

  ReadResult readTemp() {
    ReadResult r; r.source=SensorSource::Hardware;
    if(cfg::PIN_TEMP_ADC<0) return r;
    uint16_t raw=AnalogSampler::averageRaw(cfg::PIN_TEMP_ADC);
    float v=AnalogSampler::toVolt(raw); r.raw=v;
    if(v<0.02f||v>cfg::ADC_REF_VOLT-0.02f){ r.status=SensorStatus::Error; r.value=_s[idx(SensorId::EngineTemp)].lastGood; return r; }
    float R=cfg::TEMP_NTC_FIXED_OHM*(v/(cfg::ADC_REF_VOLT-v));
    float iT=(1.f/298.15f)+(1.f/cfg::TEMP_NTC_BETA)*logf(R/cfg::TEMP_NTC_R25_OHM);
    float tc=clamp(1.f/iT-273.15f+cfg::TEMP_OFFSET_C,-20.f,160.f);
    const float warn = _profile ? _profile->temp.warningTemp : 105.0f;
    const float danger = _profile ? _profile->temp.dangerTemp : 120.0f;
    r.value=tc; r.valid=true; r.status=tc>danger?SensorStatus::Error:(tc>warn?SensorStatus::Warning:SensorStatus::Ok); return r;
  }

  ReadResult readBatt() {
    ReadResult r; r.source=SensorSource::Hardware;
    if(cfg::PIN_BATT_ADC<0) return r;
    uint16_t raw=AnalogSampler::averageRaw(cfg::PIN_BATT_ADC,8);
    float v=AnalogSampler::toVolt(raw)*cfg::BATT_DIVIDER_RATIO;
    r.raw=raw;
    if(v<6.f||v>16.5f){ r.status=SensorStatus::Error; r.value=_s[idx(SensorId::Battery)].lastGood; return r; }
    r.value=v; r.valid=true; r.status=v<11.4f?SensorStatus::Warning:SensorStatus::Ok; return r;
  }

  ReadResult readFuel() {
    ReadResult r; r.source=SensorSource::Hardware;
    if(cfg::PIN_FUEL_ADC<0) return r;
    uint16_t raw=AnalogSampler::averageRaw(cfg::PIN_FUEL_ADC,8);
    const float empty = _profile ? static_cast<float>(_profile->fuelAdcEmpty) : 3800.0f;
    const float full = _profile ? static_cast<float>(_profile->fuelAdcFull) : 400.0f;
    float pct=clamp(mapf(raw,empty,full,0.f,100.f),0.f,100.f);
    r.raw=raw; r.value=pct; r.valid=(raw>0&&raw<4095);
    r.status=r.valid?(pct<10.f?SensorStatus::Warning:SensorStatus::Ok):SensorStatus::Error; return r;
  }

  ReadResult readTps() {
    ReadResult r; r.source=SensorSource::Hardware;
    if(cfg::PIN_TPS_ADC<0) return r;
    uint16_t raw=AnalogSampler::averageRaw(cfg::PIN_TPS_ADC);
    float v=AnalogSampler::toVolt(raw);
    const float closed = _profile ? _profile->tpsVoltClosed : 0.50f;
    const float open = _profile ? _profile->tpsVoltOpen : 4.50f;
    float pct=clamp(mapf(v,closed,open,0.f,100.f),0.f,100.f);
    r.raw=v; r.value=pct; r.valid=(raw>0&&raw<4095);
    r.status=r.valid?SensorStatus::Ok:SensorStatus::Error; return r;
  }

  ReadResult readMap() {
    ReadResult r; r.source=SensorSource::Hardware;
    if(cfg::PIN_MAP_ADC<0) return r;
    uint16_t raw=AnalogSampler::averageRaw(cfg::PIN_MAP_ADC);
    float kpa=clamp(mapf(raw,120.f,3900.f,20.f,140.f),0.f,300.f);
    r.raw=raw; r.value=kpa; r.valid=(raw>0&&raw<4095);
    r.status=r.valid?SensorStatus::Ok:SensorStatus::Error; return r;
  }

  ReadResult readExt(SensorId id) {
    ReadResult r; const auto& f=_ext[idx(id)]; r.source=f.src;
    if(f.valid&&(millis()-f.lastMs)<=cfg::ECU_EXT_TIMEOUT_MS){
      r.raw=f.value; r.value=f.value; r.valid=true; r.status=SensorStatus::Ok; r.source=f.src; return r;
    }
    r.raw=_s[idx(id)].lastGood; r.value=r.raw; r.valid=false;
    r.status=SensorStatus::Offline; r.source=SensorSource::Offline; return r;
  }

  ReadResult estimateInjPulse(uint32_t now){
    ReadResult r=readExt(SensorId::InjectorPulse); if(r.valid) return r;
    float rpm=_s[idx(SensorId::Rpm)].filtered, tps=_s[idx(SensorId::Tps)].filtered,
          map=_s[idx(SensorId::Map)].filtered, tc=_s[idx(SensorId::EngineTemp)].filtered;
    if(rpm<100.f){ r.status=SensorStatus::Offline; return r; }
    float ld=0.55f*(tps/100.f)+0.45f*clamp(map/101.3f,0.f,1.6f);
    float w=(tc<60.f)?1.12f:1.f;
    float p=clamp((1.f+ld*5.5f+(rpm/10000.f)*1.8f)*w,0.8f,16.f);
    r.raw=p; r.value=p; r.valid=true; r.status=SensorStatus::Warning; r.source=SensorSource::Estimated;
    (void)now; return r;
  }

  void updateFast(uint32_t now){
    writeSample(SensorId::Speed,readSpd(now),now);
    writeSample(SensorId::Rpm,readRpm(now),now);
    writeSample(SensorId::Afr,readAfr(),now);
    writeSample(SensorId::Tps,readTps(),now);
    writeSample(SensorId::Map,readMap(),now);
    writeSample(SensorId::InjectorPulse,estimateInjPulse(now),now);
    writeSample(SensorId::Ckp,readExt(SensorId::Ckp),now);
  }

  void updateSlow(uint32_t now){
    writeSample(SensorId::EngineTemp,readTemp(),now);
    writeSample(SensorId::Battery,readBatt(),now);
    writeSample(SensorId::FuelLevel,readFuel(),now);
    for(SensorId id:{SensorId::Iat,SensorId::Eot,SensorId::FuelPump,SensorId::RadiatorFan,
                     SensorId::GearPosition,SensorId::LeanAngle,SensorId::OilPressure,
                     SensorId::Knock,SensorId::AmbientTemp,SensorId::Humidity,
                     SensorId::BarometricPressure,SensorId::DtcCode,SensorId::IgnitionTiming})
      writeSample(id,readExt(id),now);
  }

  void updateSim(uint32_t now){
    _simPh+=0.015f;
    auto w=[&](SensorId id,float v,SensorStatus st=SensorStatus::Ok){
      writeSample(id,{v,v,true,st,SensorSource::Simulation},now); };
    w(SensorId::Speed,   58.f+22.f*sinf(_simPh*0.8f));
    w(SensorId::Rpm,   3200.f+1800.f*sinf(_simPh*1.7f));
    w(SensorId::Afr,    14.6f+0.9f*sinf(_simPh*2.3f));
    w(SensorId::EngineTemp, 82.f+11.f*sinf(_simPh*0.45f));
    w(SensorId::Battery,13.9f+0.3f*sinf(_simPh*0.2f));
    w(SensorId::FuelLevel,clamp(62.f-fmodf(_simPh*0.04f,6.f),0.f,100.f));
    w(SensorId::Tps,18.f+25.f*(0.5f+0.5f*sinf(_simPh*1.1f)));
    w(SensorId::Map,66.f+24.f*sinf(_simPh*0.9f));
    w(SensorId::Iat,31.f+2.f*sinf(_simPh*0.25f));
    w(SensorId::Eot,86.f+8.f*sinf(_simPh*0.35f));
    (void)now;
  }

  void writeSample(SensorId id, const ReadResult& rr, uint32_t now){
    size_t i=idx(id); SensorSample& s=_s[i];
    s.raw=rr.raw; s.lastUpdateMs=now;
    if(rr.valid&&finite_(rr.value)){
      s.valid=true; s.source=rr.source; s.status=rr.status; s.converted=rr.value;
      s.filtered=s.hasValue?lpf(s.filtered,rr.value,0.22f):rr.value;
      s.hasValue=true;
      _hist[i].push(s.filtered); s.avg=_hist[i].mean();
      if(s.lastGoodMs==0){s.min=s.max=s.filtered;}
      else{if(s.filtered<s.min)s.min=s.filtered; if(s.filtered>s.max)s.max=s.filtered;}
      s.lastGood=s.filtered; s.lastGoodMs=now;
    } else {
      s.valid=false; s.source=rr.source; s.errorCount++;
      s.converted=s.lastGood; s.filtered=s.lastGood; s.avg=_hist[i].mean();
      s.status=(rr.status==SensorStatus::Error)?SensorStatus::Error:
               (s.lastGoodMs==0||(now-s.lastGoodMs)>s.timeoutMs)?SensorStatus::Offline:SensorStatus::Warning;
    }
  }

  void updateTimeouts(uint32_t now){
    for(size_t i=0;i<SENSOR_COUNT;i++){
      auto& s=_s[i];
      if(s.lastGoodMs&&(now-s.lastGoodMs)>s.timeoutMs){s.status=SensorStatus::Offline;s.valid=false;}
    }
  }

  void updateDist(uint32_t now){
    if(!_lastDistMs){_lastDistMs=now;return;}
    float dt=(now-_lastDistMs)/3600000.f; _lastDistMs=now;
    _distM+=_s[idx(SensorId::Speed)].filtered*dt*1000.f;
  }

  void buildSnap(uint32_t now){
    auto g=[&](SensorId id){return _s[idx(id)].filtered;};
    const MotorProfile* profile = _profile ? _profile : getDefaultProfile();
    _snap.timestampMs=now;
    _snap.speedKmh    =g(SensorId::Speed);
    _snap.rpm         =static_cast<uint16_t>(roundf(g(SensorId::Rpm)));
    _snap.afr         =g(SensorId::Afr);
    _snap.engineTempC =g(SensorId::EngineTemp);
    _snap.batteryVolt =g(SensorId::Battery);
    _snap.fuelPercent =g(SensorId::FuelLevel);
    _snap.throttlePct =g(SensorId::Tps);
    _snap.mapKpa      =g(SensorId::Map);
    _snap.iatC        =g(SensorId::Iat);
    _snap.eotC        =g(SensorId::Eot);
    _snap.injectorPulseMs   =g(SensorId::InjectorPulse);
    _snap.ignitionTimingDeg =g(SensorId::IgnitionTiming);
    _snap.ckpHz       =g(SensorId::Ckp);
    _snap.fuelPumpOn  =g(SensorId::FuelPump)>0.5f;
    _snap.radiatorFanOn=g(SensorId::RadiatorFan)>0.5f;
    _snap.dtcCode     =static_cast<uint16_t>(roundf(g(SensorId::DtcCode)));
    _snap.ecuEnabled  =_ecuEnabled; _snap.ecuOnline=_ecuOnline;
    _snap.engineRunning=_snap.rpm>300U;
    _snap.tankCapacityL = profile->tankCapacityL;
    _snap.motorModel = profile->model;
    strncpy(_snap.motorName, profile->name, sizeof(_snap.motorName)-1);
    _snap.motorName[sizeof(_snap.motorName)-1] = '\0';
    updateFuelAndHealth(profile);
    _snap.brand = profile->brandId;
    _snap.mode = _ecuEnabled ? UiMode::Dashboard : UiMode::SensorMonitor;
  }

  void updateFuelAndHealth(const MotorProfile* profile) {
    const float rpm = static_cast<float>(_snap.rpm);
    const float pulseMs = _snap.injectorPulseMs;
    const bool pulseFromEcu = _s[idx(SensorId::InjectorPulse)].source == SensorSource::Ecu;
    _snap.fuelConsumptionEstimated = !pulseFromEcu;
    if (_snap.engineRunning && rpm > 300.0f && pulseMs > 0.05f) {
      const float duty = clamp((pulseMs * rpm) / 120000.0f, 0.0f, 0.95f);
      const float fuelLph = (cfg::INJECTOR_FLOW_CC_MIN * duty) * 60.0f / 1000.0f;
      if (fuelLph > 0.001f && _snap.speedKmh > 1.0f) {
        _snap.fuelInstantKmL = clamp(_snap.speedKmh / fuelLph, 0.0f, 99.9f);
        _fuelAvgKmL = (_fuelAvgKmL <= 0.01f) ? _snap.fuelInstantKmL : lpf(_fuelAvgKmL, _snap.fuelInstantKmL, 0.03f);
      }
      _snap.fuelLPer100Km = (_snap.fuelInstantKmL > 0.01f) ? (100.0f / _snap.fuelInstantKmL) : 0.0f;
    }
    _snap.fuelAverageKmL = _fuelAvgKmL;
    const float usableFuelL = profile->tankCapacityL * clamp(_snap.fuelPercent, 0.0f, 100.0f) / 100.0f;
    const float kmL = (_fuelAvgKmL > 0.01f) ? _fuelAvgKmL : _snap.fuelInstantKmL;
    _snap.fuelRangeKm = usableFuelL * kmL;

    const float tempScore = (_snap.engineTempC <= profile->temp.warningTemp) ? 100.0f :
      (_snap.engineTempC >= profile->temp.dangerTemp ? 0.0f :
       mapf(_snap.engineTempC, profile->temp.warningTemp, profile->temp.dangerTemp, 100.0f, 0.0f));
    const float afrScore = (_snap.afr >= 13.0f && _snap.afr <= 15.5f) ? 100.0f :
      clamp(100.0f - fabsf(_snap.afr - 14.7f) * 25.0f, 0.0f, 100.0f);
    const float voltScore = (_snap.batteryVolt >= 12.0f && _snap.batteryVolt <= 15.0f) ? 100.0f :
      clamp(100.0f - fabsf(_snap.batteryVolt - 13.6f) * 35.0f, 0.0f, 100.0f);
    const float rpmScore = (_snap.rpm < 11500U) ? 100.0f : 35.0f;
    const float fuelScore = _snap.fuelPercent > 10.0f ? 100.0f : clamp(_snap.fuelPercent * 10.0f, 0.0f, 100.0f);
    _snap.engineHealth = clamp(tempScore * cfg::HEALTH_W_TEMP +
                               afrScore * cfg::HEALTH_W_AFR +
                               voltScore * cfg::HEALTH_W_VOLT +
                               rpmScore * cfg::HEALTH_W_RPM +
                               fuelScore * cfg::HEALTH_W_FUEL, 0.0f, 100.0f);
    if (_snap.engineTempC >= profile->temp.dangerTemp) {
      _snap.warningLevel = WarningLevel::Critical;
      _snap.warningReason = WarningReason::Overheat;
    } else if (_snap.engineTempC >= profile->temp.warningTemp) {
      _snap.warningLevel = WarningLevel::Warning;
      _snap.warningReason = WarningReason::Overheat;
    } else if (_snap.batteryVolt > 0.1f && _snap.batteryVolt < 11.4f) {
      _snap.warningLevel = WarningLevel::Warning;
      _snap.warningReason = WarningReason::BatteryLow;
    } else if (_snap.afr > 16.0f) {
      _snap.warningLevel = WarningLevel::Warning;
      _snap.warningReason = WarningReason::AfrLean;
    } else if (_snap.afr < 12.5f) {
      _snap.warningLevel = WarningLevel::Warning;
      _snap.warningReason = WarningReason::AfrRich;
    } else {
      _snap.warningLevel = WarningLevel::None;
      _snap.warningReason = WarningReason::None;
    }
  }
};

volatile uint32_t SensorHub::s_rpmCnt=0, SensorHub::s_rpmEdge=0;
volatile uint32_t SensorHub::s_spdCnt=0, SensorHub::s_spdEdge=0;

class ECURequestManager {
public:
  void begin(const MotorProfile* profile, bool enabled) {
    _profile = profile; _enabled = enabled; _cursor = 0; _lastPollMs = 0;
    resetPollTimers();
  }
  void setProfile(const MotorProfile* profile) { _profile = profile; _cursor = 0; resetPollTimers(); }

  void update(KLineManager& kline, SensorHub& hub) {
    if (!_enabled || !_profile || !kline.isConnected()) return;
    ECUFrame frame;
    while (kline.pollFrame(frame)) ECUDataParser::parse(frame, hub, *_profile);
    if (kline.waitingResponse()) return;
    const uint32_t now = millis();
    if ((now - _lastPollMs) < cfg::KLINE_POLL_MS) return;
    _lastPollMs = now;
    if (_profile->pidCount == 0) return;
    for (uint8_t i = 0; i < _profile->pidCount; ++i) {
      uint8_t idx = (_cursor + i) % _profile->pidCount;
      const PidSlot& slot = _profile->pids[idx];
      if ((now - _slotLastMs[idx]) >= slot.intervalMs) {
        _slotLastMs[idx] = now;
        kline.sendRequest(_profile->klineHeader, _profile->ecuAddress,
                          _profile->testerAddress, slot.service, slot.pid);
        _cursor = (idx + 1) % _profile->pidCount;
        return;
      }
    }
  }
  void setEnabled(bool e) { _enabled = e; }

private:
  const MotorProfile* _profile   = nullptr;
  bool     _enabled              = false;
  uint8_t  _cursor               = 0;
  uint32_t _lastPollMs           = 0;
  uint32_t _slotLastMs[cfg::MAX_PIDS] = {};

  void resetPollTimers() {
    for (size_t i = 0; i < cfg::MAX_PIDS; ++i) _slotLastMs[i] = 0;
  }
};

void ECUDataParser::parse(const ECUFrame& frame, SensorHub& hub, const MotorProfile& profile) {
  if (!frame.valid || frame.payloadLen < 2) return;
  const uint8_t svc = frame.payload[0];
  const uint8_t pid = frame.payload[1];
  const BrandId brand = profile.brandId;
  if (brand == BrandId::Honda || brand == BrandId::Kawasaki || brand == BrandId::GenericObd2) {
    if (svc < 0x40) return;
    switch (pid) {
      case cfg::PID_ENGINE_RPM:
        if (frame.payloadLen >= 4)
          hub.injectRpm(((uint16_t(frame.payload[2])<<8)|frame.payload[3])/4.0f);
        break;
      case cfg::PID_SPEED:
        if (frame.payloadLen >= 3) hub.injectSpeed(frame.payload[2]);
        break;
      case cfg::PID_COOLANT:
        if (frame.payloadLen >= 3) hub.injectEngineTemp(frame.payload[2]-40.0f);
        break;
      case cfg::PID_TPS:
        if (frame.payloadLen >= 3) hub.injectTps(frame.payload[2]*100.0f/255.0f);
        break;
      case cfg::PID_O2_1:
        if (frame.payloadLen >= 3) {
          float v = frame.payload[2]*0.005f;
          hub.injectAfr(afrFromNarrowbandVoltage(v));
        }
        break;
      case cfg::PID_MAP:
        if (frame.payloadLen >= 3) hub.injectMap(frame.payload[2]);
        break;
      case cfg::PID_IAT:
        if (frame.payloadLen >= 3) hub.injectIat(frame.payload[2]-40.0f);
        break;
      case cfg::PID_BATT:
        if (frame.payloadLen >= 3) hub.injectBattery(frame.payload[2]*0.1f);
        break;
      case 0xD1:
        if (frame.payloadLen >= 3)
          hub.inject(SensorId::InjectorPulse, frame.payload[2]*0.1f, true, SensorSource::Ecu);
        break;
      case 0xD2:
        if (frame.payloadLen >= 3)
          hub.inject(SensorId::IgnitionTiming, frame.payload[2]-64.0f, true, SensorSource::Ecu);
        break;
      default: break;
    }
    return;
  }
  if (brand == BrandId::Yamaha) {
    switch (pid) {
      case 0x01: hub.injectRpm(((uint16_t(frame.payload[2])<<8)|frame.payload[3])/4.0f); break;
      case 0x02: if(frame.payloadLen>=3) hub.injectTps(frame.payload[2]*100.0f/255.0f); break;
      case 0x03: if(frame.payloadLen>=3) hub.injectEngineTemp(frame.payload[2]-40.0f); break;
      case 0x04: if(frame.payloadLen>=3) hub.injectBattery(frame.payload[2]*0.1f); break;
      case 0x20: if(frame.payloadLen>=3) { float v=frame.payload[2]*0.005f; hub.injectAfr(afrFromNarrowbandVoltage(v)); } break;
      case 0x21: if(frame.payloadLen>=3) hub.injectIat(frame.payload[2]-40.0f); break;
      case 0x30: if(frame.payloadLen>=3) hub.inject(SensorId::InjectorPulse,frame.payload[2]*0.1f,true,SensorSource::Ecu); break;
      default: break;
    }
    return;
  }
  if (brand == BrandId::Suzuki) {
    if (svc < 0x40) return;
    switch (pid) {
      case cfg::PID_ENGINE_RPM: if(frame.payloadLen>=4) hub.injectRpm(((uint16_t(frame.payload[2])<<8)|frame.payload[3])/4.f); break;
      case cfg::PID_SPEED:      if(frame.payloadLen>=3) hub.injectSpeed(frame.payload[2]); break;
      case cfg::PID_COOLANT:    if(frame.payloadLen>=3) hub.injectEngineTemp(frame.payload[2]-40.f); break;
      case cfg::PID_TPS:        if(frame.payloadLen>=3) hub.injectTps(frame.payload[2]*100.f/255.f); break;
      case cfg::PID_MAP:        if(frame.payloadLen>=3) hub.injectMap(frame.payload[2]); break;
      case cfg::PID_IAT:        if(frame.payloadLen>=3) hub.injectIat(frame.payload[2]-40.f); break;
      default: break;
    }
    return;
  }
}

// ============================================================================
// Main sketch
// ============================================================================
KLineManager klineManager;
ProtocolScanner scanner;
SensorHub sensorHub;
ECURequestManager requestManager;
Preferences profilePrefs;

bool scanStarted = false;
bool scanProcessed = false;
bool pendingRescan = false;
uint32_t pendingRescanAtMs = 0;
const MotorProfile* requestedScanProfile = nullptr;
uint32_t lastPrintMs = 0;
volatile bool scanButtonPressed = false;
char serialLine[96] = {};
uint8_t serialLineLen = 0;

static inline bool pinActive(int16_t p) { return p >= 0; }

void IRAM_ATTR handleScanButton() {
  scanButtonPressed = true;
}

void printActiveProfile() {
  const MotorProfile* p = activeProfile ? activeProfile : getDefaultProfile();
  Serial.println("----- ACTIVE MOTOR PROFILE -----");
  Serial.printf("ID: %s\n", p->id);
  Serial.printf("Name: %s\n", p->name);
  Serial.printf("Protocol: %s | Checksum: %s | Init: %s\n",
                protocolText(p->protocol), checksumText(p->checksumMode), initMethodText(p->initMethod));
  Serial.printf("Tank: %.1f L | Wheel: %.3f m | RPM pulse/rev: %.2f | Speed pulse/rev: %.2f\n",
                p->tankCapacityL, p->wheelCircumferenceM, p->rpmPulsePerRev, p->speedPulsePerRev);
  Serial.printf("Fuel ADC empty/full: %u/%u | TPS closed/open: %.2f/%.2f V\n",
                p->fuelAdcEmpty, p->fuelAdcFull, p->tpsVoltClosed, p->tpsVoltOpen);
  Serial.printf("Temp normal %.0f-%.0f C | warning %.0f C | danger %.0f C\n",
                p->temp.normalMin, p->temp.normalMax, p->temp.warningTemp, p->temp.dangerTemp);
  Serial.printf("K-Line addr ECU=0x%02X tester=0x%02X header=0x%02X baud=%lu PID=%u\n",
                p->ecuAddress, p->testerAddress, p->klineHeader, (unsigned long)p->baudRate, p->pidCount);
  Serial.printf("ECU sensors: rpm=%s temp=%s tps=%s volt=%s dtc=%s\n",
                p->supportsEcuRpm?"yes":"no", p->supportsEcuTemp?"yes":"no",
                p->supportsEcuTps?"yes":"no", p->supportsEcuVoltage?"yes":"no",
                p->supportsDtc?"yes":"no");
  Serial.println("--------------------------------");
}

void printProfileList() {
  Serial.println("----- MOTOR PROFILES -----");
  for (size_t i = 0; i < USER_PROFILE_COUNT; ++i) {
    const MotorProfile* p = USER_PROFILES[i];
    Serial.printf("%s  -  %s  [%s]\n", p->id, p->name, protocolText(p->protocol));
  }
  Serial.println("--------------------------");
}

void applyMotorProfile(const MotorProfile* profile, const char* reason = "apply") {
  if (!profile) profile = getDefaultProfile();
  activeProfile = profile;
  sensorHub.setProfile(profile);
  klineManager.setProfile(profile);
  requestManager.begin(profile, false);
  sensorHub.setEcuEnabled(profile->protocol != ProtocolType::None);
  sensorHub.setEcuOnline(false);
  if (profile->protocol == ProtocolType::None) {
    klineManager.enable(false);
  }
  Serial.printf("[PROFILE] %s: %s (%s)\n", reason, profile->name, profile->id);
}

ProfileDetectResult detectMotorProfile(const ScanResult& result) {
  ProfileDetectResult out;
  if (!result.detected) {
    out.profile = getDefaultProfile();
    out.confidence = 0;
    out.confident = false;
    return out;
  }
  out.profile = profileForScanResult(result);
  out.confidence = result.confidence;
  out.confident = out.profile && out.confidence >= 70;
  if (!out.confident) {
    out.profile = result.detected ? &PROFILE_GENERIC_KLINE : getDefaultProfile();
  }
  return out;
}

ProfileDetectResult detectMotorProfile() {
  return detectMotorProfile(scanner.result());
}

void saveActiveProfile() {
  const MotorProfile* p = activeProfile ? activeProfile : getDefaultProfile();
  profilePrefs.putString("profile", p->id);
  Serial.printf("[PROFILE] Saved manual profile: %s\n", p->id);
}

void clearSavedProfile() {
  profilePrefs.remove("profile");
  Serial.println("[PROFILE] Saved manual profile cleared");
}

const MotorProfile* loadSavedProfile() {
  char savedId[32] = {};
  size_t len = profilePrefs.getString("profile", savedId, sizeof(savedId));
  if (len == 0) return nullptr;
  const MotorProfile* p = findProfile(savedId);
  if (!p) {
    Serial.printf("[PROFILE] Saved profile '%s' not found, clearing\n", savedId);
    clearSavedProfile();
    return nullptr;
  }
  return p;
}

void printScanResult(const ScanResult& result) {
  Serial.println("----- ECU SCAN RESULT -----");
  if (!result.detected) {
    Serial.println("ECU NOT DETECTED.");
    return;
  }
  Serial.printf("Brand: %s\n", result.brandName);
  Serial.printf("Profile ID: %s\n", result.profileId);
  Serial.printf("Protocol: %s\n", result.protoName);
  Serial.printf("Address: 0x%02X\n", result.ecuAddress);
  Serial.printf("Baud: %lu\n", (unsigned long)result.baudRate);
  Serial.printf("PID candidates: %u\n", result.pidsFound);
  Serial.printf("Confidence: %u\n", result.confidence);
  Serial.printf("Scan time: %lums\n", (unsigned long)result.scanDurationMs);
  Serial.println("---------------------------");
}

void printSnapshot(const DashboardSnapshot& snap) {
  Serial.println("----- ECU / SENSOR SNAPSHOT -----");
  Serial.printf("Time: %lums | ECU: %s | Online: %s\n", (unsigned long)snap.timestampMs,
                snap.ecuEnabled?"ENABLED":"DISABLED",
                snap.ecuOnline?"YES":"NO");
  Serial.printf("Profile: %s | Brand: %s | RPM: %u | Speed: %.1f km/h\n",
                snap.motorName, brandText(snap.brand), snap.rpm, snap.speedKmh);
  Serial.printf("AFR: %.2f | Temp: %.1f C | Batt: %.2f V\n", snap.afr, snap.engineTempC, snap.batteryVolt);
  Serial.printf("TPS: %.1f %% | MAP: %.1f kPa | IAT: %.1f C\n", snap.throttlePct, snap.mapKpa, snap.iatC);
  Serial.printf("Fuel: %.1f %% / %.1f L | Range: %.1f km | Km/L: %.1f %s | Health: %.0f %% | DTC: 0x%04X\n",
                snap.fuelPercent, snap.tankCapacityL, snap.fuelRangeKm,
                snap.fuelAverageKmL > 0.01f ? snap.fuelAverageKmL : snap.fuelInstantKmL,
                snap.fuelConsumptionEstimated ? "(est)" : "(ecu)", snap.engineHealth, snap.dtcCode);
  Serial.println("-------------------------------");
}

void triggerRescan(const MotorProfile* preferredProfile = nullptr) {
  requestedScanProfile = preferredProfile;
  pendingRescan = true;
  pendingRescanAtMs = millis() + cfg::RESCAN_REINIT_DELAY_MS;
  scanStarted = false;
  scanProcessed = false;
  requestManager.setEnabled(false);
  sensorHub.setEcuEnabled(false);
  sensorHub.setEcuOnline(false);
  klineManager.enable(false);
  Serial.printf("[SYS] ECU rescan scheduled (%s)\n",
                preferredProfile ? preferredProfile->id : "auto");
}

void servicePendingRescan() {
  if (!pendingRescan || static_cast<int32_t>(millis() - pendingRescanAtMs) < 0) return;
  pendingRescan = false;
  const MotorProfile* profile = requestedScanProfile ? requestedScanProfile : activeProfile;
  if (!profile) profile = getDefaultProfile();
  const uint32_t baud = profile->baudRate ? profile->baudRate : cfg::KLINE_BAUD;
  klineManager.setProfile(profile);
  klineManager.reinit(cfg::PIN_KLINE_RX, cfg::PIN_KLINE_TX, baud);
  klineManager.enable(true);
  scanner.begin(klineManager, requestedScanProfile);
  scanStarted = true;
  scanProcessed = false;
  Serial.printf("[SYS] ECU rescan started (%s)\n",
                requestedScanProfile ? requestedScanProfile->id : "auto");
}

void autoReconnect() {
  static uint32_t lostSinceMs = 0;
  if (!activeProfile || activeProfile->protocol == ProtocolType::None) {
    lostSinceMs = 0;
    return;
  }
  const ScanResult result = scanner.result();
  if (!scanProcessed || !result.detected) {
    lostSinceMs = 0;
    return;
  }
  if (klineManager.isConnected()) {
    lostSinceMs = 0;
    return;
  }
  const uint32_t now = millis();
  if (lostSinceMs == 0) lostSinceMs = now;
  if ((now - lostSinceMs) >= cfg::ECU_RECONNECT_DELAY_MS) {
    lostSinceMs = 0;
    Serial.println("[SYS] ECU offline, restarting scan");
    triggerRescan(activeProfile && activeProfile->protocol != ProtocolType::None ? activeProfile : nullptr);
  }
}

void processProfileCommand(char* line) {
  if (strcmp(line, "profile list") == 0) {
    printProfileList();
    return;
  }
  if (strcmp(line, "profile show") == 0) {
    printActiveProfile();
    return;
  }
  if (strcmp(line, "profile auto") == 0) {
    clearSavedProfile();
    applyMotorProfile(getDefaultProfile(), "auto mode");
    triggerRescan(nullptr);
    return;
  }
  if (strncmp(line, "profile set ", 12) == 0) {
    const char* id = line + 12;
    const MotorProfile* p = findProfile(id);
    if (!p) {
      Serial.printf("[PROFILE] Unknown profile '%s'\n", id);
      printProfileList();
      return;
    }
    applyMotorProfile(p, "manual override");
    if (p->protocol == ProtocolType::None) {
      scanStarted = false;
      scanProcessed = true;
      pendingRescan = false;
    } else {
      triggerRescan(p);
    }
    return;
  }
  if (strcmp(line, "profile save") == 0) {
    saveActiveProfile();
    return;
  }
  if (strcmp(line, "profile clear") == 0) {
    clearSavedProfile();
    return;
  }
  Serial.println("[PROFILE] Commands: profile list | profile show | profile auto | profile set <id> | profile save | profile clear");
}

void serviceSerialCommands() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      serialLine[serialLineLen] = '\0';
      if (serialLineLen > 0) {
        if (strncmp(serialLine, "profile", 7) == 0) processProfileCommand(serialLine);
        else Serial.println("[SERIAL] Unknown command");
      }
      serialLineLen = 0;
      return;
    }
    if (serialLineLen < sizeof(serialLine) - 1) {
      serialLine[serialLineLen++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Universal ECU scanner starting...");
  profilePrefs.begin("ecu-dash", false);
  if (!sensorHub.begin()) Serial.println("[ERROR] SensorHub init failed");
  if (pinActive(cfg::PIN_SCAN_BUTTON)) {
    pinMode(static_cast<uint8_t>(cfg::PIN_SCAN_BUTTON), INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(cfg::PIN_SCAN_BUTTON), handleScanButton, FALLING);
  }
  klineManager.begin(cfg::PIN_KLINE_RX, cfg::PIN_KLINE_TX, cfg::KLINE_BAUD);
  const MotorProfile* saved = loadSavedProfile();
  if (saved) {
    applyMotorProfile(saved, "boot saved profile");
    if (saved->protocol == ProtocolType::None) {
      scanProcessed = true;
    } else {
      triggerRescan(saved);
    }
  } else {
    applyMotorProfile(getDefaultProfile(), "boot default");
    triggerRescan(nullptr);
  }
  printActiveProfile();
}

void loop() {
  serviceSerialCommands();

  if (scanButtonPressed) {
    scanButtonPressed = false;
    triggerRescan();
  }

  servicePendingRescan();
  klineManager.update();
  bool scanDone = scanStarted ? scanner.isDone() : scanProcessed;
  if (scanStarted && !scanDone) scanDone = scanner.update();

  if (scanDone && !scanProcessed) {
    ScanResult result = scanner.result();
    printScanResult(result);
    const MotorProfile* finalProfile = requestedScanProfile;
    if (!finalProfile) {
      ProfileDetectResult detected = detectMotorProfile(result);
      finalProfile = detected.profile;
      if (detected.confident) {
        Serial.printf("[PROFILE] Auto detect confident: %s (%u)\n",
                      getProfileName(finalProfile), detected.confidence);
      } else {
        Serial.printf("[PROFILE] Auto detect not confident (%u), fallback: %s\n",
                      detected.confidence, getProfileName(finalProfile));
      }
    }
    applyMotorProfile(finalProfile, requestedScanProfile ? "manual scan result" : "auto detect");
    if (result.detected && finalProfile && finalProfile->protocol != ProtocolType::None) {
      requestManager.begin(finalProfile, true);
      sensorHub.setEcuEnabled(true);
      sensorHub.setEcuOnline(klineManager.isConnected());
    } else {
      requestManager.setEnabled(false);
      sensorHub.setEcuEnabled(false);
      sensorHub.setEcuOnline(false);
    }
    scanProcessed = true;
    requestedScanProfile = nullptr;
  }

  if (scanProcessed && activeProfile && activeProfile->protocol != ProtocolType::None && klineManager.isConnected()) {
    requestManager.update(klineManager, sensorHub);
  }

  sensorHub.setEcuOnline(klineManager.isConnected());
  sensorHub.update();
  autoReconnect();

  const uint32_t now = millis();
  if ((now - lastPrintMs) >= 1000) {
    lastPrintMs = now;
    printSnapshot(sensorHub.snapshot());
  }
}
