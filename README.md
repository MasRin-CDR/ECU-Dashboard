# Motorcycle ECU Dashboard ESP32

Production-oriented ESP32 / ESP32-S3 dashboard project for 5 inch TFT displays, real-time sensor acquisition, and future-ready K-Line / KWP2000 ECU integration.

## What this repo contains

This workspace is a small project family, not just a single sketch. The newest and most production-focused base is:

- [ProductionDashboardV2/ProductionDashboardV2.ino](ProductionDashboardV2/ProductionDashboardV2.ino)

Other variants are kept as reference and migration paths:

- [ProductionDashboard/ProductionDashboard.ino](ProductionDashboard/ProductionDashboard.ino)
- [ECU_Dashboard_ESP32S3/ECU_Dashboard/ECU_Dashboard.ino](ECU_Dashboard_ESP32S3/ECU_Dashboard/ECU_Dashboard.ino)
- [MotorcycleECUDashboard/src/main.cpp](MotorcycleECUDashboard/src/main.cpp)
- [vario_dashboard.ino](vario_dashboard.ino)

If you want one recommended starting point, use `ProductionDashboardV2`.

## Core goals

- Real-time dashboard rendering
- Non-blocking sensor acquisition
- Interrupt-driven RPM and speed capture
- Smooth TFT_eSPI UI with partial redraw
- Sensor filtering and fail-safe fallback
- ECU communication scaffold for K-Line / KWP2000
- Easy migration to LVGL or a multi-file architecture later

## Main features

- Speed
- RPM
- AFR
- Engine temperature
- Battery voltage
- Fuel level
- Fuel consumption estimate
- Engine health score
- Sensor status and warning panels
- Mini graphs for live telemetry
- ECU communication status
- Four UI modes:
  - Dashboard
  - ECU Mapping
  - Diagnostic
  - Sensor Monitor

## Repository layout

```text
E:\MONITOR ECU
|-- ProductionDashboardV2
|   `-- ProductionDashboardV2.ino   <- recommended base
|-- ProductionDashboard
|   `-- ProductionDashboard.ino
|-- ECU_Dashboard_ESP32S3
|   `-- ECU_Dashboard
|       |-- ECU_Dashboard.ino
|       |-- KLineProtocol.*
|       |-- ECUManager.*
|       |-- Analyzers.*
|       `-- DashboardUI.*
|-- MotorcycleECUDashboard
|   `-- src
|       `-- main.cpp
|-- vario_dashboard.ino
|-- config.h
|-- sensors.h
|-- kline.h
|-- display.h
`-- README.md
```

## Recommended hardware

- ESP32 or ESP32-S3
- TFT 5 inch 800x480 panel
- TFT_eSPI library
- K-Line transceiver such as MC33290 or L9637 for ECU interface
- Proper automotive power regulation
- Optocoupler isolation for RPM and injector inputs
- Voltage divider and protection network for battery sensing
- TVS diode and proper grounding for vehicle noise immunity

## Default pin map used in the production base

| Signal | GPIO | Notes |
|---|---:|---|
| RPM pickup | 34 | Input only, use isolation and filtering |
| Wheel speed | 35 | Input only, use pull-up and debounce |
| AFR ADC | 36 | ADC1 preferred |
| Engine temp ADC | 39 | ADC1 preferred |
| Battery ADC | 32 | Divider required |
| Fuel level ADC | 33 | Analog float sensor |
| TPS ADC | 25 | ADC2, avoid if Wi-Fi telemetry becomes active |
| MAP ADC | 26 | ADC2, avoid if Wi-Fi telemetry becomes active |
| K-Line RX | 16 | UART to transceiver |
| K-Line TX | 17 | UART to transceiver |
| Mode button | 27 | Physical mode switch |

## Wiring notes

### RPM and injector signals

- Use optocoupler isolation.
- Add RC filtering where needed.
- Never connect noisy 12V vehicle pulses directly to ESP32 GPIO.

### Battery voltage

- Always use a resistor divider.
- Add reverse protection and transient suppression.
- Keep the ADC input inside 0 to 3.3V.

### K-Line / KWP2000

- Do not connect K-Line directly to ESP32 GPIO.
- Use a proper transceiver such as MC33290 or L9637.
- Add protection for DLC wiring and vehicle transients.

### TFT

- Configure the TFT_eSPI driver in its `User_Setup.h`.
- Keep TFT pin assignments aligned with the actual panel and board.
- If PSRAM is available, it can help future UI expansion.

## Build options

### Arduino IDE

1. Open `ProductionDashboardV2/ProductionDashboardV2.ino`
2. Install `TFT_eSPI`
3. Configure `TFT_eSPI` `User_Setup.h`
4. Select ESP32 or ESP32-S3 board
5. Upload

### PlatformIO

Use the PlatformIO-oriented variants in the repo if you prefer a managed build flow. The modular `ECU_Dashboard_ESP32S3` project already contains a `platformio.ini` and separated source files.

## UI modes

### Dashboard mode

Large speed display, RPM bar, AFR, temperature, battery, fuel, engine health, warning panel, and mini graphs.

### ECU mapping mode

Table-style map view with RPM axes, TPS axes, active-cell highlight, and live ECU values.

### Diagnostic mode

Sensor-by-sensor status view with age, timeout, source, and ECU communication status.

### Sensor monitor mode

Raw ADC and raw counter monitoring for tuning, calibration, and noise analysis.

## K-Line architecture

The production base includes a safe scaffold for future ECU reader development:

- `KLineManager`
- `KWP2000Handler`
- `ECUDataParser`
- `ECURequestManager`
- `PacketValidator`

Design goals:

- UART transport ready
- Packet buffer management
- Checksum validation placeholder
- Timeout and retry handling
- Safe parsing boundaries
- No proprietary ECU reverse engineering baked into the core

## Filtering and stability

The dashboard is designed to survive bad or missing sensor data:

- Moving average filtering
- Low-pass filtering
- ADC smoothing
- Anti-jitter thresholds
- Timeout detection
- Last-known-good fallback
- Error and offline state handling

## Fuel and engine analysis

The engine health and fuel metrics are estimate-based until full ECU integration is available.

- Fuel consumption is calculated from speed, injector pulse estimate, and flow assumptions
- Engine health is weighted from temperature, AFR, voltage, RPM stability, and fuel economy
- Values are clamped and filtered to avoid UI spikes

## Security and safety notes

- Do not hardcode Wi-Fi credentials
- Keep secure config placeholders in code
- Sanitize ECU packet parsing
- Limit buffer sizes
- Keep all update loops watchdog-friendly
- Use protected power and signal conditioning in the vehicle harness

## Calibration checklist

1. Calibrate RPM pulse count per revolution
2. Calibrate wheel circumference for speed
3. Calibrate AFR voltage range against your wideband controller
4. Calibrate battery divider ratio with a multimeter
5. Calibrate fuel sender empty/full points
6. Tune temperature curve if you use an NTC sensor

## Suggested next upgrades

- CAN bus support
- BLE telemetry
- Wi-Fi telemetry
- SD card logging
- Data logger export
- GPS speed source
- LVGL UI migration
- ECU-specific PID decoding

## Development roadmap

1. Validate the current single-file production base on real hardware
2. Lock the TFT_eSPI setup for your exact 5 inch panel
3. Finalize pin mapping and analog calibration
4. Add ECU-specific parsing once the K-Line behavior is confirmed
5. Split into multi-file modules if the project becomes a long-term product

## Notes for contributors

- Keep the code non-blocking
- Prefer `millis()` over `delay()`
- Keep ISR work tiny
- Preserve fail-safe behavior
- Avoid rewriting the sensor layer unless the hardware contract changes

## License

Released under the MIT License. See [LICENSE](LICENSE) for the full text.
