# Motorcycle ECU Dashboard

## Overview
The Motorcycle ECU Dashboard is a modern, real-time dashboard application designed for motorcycles. It utilizes the ESP32-S3 microcontroller to communicate with the motorcycle's ECU via K-Line protocols (ISO9141, KWP2000) and displays critical engine data on a 5-inch TFT touchscreen.

## Features
- Real-time display of:
  - RPM (Revolutions Per Minute)
  - Vehicle Speed
  - Battery Voltage
  - Air-Fuel Ratio (AFR)
  - Average Fuel Consumption
  - Engine Health Status
- Modern racing dashboard UI with dark mode and smooth animations.
- Automatic reconnection to ECU in case of communication loss.
- Warning popups for critical engine conditions.

## Hardware Requirements
- **Microcontroller**: ESP32-S3
- **Display**: 5-inch TFT ILI9488 with touchscreen XPT2046
- **ECU Interface**: L9637D

## Pin Configuration
- **UART K-Line**:
  - RX: GPIO16
  - TX: GPIO17
- **TFT Display**:
  - MOSI: GPIO23
  - MISO: GPIO19
  - SCK: GPIO18
  - CS: GPIO5
  - DC: GPIO2
  - RST: GPIO4
- **Touchscreen**:
  - T_CS: GPIO15
  - T_IRQ: GPIO27

## Installation
1. Clone the repository:
   ```
   git clone https://github.com/yourusername/MotorcycleECUDashboard.git
   ```
2. Navigate to the project directory:
   ```
   cd MotorcycleECUDashboard
   ```
3. Open the project in your preferred IDE and ensure you have the necessary libraries installed:
   - TFT_eSPI
   - LVGL
   - HardwareSerial
   - ArduinoJson
   - XPT2046_Touchscreen
   - SPI
   - Wire

4. Configure the `platformio.ini` file as needed for your environment.

## Usage
- Upload the code to the ESP32-S3 using PlatformIO.
- Connect the hardware as per the pin configuration.
- Power on the system and the dashboard will initialize, displaying real-time data from the motorcycle's ECU.

## Contributing
Contributions are welcome! Please feel free to submit a pull request or open an issue for any enhancements or bug fixes.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.