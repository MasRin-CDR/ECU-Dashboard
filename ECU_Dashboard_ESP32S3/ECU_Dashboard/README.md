# 🏍️ Motorcycle ECU Dashboard
## ESP32-S3 | K-Line | TFT 5" ILI9488

Dashboard ECU motor realtime berbasis ESP32-S3 dengan komunikasi K-Line (ISO9141/KWP2000)
dan tampilan TFT 5 inch touchscreen racing modern.

---

## 📁 Struktur File

```
ECU_Dashboard/
├── ECU_Dashboard.ino       ← Main sketch (entry point)
├── KLineProtocol.h/.cpp    ← K-Line protocol handler
├── ECUManager.h/.cpp       ← ECU communication manager
├── Analyzers.h/.cpp        ← AFR, Fuel, Engine Health analyzers
├── DashboardUI.h/.cpp      ← TFT display & UI
├── User_Setup.h            ← Konfigurasi TFT_eSPI (copy ke library)
├── platformio.ini          ← PlatformIO build config
└── README.md               ← Dokumentasi ini
```

---

## 🔌 Wiring / Pin Configuration

### K-Line ECU Interface (L9637D)
```
ESP32-S3          L9637D
GPIO17 (TX) ──── TX pin
GPIO16 (RX) ──── RX pin
3.3V ─────────── VCC
GND ──────────── GND
                 K-Line ──── ECU K-Line pin
```

### TFT Display (ILI9488 5")
```
ESP32-S3     TFT ILI9488
GPIO23 ───── MOSI (SDI)
GPIO19 ───── MISO (SDO)
GPIO18 ───── SCK (CLK)
GPIO5  ───── CS
GPIO2  ───── DC (A0)
GPIO4  ───── RST
3.3V   ───── VCC (atau 5V sesuai modul)
GND    ───── GND
```

### Touchscreen (XPT2046)
```
ESP32-S3     XPT2046
GPIO23 ───── T_DIN (shared MOSI)
GPIO19 ───── T_DO  (shared MISO)
GPIO18 ───── T_CLK (shared SCK)
GPIO15 ───── T_CS
GPIO27 ───── T_IRQ
```

### Battery Voltage Sensor
```
ESP32-S3     Voltage Divider
GPIO34 ───── Output
              R1=100kΩ (ke battery +)
              R2=33kΩ (ke GND)
              Rasio: ~3.7x
```

---

## 📦 Library Dependencies

### Wajib
- **TFT_eSPI** v2.5.34+ (Bodmer) → Display ILI9488
- **XPT2046_Touchscreen** v1.4.0+ (Paul Stoffregen) → Touch
- **ArduinoJson** v7.0+ → JSON parsing (opsional, untuk future features)

### Built-in ESP32
- HardwareSerial → K-Line UART
- SPI → TFT & Touch
- Wire → I2C (opsional)
- esp_task_wdt.h → Watchdog timer

### Instalasi via Arduino IDE
```
Sketch > Include Library > Manage Libraries
- Search "TFT_eSPI" → Install Bodmer's version
- Search "XPT2046_Touchscreen" → Install
- Search "ArduinoJson" → Install
```

### Instalasi via PlatformIO
```bash
# Gunakan platformio.ini yang sudah disediakan
pio run  # Otomatis download semua library
```

---

## ⚙️ Konfigurasi TFT_eSPI

**PENTING**: Copy `User_Setup.h` ke direktori library TFT_eSPI:

**Windows:**
```
C:\Users\[Username]\Documents\Arduino\libraries\TFT_eSPI\User_Setup.h
```

**Linux/Mac:**
```
~/Arduino/libraries/TFT_eSPI/User_Setup.h
```

Atau jika menggunakan PlatformIO, build flags sudah dikonfigurasi
di `platformio.ini` dan tidak perlu copy manual.

---

## 🖥️ Dashboard Layout

```
┌─────────────────────────────────────┐ ← 480px
│  [SHIFT LEDs: ■■■■■■■■■■]  ECU OK  │
│                                     │
│       ┌──────────────┐              │
│   10  │   ░░░░░░     │  12          │
│  11   │  ░░░░░░░░░   │   2          │
│  ──   │ ░           ░│  ──          │
│   9   │     ⊙ •     │   4          │
│  ──   │  [  4500  ] │  ──          │
│       │    RPM      │              │
│       └──────────────┘              │
│                            ← 150px (AREA ATAS)
├─────────────────────────────────────┤
│KM/H        AFR           VOLT       │
│                                     │
│  120    14.70 [NORMAL]   12.4V ████ │
│         ──────┼──────               │
│                            ← 80px  (AREA TENGAH)
├─────────────────────────────────────┤
│FUEL ECONOMY      ENGINE HEALTH      │
│                                     │
│ 42.5 km/L      87% ████████░        │
│ AVG: 38.2       [Excellent]         │
└─────────────────────────────────────┘ ← 90px (AREA BAWAH)
```

---

## 🔄 K-Line Protocol Flow

```
ECU Dashboard                    ECU
     │                            │
     │── 300ms LINE HIGH ─────────│  (W5)
     │                            │
     │── 5-baud addr 0x33 ───────→│  (Slow Init)
     │                            │
     │←──── 0x55 (sync) ──────────│
     │←──── Key Byte 1 ───────────│
     │←──── Key Byte 2 ───────────│
     │                            │
     │── ~Key Byte 2 ────────────→│  (confirm)
     │                            │
     │  === CONNECTED ===         │
     │                            │
     │── Mode01 PID0C (RPM) ─────→│
     │←── 41 0C AA BB ────────────│  RPM = (AA*256+BB)/4
     │                            │
     │── Mode01 PID0D (Speed) ───→│
     │←── 41 0D AA ───────────────│  Speed = AA km/h
     │                            │
     │  ... (repeat every 20ms)   │
```

---

## 📊 Kalkulasi Data

### RPM
```
RPM = ((A × 256) + B) / 4
```

### AFR
```
AFR = f(O2_voltage, MAP, TPS, RPM, FuelTrim)
Lambda = AFR / 14.7
Status: Lean > 15.5, Normal 13.5-15.5, Rich < 13.5
```

### Konsumsi BBM (km/L)
```
Injector_Duty = pulse_width / (60000 / RPM)
Fuel_Flow (cc/min) = injector_cc_per_min × duty
Fuel_L_per_Hour = fuel_flow × 60 / 1000
Consumption = speed_km_h / fuel_L_per_hour
```

### Engine Health Score
```
Score = (AFR_stability × 25%) +
        (Battery_voltage × 20%) +
        (Coolant_temp × 20%) +
        (Idle_RPM_stability × 15%) +
        (TPS × 10%) +
        (No_DTC × 10%)

Excellent: 80-100%
Good:      60-79%
Warning:   40-59%
Critical:  0-39%
```

---

## 🚨 Warning System

| Warning | Kondisi |
|---------|---------|
| BATTERY LOW | Tegangan < 11.5V |
| AFR LEAN | AFR > 17.0 |
| AFR RICH | AFR < 12.0 |
| OVERHEAT | Coolant temp > 105°C |
| ECU LOST | Tidak ada respons ECU > 3 detik |

---

## 🔧 Troubleshooting

### ECU tidak terdeteksi
1. Periksa wiring L9637D
2. Pastikan K-Line baud rate 10400 bps
3. Cek apakah motor sudah dalam posisi ignition ON
4. Monitor Serial untuk melihat log K-Line handshake

### Display tidak tampil
1. Periksa konfigurasi `User_Setup.h`
2. Pastikan driver `ILI9488_DRIVER` dipilih
3. Periksa SPI pins (MOSI=23, MISO=19, SCK=18)
4. Cek power supply TFT (3.3V atau 5V)

### Touch tidak responsif
1. Periksa pin CS=15 dan IRQ=27
2. Kalibrasi touchscreen jika perlu
3. Pastikan SPI frequency touch = 2.5 MHz

### Low FPS / Lambat
1. Kurangi area yang di-redraw (gunakan dirty flag)
2. Naikkan SPI frequency TFT (max 40MHz untuk ILI9488)
3. Pastikan ESP32-S3 berjalan di 240MHz

---

## 🔮 Rencana Pengembangan

- [ ] Wideband O2 sensor support (Innovate LC-2)
- [ ] GPS speed integration
- [ ] Data logging ke SD card
- [ ] Bluetooth telemetry ke smartphone
- [ ] Custom PID untuk ECU spesifik motor Indonesia
- [ ] Night mode / Day mode otomatis
- [ ] Multiple theme racing dashboard
- [ ] OTA firmware update via WiFi

---

## 📄 Lisensi

Open source untuk pengembangan hobi otomotif.
Gunakan sesuai kebutuhan dengan menyebutkan sumber.

---

*ECU Dashboard Project - ESP32-S3 Motorcycle Telemetry*
