#pragma once
/**
 * ============================================================
 * DashboardUI.h
 * Racing Dashboard TFT Display Manager
 * TFT 5" ILI9488 + XPT2046 Touchscreen
 * ============================================================
 * Mengelola tampilan dashboard racing modern dark mode
 * Layout: RPM gauge atas, speed tengah, info bawah
 * ============================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include "ECUManager.h"
#include "Analyzers.h"

// ─── TFT Pin Configuration ────────────────────────────────────
// Pin dikonfigurasi di User_Setup.h TFT_eSPI
// MOSI=23, MISO=19, SCK=18, CS=5, DC=2, RST=4

// ─── Touch Pin Configuration ─────────────────────────────────
#define TOUCH_CS_PIN    15
#define TOUCH_IRQ_PIN   27

// ─── Display Dimensions (ILI9488 5") ─────────────────────────
#define TFT_WIDTH       480
#define TFT_HEIGHT      320

// ─── Color Palette (16-bit RGB565) ───────────────────────────
#define COLOR_BG        0x0841  // Very dark blue-gray (#080808)
#define COLOR_BG2       0x10A3  // Dark panel (#102030)
#define COLOR_ACCENT    0x051F  // Electric blue (#0040FF)
#define COLOR_ACCENT2   0x07FF  // Cyan (#00FFFF)
#define COLOR_RED       0xF800  // Pure red
#define COLOR_RED_DIM   0x7800  // Dim red
#define COLOR_GREEN     0x07E0  // Pure green
#define COLOR_GREEN_DIM 0x0380  // Dim green
#define COLOR_YELLOW    0xFFE0  // Yellow
#define COLOR_ORANGE    0xFC20  // Orange
#define COLOR_WHITE     0xFFFF  // White
#define COLOR_GRAY      0x7BEF  // Mid gray
#define COLOR_DARK_GRAY 0x4208  // Dark gray
#define COLOR_RPM_BAR   0x051F  // RPM bar blue

// ─── Layout Zones ─────────────────────────────────────────────
// AREA ATAS: y=0 to y=150
#define ZONE_TOP_Y      0
#define ZONE_TOP_H      150
// AREA TENGAH: y=150 to y=230
#define ZONE_MID_Y      150
#define ZONE_MID_H      80
// AREA BAWAH: y=230 to y=320
#define ZONE_BOT_Y      230
#define ZONE_BOT_H      90

// ─── RPM Gauge ────────────────────────────────────────────────
#define RPM_GAUGE_CX    240     // Center X
#define RPM_GAUGE_CY    85      // Center Y
#define RPM_GAUGE_R     75      // Radius outer
#define RPM_GAUGE_IR    55      // Radius inner arc
#define RPM_MAX         12000   // Max RPM

// ─── Shift Light Thresholds ───────────────────────────────────
#define SHIFT_RPM_1     7000    // Green
#define SHIFT_RPM_2     9000    // Yellow
#define SHIFT_RPM_3     10500   // Red blink

// ─── Warning Types ────────────────────────────────────────────
enum class WarningType {
    NONE,
    BATTERY_LOW,
    AFR_LEAN,
    AFR_RICH,
    OVERHEAT,
    ECU_LOST
};

// ─── Sprite Buffer ────────────────────────────────────────────
// Gunakan TFT_eSPI sprites untuk smooth rendering

// ─── DashboardUI Class ────────────────────────────────────────
class DashboardUI {
public:
    DashboardUI();
    bool begin();
    void update(const ECUData& data,
                const AFRAnalyzer& afr,
                const FuelAnalyzer& fuel,
                const EngineHealthAnalyzer& health);

    // Warning system
    void showWarning(WarningType type, const String& message);
    void clearWarning();
    WarningType checkWarnings(const ECUData& data,
                               const AFRAnalyzer& afr);

    // Touch handler
    void handleTouch();

private:
    TFT_eSPI         _tft;
    TFT_eSprite      _spriteTop;     // Top area sprite
    TFT_eSprite      _spriteMid;     // Middle area sprite
    TFT_eSprite      _spriteBot;     // Bottom area sprite
    XPT2046_Touchscreen _touch;

    WarningType _activeWarning;
    uint32_t    _warningStartMs;
    bool        _warningVisible;
    uint32_t    _lastFullRedraw;

    // Cached previous values untuk dirty rendering
    uint16_t _prevRPM;
    uint8_t  _prevSpeed;
    float    _prevAFR;
    float    _prevVoltage;
    float    _prevFuel;
    uint8_t  _prevHealth;
    bool     _needFullRedraw;

    // ─── Drawing Methods ──────────────────────────────────────
    void drawBackground();
    void drawTopArea(uint16_t rpm, bool connected);
    void drawRPMGauge(uint16_t rpm);
    void drawRPMArc(uint16_t rpm);
    void drawRPMText(uint16_t rpm);
    void drawShiftIndicator(uint16_t rpm);
    void drawMidArea(uint8_t speed, float afr, float voltage);
    void drawSpeedDigital(uint8_t speed);
    void drawAFRDisplay(float afr, const String& status);
    void drawVoltageDisplay(float voltage);
    void drawBotArea(float fuelInstant, float fuelAvg,
                     uint8_t health, const String& healthStatus,
                     uint32_t healthColor);
    void drawFuelConsumption(float instant, float avg);
    void drawEngineHealth(uint8_t percent, const String& status,
                          uint32_t color);
    void drawConnectionStatus(bool connected);
    void drawWarningPopup(WarningType type, const String& message);

    // Utility drawing
    void drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                       int16_t r, uint32_t color, bool filled = true);
    void drawGaugeTick(int16_t cx, int16_t cy, int16_t r1, int16_t r2,
                       float angleDeg, uint32_t color, uint8_t thick = 2);
    void drawArcSegment(int16_t cx, int16_t cy, int16_t r,
                        float startAngle, float endAngle,
                        uint32_t color, uint8_t thick = 4);
    void drawGlowText(const char* text, int16_t x, int16_t y,
                      uint8_t font, uint32_t color, uint32_t glowColor);

    // Static layout elements
    void drawStaticLayout();
    void drawDividers();
    void drawLabels();

    // Color helpers
    uint32_t rpmToColor(uint16_t rpm);
    uint32_t afrToColor(float afr);
    uint32_t voltageToColor(float voltage);

    // Animation
    uint32_t _animFrame;
    bool     _blinkState;
    uint32_t _lastBlinkMs;
};
