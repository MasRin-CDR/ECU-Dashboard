/**
 * ============================================================
 * DashboardUI.cpp
 * Implementasi Racing Dashboard TFT Display
 * ============================================================
 */

#include "DashboardUI.h"
#include <math.h>

// ─── Constructor ─────────────────────────────────────────────
DashboardUI::DashboardUI()
    : _spriteTop(&_tft),
      _spriteMid(&_tft),
      _spriteBot(&_tft),
      _touch(TOUCH_CS_PIN, TOUCH_IRQ_PIN),
      _activeWarning(WarningType::NONE),
      _warningStartMs(0),
      _warningVisible(false),
      _lastFullRedraw(0),
      _prevRPM(0), _prevSpeed(0), _prevAFR(0),
      _prevVoltage(0), _prevFuel(0), _prevHealth(0),
      _needFullRedraw(true),
      _animFrame(0), _blinkState(false), _lastBlinkMs(0)
{
}

// ─── Begin ────────────────────────────────────────────────────
bool DashboardUI::begin() {
    Serial.println("[UI] Initializing TFT display...");

    // Init TFT
    _tft.init();
    _tft.setRotation(1); // Landscape
    _tft.fillScreen(COLOR_BG);

    // Init touchscreen
    _touch.begin();
    _touch.setRotation(1);

    Serial.println("[UI] TFT initialized");

    // Inisialisasi sprites untuk area utama
    // Top sprite: 480 x 150
    _spriteTop.createSprite(TFT_WIDTH, ZONE_TOP_H);
    _spriteTop.setColorDepth(16);

    // Mid sprite: 480 x 80
    _spriteMid.createSprite(TFT_WIDTH, ZONE_MID_H);
    _spriteMid.setColorDepth(16);

    // Bot sprite: 480 x 90
    _spriteBot.createSprite(TFT_WIDTH, ZONE_BOT_H);
    _spriteBot.setColorDepth(16);

    // Gambar splash screen
    _tft.fillScreen(COLOR_BG);
    _tft.setTextColor(COLOR_ACCENT2, COLOR_BG);
    _tft.setTextDatum(MC_DATUM);
    _tft.setTextSize(3);
    _tft.drawString("ECU DASHBOARD", TFT_WIDTH/2, TFT_HEIGHT/2 - 20);
    _tft.setTextSize(1);
    _tft.setTextColor(COLOR_GRAY, COLOR_BG);
    _tft.drawString("ESP32-S3 | K-Line | ILI9488", TFT_WIDTH/2, TFT_HEIGHT/2 + 20);
    _tft.drawString("Initializing...", TFT_WIDTH/2, TFT_HEIGHT/2 + 45);
    delay(1500);

    // Gambar layout statis
    drawBackground();
    drawStaticLayout();

    Serial.println("[UI] Dashboard ready");
    return true;
}

// ─── Main Update ─────────────────────────────────────────────
void DashboardUI::update(const ECUData& data,
                          const AFRAnalyzer& afr,
                          const FuelAnalyzer& fuel,
                          const EngineHealthAnalyzer& health)
{
    _animFrame++;

    // Blink state (500ms interval)
    if (millis() - _lastBlinkMs > 500) {
        _blinkState = !_blinkState;
        _lastBlinkMs = millis();
    }

    // Check warnings
    WarningType warn = checkWarnings(data, afr);
    if (warn != WarningType::NONE) {
        showWarning(warn, "");
    } else if (_activeWarning != WarningType::NONE) {
        clearWarning();
    }

    // Deteksi perubahan untuk partial redraw
    bool rpmChanged     = abs((int)data.rpm - (int)_prevRPM) > 10;
    bool speedChanged   = data.speed != _prevSpeed;
    bool afrChanged     = abs(data.afr - _prevAFR) > 0.05f;
    bool voltChanged    = abs(data.batteryVoltage - _prevVoltage) > 0.05f;
    bool fuelChanged    = abs(fuel.getInstantConsumption() - _prevFuel) > 0.1f;
    bool healthChanged  = data.engineHealth != _prevHealth;

    // Update AREA ATAS jika RPM berubah (paling sering)
    if (_needFullRedraw || rpmChanged) {
        drawTopArea(data.rpm, data.valid);
        _prevRPM = data.rpm;
    }

    // Update AREA TENGAH
    if (_needFullRedraw || speedChanged || afrChanged || voltChanged) {
        drawMidArea(data.speed, data.afr, data.batteryVoltage);
        _prevSpeed   = data.speed;
        _prevAFR     = data.afr;
        _prevVoltage = data.batteryVoltage;
    }

    // Update AREA BAWAH (lebih jarang)
    if (_needFullRedraw || fuelChanged || healthChanged) {
        drawBotArea(
            fuel.getInstantConsumption(),
            fuel.getAverageConsumption(),
            health.getHealthPercent(),
            health.getStatusString(),
            health.getStatusColor()
        );
        _prevFuel   = fuel.getInstantConsumption();
        _prevHealth = data.engineHealth;
    }

    // Warning popup (overlay)
    if (_warningVisible) {
        drawWarningPopup(_activeWarning, "");
    }

    _needFullRedraw = false;
}

// ─── Draw Background ─────────────────────────────────────────
void DashboardUI::drawBackground() {
    _tft.fillScreen(COLOR_BG);

    // Gradient-like background menggunakan horizontal bands
    for (int y = 0; y < TFT_HEIGHT; y++) {
        // Subtle blue tint yang makin gelap ke bawah
        uint8_t blend = (y * 3) / TFT_HEIGHT;
        uint32_t color = _tft.color565(8 + blend, 12 + blend, 20 + blend * 2);
        _tft.drawFastHLine(0, y, TFT_WIDTH, color);
    }
}

// ─── Draw Static Layout ───────────────────────────────────────
void DashboardUI::drawStaticLayout() {
    drawDividers();
    drawLabels();
}

// ─── Draw Dividers ────────────────────────────────────────────
void DashboardUI::drawDividers() {
    // Garis horizontal pembatas zona
    _tft.drawFastHLine(0, ZONE_MID_Y - 1, TFT_WIDTH, COLOR_ACCENT);
    _tft.drawFastHLine(0, ZONE_MID_Y,     TFT_WIDTH, COLOR_BG2);
    _tft.drawFastHLine(0, ZONE_BOT_Y - 1, TFT_WIDTH, COLOR_ACCENT);
    _tft.drawFastHLine(0, ZONE_BOT_Y,     TFT_WIDTH, COLOR_BG2);

    // Garis vertikal pemisah tengah
    _tft.drawFastVLine(160, ZONE_MID_Y, ZONE_MID_H, COLOR_DARK_GRAY);
    _tft.drawFastVLine(320, ZONE_MID_Y, ZONE_MID_H, COLOR_DARK_GRAY);

    // Garis vertikal pemisah bawah
    _tft.drawFastVLine(240, ZONE_BOT_Y, ZONE_BOT_H, COLOR_DARK_GRAY);
}

// ─── Draw Labels ──────────────────────────────────────────────
void DashboardUI::drawLabels() {
    _tft.setTextDatum(TC_DATUM);
    _tft.setTextColor(COLOR_DARK_GRAY, COLOR_BG);
    _tft.setTextSize(1);

    // Label area tengah
    _tft.drawString("KM/H",   80,  ZONE_MID_Y + 3);
    _tft.drawString("AFR",   240,  ZONE_MID_Y + 3);
    _tft.drawString("VOLT",  400,  ZONE_MID_Y + 3);

    // Label area bawah
    _tft.drawString("FUEL ECONOMY",   120,  ZONE_BOT_Y + 3);
    _tft.drawString("ENGINE HEALTH",  360,  ZONE_BOT_Y + 3);
}

// ─── Draw Top Area ────────────────────────────────────────────
void DashboardUI::drawTopArea(uint16_t rpm, bool connected) {
    _spriteTop.fillSprite(COLOR_BG);

    // Background panel untuk RPM
    _spriteTop.fillRoundRect(0, 0, TFT_WIDTH, ZONE_TOP_H - 2, 0, COLOR_BG);

    // Shift light bar (atas)
    drawShiftIndicator(rpm);

    // RPM Gauge arc
    drawRPMArc(rpm);

    // RPM Text
    drawRPMText(rpm);

    // Status koneksi
    drawConnectionStatus(connected);

    // Push sprite ke TFT
    _spriteTop.pushSprite(0, ZONE_TOP_Y);
}

// ─── Draw RPM Arc ─────────────────────────────────────────────
void DashboardUI::drawRPMArc(uint16_t rpm) {
    // Gauge span: 210° (dari -105° ke +105°, 0° = bawah)
    const float START_ANGLE = 135.0f;  // degrees (dari kiri bawah)
    const float END_ANGLE   = 405.0f;  // degrees (ke kanan bawah)
    const float ARC_SPAN    = END_ANGLE - START_ANGLE; // 270°

    int16_t cx = RPM_GAUGE_CX;
    int16_t cy = RPM_GAUGE_CY;

    // Background arc (seluruh span, gelap)
    drawArcSegment(cx, cy, RPM_GAUGE_IR, START_ANGLE, END_ANGLE,
                   COLOR_DARK_GRAY, 8);

    // Active RPM arc
    if (rpm > 0) {
        float rpmFraction = min(1.0f, (float)rpm / RPM_MAX);
        float activeEnd   = START_ANGLE + (rpmFraction * ARC_SPAN);
        uint32_t arcColor = rpmToColor(rpm);
        drawArcSegment(cx, cy, RPM_GAUGE_IR, START_ANGLE, activeEnd,
                       arcColor, 8);

        // Glow layer (tipis, lebih terang)
        drawArcSegment(cx, cy, RPM_GAUGE_IR + 1, START_ANGLE, activeEnd,
                       arcColor | 0x0820, 3);
    }

    // Tick marks setiap 1000 RPM
    for (int r = 0; r <= 12; r++) {
        float fraction = (float)r / 12.0f;
        float angle = START_ANGLE + fraction * ARC_SPAN;
        float rad = angle * M_PI / 180.0f;

        bool isMajor = (r % 2 == 0);
        int16_t r1 = isMajor ? RPM_GAUGE_R - 2  : RPM_GAUGE_R - 8;
        int16_t r2 = RPM_GAUGE_R + 5;

        int16_t x1 = cx + r1 * cos(rad);
        int16_t y1 = cy + r1 * sin(rad);
        int16_t x2 = cx + r2 * cos(rad);
        int16_t y2 = cy + r2 * sin(rad);

        uint32_t tickColor = (r * 1000 <= rpm) ? rpmToColor(rpm) : COLOR_DARK_GRAY;
        _spriteTop.drawLine(x1, y1, x2, y2, tickColor);

        // Label RPM untuk major ticks
        if (isMajor && r > 0) {
            int16_t labelR = RPM_GAUGE_R + 16;
            int16_t lx = cx + labelR * cos(rad);
            int16_t ly = cy + labelR * sin(rad);
            _spriteTop.setTextDatum(MC_DATUM);
            _spriteTop.setTextColor(COLOR_GRAY, COLOR_BG);
            _spriteTop.setTextSize(1);
            String label = String(r * 1);
            _spriteTop.drawString(label, lx, ly);
        }
    }

    // Center circle decoration
    _spriteTop.fillCircle(cx, cy, 18, COLOR_BG2);
    _spriteTop.drawCircle(cx, cy, 18, COLOR_ACCENT);
    _spriteTop.fillCircle(cx, cy, 4, COLOR_ACCENT2);
}

// ─── Draw RPM Text ────────────────────────────────────────────
void DashboardUI::drawRPMText(uint16_t rpm) {
    int16_t cx = RPM_GAUGE_CX;
    int16_t cy = RPM_GAUGE_CY;

    // RPM value
    char rpmStr[8];
    snprintf(rpmStr, sizeof(rpmStr), "%5d", rpm);

    _spriteTop.setTextDatum(MC_DATUM);
    _spriteTop.setTextColor(rpmToColor(rpm), COLOR_BG);
    _spriteTop.setTextSize(3);
    _spriteTop.drawString(rpmStr, cx, cy + 45);

    // RPM label
    _spriteTop.setTextSize(1);
    _spriteTop.setTextColor(COLOR_DARK_GRAY, COLOR_BG);
    _spriteTop.drawString("RPM", cx, cy + 65);
}

// ─── Draw Shift Indicator ─────────────────────────────────────
void DashboardUI::drawShiftIndicator(uint16_t rpm) {
    // 10 LED strip di atas
    const int LED_COUNT = 10;
    const int LED_W = 44;
    const int LED_H = 10;
    const int LED_GAP = 4;
    const int LED_Y = 2;
    int startX = (TFT_WIDTH - (LED_COUNT * (LED_W + LED_GAP) - LED_GAP)) / 2;

    for (int i = 0; i < LED_COUNT; i++) {
        int x = startX + i * (LED_W + LED_GAP);
        float ledFraction = (float)(i + 1) / LED_COUNT;
        uint16_t ledRPM = (uint16_t)(ledFraction * RPM_MAX);
        bool active = (rpm >= ledRPM);

        // Warna LED berdasarkan posisi
        uint32_t ledColor;
        if (i < 4)      ledColor = active ? COLOR_GREEN      : COLOR_DARK_GRAY;
        else if (i < 7) ledColor = active ? COLOR_YELLOW     : COLOR_DARK_GRAY;
        else            ledColor = active ? COLOR_RED         : COLOR_DARK_GRAY;

        // Blink di RPM kritis
        if (i >= 9 && rpm > SHIFT_RPM_3 && !_blinkState) {
            ledColor = COLOR_DARK_GRAY;
        }

        _spriteTop.fillRoundRect(x, LED_Y, LED_W, LED_H, 3, ledColor);

        // Glow efek untuk LED aktif
        if (active) {
            _spriteTop.drawRoundRect(x - 1, LED_Y - 1, LED_W + 2, LED_H + 2,
                                     3, ledColor | 0x1861);
        }
    }
}

// ─── Draw Mid Area ────────────────────────────────────────────
void DashboardUI::drawMidArea(uint8_t speed, float afr, float voltage) {
    _spriteMid.fillSprite(COLOR_BG);

    // Panel backgrounds
    _spriteMid.fillRect(0, 0, 160, ZONE_MID_H, COLOR_BG2);
    _spriteMid.fillRect(161, 0, 158, ZONE_MID_H, COLOR_BG);
    _spriteMid.fillRect(321, 0, 159, ZONE_MID_H, COLOR_BG2);

    // Speed
    drawSpeedDigital(speed);

    // AFR
    // Re-create AFRAnalyzer data from float
    String afrStatus;
    if (afr > AFR_LEAN_THRESHOLD) afrStatus = "LEAN";
    else if (afr < AFR_RICH_THRESHOLD) afrStatus = "RICH";
    else afrStatus = "NORMAL";
    drawAFRDisplay(afr, afrStatus);

    // Voltage
    drawVoltageDisplay(voltage);

    _spriteMid.pushSprite(0, ZONE_MID_Y);
}

// ─── Draw Speed Digital ───────────────────────────────────────
void DashboardUI::drawSpeedDigital(uint8_t speed) {
    _spriteMid.setTextDatum(MC_DATUM);

    // Speed value - besar
    char speedStr[4];
    snprintf(speedStr, sizeof(speedStr), "%3d", speed);
    _spriteMid.setTextColor(COLOR_WHITE, COLOR_BG2);
    _spriteMid.setTextSize(5); // Besar
    _spriteMid.drawString(speedStr, 80, ZONE_MID_H / 2 + 8);
}

// ─── Draw AFR Display ─────────────────────────────────────────
void DashboardUI::drawAFRDisplay(float afr, const String& status) {
    uint32_t afrColor = afrToColor(afr);

    _spriteMid.setTextDatum(MC_DATUM);
    _spriteMid.setTextColor(afrColor, COLOR_BG);

    // AFR value
    char afrStr[7];
    snprintf(afrStr, sizeof(afrStr), "%.2f", afr);
    _spriteMid.setTextSize(3);
    _spriteMid.drawString(afrStr, 240, ZONE_MID_H / 2 + 5);

    // Status label (Lean/Normal/Rich)
    _spriteMid.setTextSize(1);
    _spriteMid.setTextColor(afrColor, COLOR_BG);
    _spriteMid.drawString(status, 240, ZONE_MID_H - 12);

    // Lambda indicator bar
    float lambda = afr / 14.7f;
    int barW = 120;
    int barX = 240 - barW / 2;
    int barY = ZONE_MID_H - 22;
    _spriteMid.drawRect(barX, barY, barW, 6, COLOR_DARK_GRAY);
    // Center = stoich
    int lambdaX = barX + (int)((lambda / 2.0f) * barW);
    lambdaX = constrain(lambdaX, barX, barX + barW - 4);
    _spriteMid.fillRect(lambdaX, barY, 4, 6, afrColor);
    // Center marker
    _spriteMid.drawFastVLine(barX + barW / 2, barY - 2, 10, COLOR_GRAY);
}

// ─── Draw Voltage Display ─────────────────────────────────────
void DashboardUI::drawVoltageDisplay(float voltage) {
    uint32_t vColor = voltageToColor(voltage);

    _spriteMid.setTextDatum(MC_DATUM);
    _spriteMid.setTextColor(vColor, COLOR_BG2);

    char voltStr[7];
    snprintf(voltStr, sizeof(voltStr), "%.1fV", voltage);
    _spriteMid.setTextSize(3);
    _spriteMid.drawString(voltStr, 400, ZONE_MID_H / 2 + 5);

    // Battery icon indicator (simple bar)
    int batX = 360, batY = ZONE_MID_H - 22;
    int batW = 80, batH = 10;
    float batPercent = constrain((voltage - 11.0f) / 3.0f, 0.0f, 1.0f);
    _spriteMid.drawRect(batX, batY, batW, batH, COLOR_GRAY);
    _spriteMid.fillRect(batX + 1, batY + 1,
                        (int)((batW - 2) * batPercent), batH - 2, vColor);
}

// ─── Draw Bottom Area ─────────────────────────────────────────
void DashboardUI::drawBotArea(float fuelInstant, float fuelAvg,
                               uint8_t health, const String& healthStatus,
                               uint32_t healthColor) {
    _spriteBot.fillSprite(COLOR_BG);
    _spriteBot.fillRect(0,   0, 240, ZONE_BOT_H, COLOR_BG2);
    _spriteBot.fillRect(241, 0, 239, ZONE_BOT_H, COLOR_BG);

    drawFuelConsumption(fuelInstant, fuelAvg);
    drawEngineHealth(health, healthStatus, healthColor);

    _spriteBot.pushSprite(0, ZONE_BOT_Y);
}

// ─── Draw Fuel Consumption ────────────────────────────────────
void DashboardUI::drawFuelConsumption(float instant, float avg) {
    _spriteBot.setTextDatum(MC_DATUM);

    // Instant consumption - besar
    char instStr[8];
    snprintf(instStr, sizeof(instStr), "%.1f", instant);
    _spriteBot.setTextColor(COLOR_ACCENT2, COLOR_BG2);
    _spriteBot.setTextSize(3);
    _spriteBot.drawString(instStr, 90, ZONE_BOT_H / 2 + 5);

    _spriteBot.setTextSize(1);
    _spriteBot.setTextColor(COLOR_GRAY, COLOR_BG2);
    _spriteBot.drawString("km/L", 90, ZONE_BOT_H / 2 + 25);

    // Average consumption kecil
    char avgStr[12];
    snprintf(avgStr, sizeof(avgStr), "AVG: %.1f", avg);
    _spriteBot.setTextColor(COLOR_DARK_GRAY, COLOR_BG2);
    _spriteBot.setTextSize(1);
    _spriteBot.drawString(avgStr, 90, ZONE_BOT_H - 12);
}

// ─── Draw Engine Health ───────────────────────────────────────
void DashboardUI::drawEngineHealth(uint8_t percent, const String& status,
                                    uint32_t color) {
    _spriteBot.setTextDatum(MC_DATUM);

    // Percentage besar
    char pctStr[6];
    snprintf(pctStr, sizeof(pctStr), "%d%%", percent);
    _spriteBot.setTextColor(color, COLOR_BG);
    _spriteBot.setTextSize(3);
    _spriteBot.drawString(pctStr, 360, ZONE_BOT_H / 2 + 5);

    // Health bar
    int barW = 180, barH = 8;
    int barX = 271, barY = ZONE_BOT_H - 28;
    _spriteBot.drawRect(barX, barY, barW, barH, COLOR_DARK_GRAY);
    int fillW = (int)((float)barW * percent / 100.0f);
    if (fillW > 0) {
        _spriteBot.fillRect(barX + 1, barY + 1, fillW - 2, barH - 2, color);
    }

    // Status text
    _spriteBot.setTextSize(1);
    _spriteBot.setTextColor(color, COLOR_BG);
    _spriteBot.drawString(status, 360, ZONE_BOT_H - 12);
}

// ─── Draw Connection Status ───────────────────────────────────
void DashboardUI::drawConnectionStatus(bool connected) {
    String statusStr = connected ? "ECU OK" : "ECU LOST";
    uint32_t color   = connected ? COLOR_GREEN : COLOR_RED;

    // Blink jika disconnected
    if (!connected && !_blinkState) return;

    _spriteTop.setTextDatum(TR_DATUM);
    _spriteTop.setTextColor(color, COLOR_BG);
    _spriteTop.setTextSize(1);
    _spriteTop.drawString(statusStr, TFT_WIDTH - 5, ZONE_TOP_H - 18);

    // Status dot
    _spriteTop.fillCircle(TFT_WIDTH - 5 - 50, ZONE_TOP_H - 14,
                          4, connected ? COLOR_GREEN : COLOR_RED);
}

// ─── Draw Warning Popup ───────────────────────────────────────
void DashboardUI::drawWarningPopup(WarningType type, const String& msg) {
    // Popup di tengah layar
    const int PW = 300, PH = 80;
    const int PX = (TFT_WIDTH - PW) / 2;
    const int PY = (TFT_HEIGHT - PH) / 2;

    String warningText;
    uint32_t warnColor;

    switch (type) {
        case WarningType::BATTERY_LOW:
            warningText = "⚡ BATTERY LOW";
            warnColor   = COLOR_YELLOW;
            break;
        case WarningType::AFR_LEAN:
            warningText = "⚠ AFR LEAN";
            warnColor   = COLOR_ORANGE;
            break;
        case WarningType::AFR_RICH:
            warningText = "⚠ AFR RICH";
            warnColor   = COLOR_ORANGE;
            break;
        case WarningType::OVERHEAT:
            warningText = "🌡 OVERHEAT!";
            warnColor   = COLOR_RED;
            break;
        case WarningType::ECU_LOST:
            warningText = "⛔ ECU DISCONNECTED";
            warnColor   = COLOR_RED;
            break;
        default:
            return;
    }

    // Blink effect
    if (!_blinkState) {
        _tft.fillRoundRect(PX, PY, PW, PH, 8, COLOR_BG2);
        _tft.drawRoundRect(PX, PY, PW, PH, 8, warnColor);
        _tft.setTextDatum(MC_DATUM);
        _tft.setTextColor(warnColor, COLOR_BG2);
        _tft.setTextSize(2);
        _tft.drawString(warningText, TFT_WIDTH / 2, TFT_HEIGHT / 2);
        _tft.setTextSize(1);
        _tft.setTextColor(COLOR_GRAY, COLOR_BG2);
        _tft.drawString("TAP TO DISMISS", TFT_WIDTH / 2, TFT_HEIGHT / 2 + 25);
    } else {
        _needFullRedraw = true; // Redraw saat blink off
    }
}

// ─── Check Warnings ───────────────────────────────────────────
WarningType DashboardUI::checkWarnings(const ECUData& data,
                                        const AFRAnalyzer& afr) {
    if (!data.valid) return WarningType::ECU_LOST;
    if (data.batteryVoltage < 11.5f && data.batteryVoltage > 1.0f)
        return WarningType::BATTERY_LOW;
    if (afr.getAFR() > 17.0f) return WarningType::AFR_LEAN;
    if (afr.getAFR() < 12.0f) return WarningType::AFR_RICH;
    if (data.coolantTemp > 105) return WarningType::OVERHEAT;
    return WarningType::NONE;
}

// ─── Show Warning ─────────────────────────────────────────────
void DashboardUI::showWarning(WarningType type, const String& msg) {
    _activeWarning   = type;
    _warningVisible  = true;
    _warningStartMs  = millis();
}

// ─── Clear Warning ────────────────────────────────────────────
void DashboardUI::clearWarning() {
    _activeWarning  = WarningType::NONE;
    _warningVisible = false;
    _needFullRedraw = true;
}

// ─── Handle Touch ─────────────────────────────────────────────
void DashboardUI::handleTouch() {
    if (!_touch.touched()) return;

    TS_Point p = _touch.getPoint();

    // Dismiss warning
    if (_warningVisible) {
        clearWarning();
        return;
    }

    Serial.printf("[Touch] x=%d y=%d\n", p.x, p.y);
}

// ─── Arc Segment Drawing (Helper) ────────────────────────────
void DashboardUI::drawArcSegment(int16_t cx, int16_t cy, int16_t r,
                                  float startAngle, float endAngle,
                                  uint32_t color, uint8_t thick) {
    float step = 1.0f; // 1 degree steps
    for (float a = startAngle; a <= endAngle; a += step) {
        float rad = a * M_PI / 180.0f;
        for (int t = 0; t < thick; t++) {
            int16_t x = cx + (r - t) * cos(rad);
            int16_t y = cy + (r - t) * sin(rad);
            if (x >= 0 && x < TFT_WIDTH && y >= 0 && y < ZONE_TOP_H) {
                _spriteTop.drawPixel(x, y, color);
            }
        }
    }
}

// ─── Color Helper: RPM ────────────────────────────────────────
uint32_t DashboardUI::rpmToColor(uint16_t rpm) {
    float fraction = (float)rpm / RPM_MAX;
    if (fraction < 0.4f)  return COLOR_ACCENT2;    // Cyan - rendah
    if (fraction < 0.65f) return COLOR_GREEN;       // Hijau - normal
    if (fraction < 0.80f) return COLOR_YELLOW;      // Kuning - tinggi
    if (fraction < 0.90f) return COLOR_ORANGE;      // Orange - sangat tinggi
    return COLOR_RED;                                // Merah - kritis
}

// ─── Color Helper: AFR ────────────────────────────────────────
uint32_t DashboardUI::afrToColor(float afr) {
    if (afr > AFR_LEAN_THRESHOLD)  return COLOR_YELLOW; // Lean
    if (afr < AFR_RICH_THRESHOLD)  return COLOR_RED;    // Rich
    return COLOR_GREEN;                                  // Normal
}

// ─── Color Helper: Voltage ────────────────────────────────────
uint32_t DashboardUI::voltageToColor(float voltage) {
    if (voltage < 11.5f) return COLOR_RED;
    if (voltage < 12.5f) return COLOR_YELLOW;
    if (voltage < 14.5f) return COLOR_GREEN;
    return COLOR_ORANGE; // Overcharge
}
