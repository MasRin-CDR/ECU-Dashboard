#pragma once
// ╔══════════════════════════════════════════════════════════════╗
// ║          VARIO DASHBOARD — display.h                        ║
// ║  Tampilan TFT — layout dashboard modern                     ║
// ║  Layar ILI9341 landscape: 320×240 px                        ║
// ║  (Ubah W=480, H=320 jika pakai ILI9488 5")                  ║
// ╚══════════════════════════════════════════════════════════════╝

#include <TFT_eSPI.h>
#include "config.h"
#include "sensors.h"

TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);   // sprite untuk double-buffer

// ── Dimensi layar (sesuaikan dengan TFT yang dipakai) ──
#define SCR_W   480    // ganti 320 untuk ILI9341
#define SCR_H   320    // ganti 240 untuk ILI9341

// ─────────────────────────────────────────
//  LAYOUT ZONA
//
//  ┌──────────────────────────────────────────────────────────┐
//  │  STATUS BAR (atas 20px)                                  │
//  ├──────────────┬───────────────────────────────────────────┤
//  │              │  RPM BAR (kanan atas 30px)                │
//  │  SPEED       ├───────────────────────────────────────────┤
//  │  (kiri)      │  AFR │ TEMP │ VOLT                        │
//  │              ├───────────────────────────────────────────┤
//  │              │  FUEL │ HEALTH │ CONS                      │
//  ├──────────────┴───────────────────────────────────────────┤
//  │  STATUS BAR BAWAH (20px)                                  │
//  └──────────────────────────────────────────────────────────┘
// ─────────────────────────────────────────
#define ZONE_STATUS_H   22
#define ZONE_SPEED_W   160
#define ZONE_RPM_H      36
#define ZONE_PANEL_H    80   // tinggi tiap row panel bawah
#define ZONE_BOTTOM_H   22

// Panel atas kanan — koordinat
#define PANEL_R_X      (ZONE_SPEED_W + 4)
#define PANEL_R_Y      (ZONE_STATUS_H + ZONE_RPM_H + 4)
#define PANEL_R_W      (SCR_W - ZONE_SPEED_W - 8)

// ─────────────────────────────────────────
//  WARNA DINAMIS
// ─────────────────────────────────────────
static uint16_t afrColor(float afr) {
  if (afr < 12.5f || afr > 16.5f) return COL_RED;
  if (afr < 13.8f || afr > 15.5f) return COL_ORANGE;
  if (afr < 14.2f || afr > 15.1f) return COL_YELLOW;
  return COL_GREEN;
}
static uint16_t tempColor(float t) {
  if (t >= 110.0f) return COL_RED;
  if (t >=  95.0f) return COL_ORANGE;
  if (t <   40.0f) return COL_CYAN;
  return COL_GREEN;
}
static uint16_t voltColor(float v) {
  if (v < 11.0f || v > 15.5f) return COL_RED;
  if (v < 12.0f)               return COL_ORANGE;
  return COL_GREEN;
}
static uint16_t healthColor(float h) {
  if (h < 40.0f) return COL_RED;
  if (h < 70.0f) return COL_YELLOW;
  return COL_GREEN;
}
static uint16_t fuelColor(float f) {
  if (f < 10.0f) return COL_RED;
  if (f < 25.0f) return COL_ORANGE;
  return COL_CYAN;
}

// ─────────────────────────────────────────
//  HELPER: gambar bar horizontal
// ─────────────────────────────────────────
static void drawBar(int16_t x, int16_t y, int16_t w, int16_t h,
                    float val, float maxVal, uint16_t color,
                    bool showLabel = false, const char* lbl = nullptr) {
  int16_t fill = constrain((int16_t)((val / maxVal) * (w - 4)), 0, w - 4);
  spr.drawRoundRect(x, y, w, h, 3, COL_GREY);
  spr.fillRect(x + 2, y + 2, w - 4, h - 4, COL_DARKGREY);
  if (fill > 0)
    spr.fillRect(x + 2, y + 2, fill, h - 4, color);
  if (showLabel && lbl) {
    spr.setTextColor(COL_WHITE);
    spr.setTextSize(1);
    spr.setCursor(x + w + 4, y + h / 2 - 4);
    spr.print(lbl);
  }
}

// ─────────────────────────────────────────
//  HELPER: gambar blok info (label + nilai)
// ─────────────────────────────────────────
static void drawInfoBlock(int16_t x, int16_t y, int16_t w, int16_t h,
                           const char* label, const char* value,
                           uint16_t valColor, const char* subVal = nullptr) {
  // Border
  spr.drawRoundRect(x, y, w, h, 5, COL_DARKGREY);
  // Label kecil
  spr.setTextColor(COL_GREY);
  spr.setTextSize(1);
  spr.setCursor(x + 6, y + 5);
  spr.print(label);
  // Nilai besar
  spr.setTextColor(valColor);
  spr.setTextSize(2);
  int16_t vlen = strlen(value) * 12;
  spr.setCursor(x + (w - vlen) / 2, y + h / 2 - 2);
  spr.print(value);
  // Sub-nilai
  if (subVal) {
    spr.setTextColor(COL_DARKGREY);
    spr.setTextSize(1);
    spr.setCursor(x + 6, y + h - 14);
    spr.print(subVal);
  }
}

// ─────────────────────────────────────────
//  STATUS BAR ATAS
// ─────────────────────────────────────────
static void drawStatusBar(const SensorData& d) {
  spr.fillRect(0, 0, SCR_W, ZONE_STATUS_H, 0x1082);  // biru gelap

  // Judul
  spr.setTextColor(COL_AMBER);
  spr.setTextSize(1);
  spr.setCursor(6, 7);
  spr.print("VARIO 125");

  // ECU status
  spr.setCursor(90, 7);
  spr.setTextColor(d.ecuOnline ? COL_GREEN : COL_RED);
  spr.print(d.ecuOnline ? "ECU:ON " : "ECU:OFF");

  // Jarak
  spr.setTextColor(COL_GREY);
  spr.setCursor(SCR_W / 2 - 20, 7);
  char distBuf[16];
  snprintf(distBuf, sizeof(distBuf), "%4lu m", d.distanceM);
  spr.print(distBuf);

  // Waktu pakai (millis)
  spr.setCursor(SCR_W - 80, 7);
  uint32_t s = millis() / 1000;
  char tBuf[12];
  snprintf(tBuf, sizeof(tBuf), "%02lu:%02lu:%02lu", s/3600, (s%3600)/60, s%60);
  spr.print(tBuf);
}

// ─────────────────────────────────────────
//  SPEEDOMETER — angka besar di kiri
// ─────────────────────────────────────────
static void drawSpeedometer(const SensorData& d) {
  int16_t x = 0, y = ZONE_STATUS_H;
  int16_t w = ZONE_SPEED_W, h = SCR_H - ZONE_STATUS_H - ZONE_BOTTOM_H;

  // Background speed area
  spr.fillRect(x, y, w, h, 0x0821);

  // Label
  spr.setTextColor(COL_GREY);
  spr.setTextSize(1);
  spr.setCursor(x + 10, y + 10);
  spr.print("km/h");

  // Angka kecepatan — ukuran 7 (sangat besar)
  char spBuf[8];
  snprintf(spBuf, sizeof(spBuf), "%3d", (int)d.speedKmh);
  spr.setTextColor(d.speedValid ? COL_WHITE : COL_GREY);
  spr.setTextSize(6);
  int16_t tw = strlen(spBuf) * 36;
  spr.setCursor(x + (w - tw) / 2, y + 30);
  spr.print(spBuf);

  // Bar kecepatan
  uint16_t sCol = (d.speedKmh > 100) ? COL_RED
                : (d.speedKmh > 60)  ? COL_YELLOW : COL_CYAN;
  drawBar(x + 6, y + h - 40, w - 12, 14, d.speedKmh, 140, sCol);

  // Skala 0 / 70 / 140
  spr.setTextColor(COL_DARKGREY);
  spr.setTextSize(1);
  spr.setCursor(x + 6,       y + h - 50); spr.print("0");
  spr.setCursor(x + w/2 - 6, y + h - 50); spr.print("70");
  spr.setCursor(x + w - 18,  y + h - 50); spr.print("140");

  // Garis pemisah kanan
  spr.drawFastVLine(w, y, h, COL_DARKGREY);
}

// ─────────────────────────────────────────
//  RPM BAR (kanan atas lebar)
// ─────────────────────────────────────────
static void drawRPMBar(const SensorData& d) {
  int16_t x = ZONE_SPEED_W + 4;
  int16_t y = ZONE_STATUS_H + 4;
  int16_t w = SCR_W - ZONE_SPEED_W - 8;
  int16_t h = ZONE_RPM_H - 8;

  // Label RPM
  char rBuf[10];
  snprintf(rBuf, sizeof(rBuf), "%5d", d.rpm);
  spr.setTextColor(COL_AMBER);
  spr.setTextSize(2);
  spr.setCursor(x, y);
  spr.print(rBuf);
  spr.setTextColor(COL_GREY);
  spr.setTextSize(1);
  spr.setCursor(x + 65, y + 5);
  spr.print("RPM");

  // Bar RPM panjang
  uint16_t rCol = (d.rpm > 9000) ? COL_RED
                : (d.rpm > 7000) ? COL_ORANGE
                : (d.rpm > 5000) ? COL_YELLOW : COL_GREEN;
  int16_t bx = x + 90;
  int16_t bw = w - 94;
  drawBar(bx, y + 2, bw, h - 4, d.rpm, 11000, rCol);

  // Marker redline 9000 RPM
  int16_t rx = bx + (int16_t)(9000.0f / 11000.0f * (bw - 4)) + 2;
  spr.drawFastVLine(rx, y, h, COL_RED);
  spr.setTextColor(COL_RED);
  spr.setTextSize(1);
  spr.setCursor(rx - 6, y - 1);
  spr.print("9k");

  // Garis pemisah bawah RPM
  spr.drawFastHLine(ZONE_SPEED_W + 2, ZONE_STATUS_H + ZONE_RPM_H, w + 2, COL_DARKGREY);
}

// ─────────────────────────────────────────
//  PANEL ROW 1: AFR | SUHU | VOLTASE
// ─────────────────────────────────────────
static void drawPanelRow1(const SensorData& d) {
  int16_t y  = ZONE_STATUS_H + ZONE_RPM_H + 4;
  int16_t x0 = ZONE_SPEED_W + 4;
  int16_t pw = (SCR_W - ZONE_SPEED_W - 12) / 3;
  int16_t ph = ZONE_PANEL_H;
  int16_t gap = 4;

  // ── AFR ──
  char afrBuf[8], afrSub[16];
  snprintf(afrBuf, sizeof(afrBuf), "%.1f", d.afr);
  const char* afrLbl = (d.afr < 14.2f) ? "RICH" :
                       (d.afr > 15.1f) ? "LEAN" : "STOICH";
  snprintf(afrSub, sizeof(afrSub), "λ=%.3f %s", d.afr / 14.7f, afrLbl);
  drawInfoBlock(x0, y, pw, ph, "AFR", afrBuf, afrColor(d.afr), afrSub);

  // Bar AFR (stoich marker)
  drawBar(x0 + 4, y + ph - 16, pw - 8, 8,
          d.afr - 9.0f, 13.0f, afrColor(d.afr));
  int16_t sx = x0 + 4 + (int16_t)((14.7f - 9.0f) / 13.0f * (pw - 12));
  spr.drawFastVLine(sx, y + ph - 18, 12, COL_WHITE);

  // ── SUHU MESIN ──
  char tmpBuf[8], tmpSub[14];
  snprintf(tmpBuf, sizeof(tmpBuf), "%d\xB0", (int)d.engineTempC);
  const char* tmpLbl = (d.engineTempC >= 110) ? "OVERHEAT" :
                       (d.engineTempC >=  95) ? "HOT" :
                       (d.engineTempC <   40) ? "COLD" : "NORMAL";
  snprintf(tmpSub, sizeof(tmpSub), "%s", tmpLbl);
  int16_t x1 = x0 + pw + gap;
  drawInfoBlock(x1, y, pw, ph, "ENGINE TEMP", tmpBuf,
                d.tempValid ? tempColor(d.engineTempC) : COL_GREY, tmpSub);
  drawBar(x1 + 4, y + ph - 16, pw - 8, 8,
          d.engineTempC + 40, 190, tempColor(d.engineTempC));

  // ── VOLTASE ──
  char vBuf[8], vSub[14];
  snprintf(vBuf, sizeof(vBuf), "%.1fV", d.battVolt);
  const char* vLbl = (d.battVolt > 13.5f) ? "CHARGING" :
                     (d.battVolt < 11.5f) ? "LOW" : "OK";
  snprintf(vSub, sizeof(vSub), "%s", vLbl);
  int16_t x2 = x1 + pw + gap;
  drawInfoBlock(x2, y, pw, ph, "BATTERY", vBuf,
                d.battValid ? voltColor(d.battVolt) : COL_GREY, vSub);
  drawBar(x2 + 4, y + ph - 16, pw - 8, 8,
          d.battVolt - 10.0f, 7.0f, voltColor(d.battVolt));
}

// ─────────────────────────────────────────
//  PANEL ROW 2: BENSIN | HEALTH | KONSUMSI
// ─────────────────────────────────────────
static void drawPanelRow2(const SensorData& d) {
  int16_t y  = ZONE_STATUS_H + ZONE_RPM_H + ZONE_PANEL_H + 8;
  int16_t x0 = ZONE_SPEED_W + 4;
  int16_t pw = (SCR_W - ZONE_SPEED_W - 12) / 3;
  int16_t ph = ZONE_PANEL_H;
  int16_t gap = 4;

  // ── BENSIN ──
  char fBuf[8], fSub[14];
  snprintf(fBuf, sizeof(fBuf), "%d%%", (int)d.fuelPct);
  float fuelLit = d.fuelPct / 100.0f * TANK_CAPACITY_L;
  snprintf(fSub, sizeof(fSub), "%.1f L / 5.5L", fuelLit);
  drawInfoBlock(x0, y, pw, ph, "FUEL TANK", fBuf,
                fuelColor(d.fuelPct), fSub);
  // Bar bensin bergaya kotak
  drawBar(x0 + 4, y + ph - 16, pw - 8, 8,
          d.fuelPct, 100, fuelColor(d.fuelPct));

  // ── ENGINE HEALTH ──
  char hBuf[8], hSub[14];
  snprintf(hBuf, sizeof(hBuf), "%d%%", (int)d.engineHealthPct);
  const char* hLbl = (d.engineHealthPct >= 85) ? "EXCELLENT" :
                     (d.engineHealthPct >= 70) ? "GOOD" :
                     (d.engineHealthPct >= 50) ? "FAIR" : "POOR";
  int16_t x1 = x0 + pw + gap;
  drawInfoBlock(x1, y, pw, ph, "ENGINE HEALTH", hBuf,
                healthColor(d.engineHealthPct), hLbl);
  drawBar(x1 + 4, y + ph - 16, pw - 8, 8,
          d.engineHealthPct, 100, healthColor(d.engineHealthPct));

  // ── KONSUMSI BBM ──
  char cBuf[10], cSub[16];
  if (d.fuelConsKmL < 1.0f) {
    snprintf(cBuf, sizeof(cBuf), "---");
    snprintf(cSub, sizeof(cSub), "Menghitung...");
  } else {
    snprintf(cBuf, sizeof(cBuf), "%.1f", d.fuelConsKmL);
    snprintf(cSub, sizeof(cSub), "km/L avg");
  }
  int16_t x2 = x1 + pw + gap;
  uint16_t cCol = (d.fuelConsKmL < 25) ? COL_RED :
                  (d.fuelConsKmL < 35) ? COL_ORANGE : COL_GREEN;
  drawInfoBlock(x2, y, pw, ph, "AVG CONS", cBuf, cCol, cSub);
  if (d.fuelConsKmL >= 1.0f) {
    drawBar(x2 + 4, y + ph - 16, pw - 8, 8,
            d.fuelConsKmL, 60, cCol);
  }
}

// ─────────────────────────────────────────
//  STATUS BAR BAWAH
// ─────────────────────────────────────────
static void drawStatusBottom(const SensorData& d) {
  int16_t y = SCR_H - ZONE_BOTTOM_H;
  spr.fillRect(0, y, SCR_W, ZONE_BOTTOM_H, 0x1082);

  spr.setTextColor(COL_DARKGREY);
  spr.setTextSize(1);

  // Sensor validity flags
  char flagBuf[48];
  snprintf(flagBuf, sizeof(flagBuf),
    "SPD:%c RPM:%c AFR:%c TMP:%c BAT:%c FUEL:%c",
    d.speedValid?'O':'X', d.rpmValid?'O':'X',
    d.afrValid?'O':'X',   d.tempValid?'O':'X',
    d.battValid?'O':'X',  d.fuelValid?'O':'X');
  spr.setCursor(6, y + 7);
  spr.print(flagBuf);

  // TPS di kanan
  char tBuf[12];
  snprintf(tBuf, sizeof(tBuf), "TPS:%.0f%%", d.tps);
  spr.setCursor(SCR_W - 58, y + 7);
  spr.print(tBuf);
}

// ═══════════════════════════════════════════
//  INIT DISPLAY
// ═══════════════════════════════════════════
void displayInit() {
  tft.init();
  tft.setRotation(1);           // landscape
  tft.fillScreen(COL_BG);
  tft.setTextWrap(false);

  // Sprite double-buffer seluruh layar
  spr.createSprite(SCR_W, SCR_H);
  spr.setTextWrap(false);

  if (DEBUG_MODE) Serial.println("[TFT] Init OK");
}

// ═══════════════════════════════════════════
//  SPLASH SCREEN
// ═══════════════════════════════════════════
void displaySplash(const char* msg, uint16_t col = COL_WHITE) {
  tft.fillScreen(COL_BG);
  tft.setTextColor(COL_AMBER);
  tft.setTextSize(2);
  tft.setCursor(SCR_W / 2 - 90, SCR_H / 2 - 30);
  tft.print("VARIO 125 DASHBOARD");
  tft.setTextColor(col);
  tft.setTextSize(1);
  tft.setCursor(SCR_W / 2 - 80, SCR_H / 2 + 5);
  tft.print(msg);
  tft.setTextColor(COL_DARKGREY);
  tft.setCursor(SCR_W / 2 - 90, SCR_H / 2 + 25);
  tft.print("Honda Keihin K-Line | ESP32 | TFT_eSPI");
}

// ═══════════════════════════════════════════
//  RENDER FRAME PENUH
//  Semua gambar ke sprite dulu → push sekali → tidak flicker
// ═══════════════════════════════════════════
void displayUpdate(const SensorData& d) {
  spr.fillSprite(COL_BG);

  drawStatusBar(d);
  drawSpeedometer(d);
  drawRPMBar(d);
  drawPanelRow1(d);
  drawPanelRow2(d);
  drawStatusBottom(d);

  // Push sprite ke TFT (satu transfer, tidak flicker)
  spr.pushSprite(0, 0);
}
