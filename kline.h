#pragma once
// ╔══════════════════════════════════════════════════════════════╗
// ║          VARIO DASHBOARD — kline.h                          ║
// ║  Komunikasi K-Line ECU Honda Keihin                         ║
// ╚══════════════════════════════════════════════════════════════╝

#include <Arduino.h>
#include <HardwareSerial.h>
#include "config.h"
#include "sensors.h"

HardwareSerial ecuSerial(2);   // UART2

// ── State K-Line ──
static bool    _ecuReady    = false;
static uint8_t _ecuRetry    = 0;
static uint32_t _ecuLastMs  = 0;

// ─────────────────────────────────────────
//  Checksum Honda Keihin
//  CS = 0x100 - (sum semua byte & 0xFF)
// ─────────────────────────────────────────
static uint8_t keihinCS(uint8_t* buf, uint8_t len) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < len; i++) sum += buf[i];
  return (uint8_t)(0x100 - (sum & 0xFF));
}

// ─────────────────────────────────────────
//  Wakeup K-Line (bit-bang 5baud style)
// ─────────────────────────────────────────
static void klineWakeup() {
  ecuSerial.end();
  pinMode(PIN_KLINE_TX, OUTPUT);
  digitalWrite(PIN_KLINE_TX, HIGH);
  delay(300);

  // Pull LOW 70ms
  digitalWrite(PIN_KLINE_TX, LOW);
  delay(70);

  // HIGH 130ms
  digitalWrite(PIN_KLINE_TX, HIGH);
  delay(130);

  // Aktifkan UART
  ecuSerial.begin(KLINE_BAUD, SERIAL_8N1, PIN_KLINE_RX, PIN_KLINE_TX);
  delay(20);
  while (ecuSerial.available()) ecuSerial.read();  // buang noise
}

// ─────────────────────────────────────────
//  Kirim ke ECU (half-duplex → buang echo)
// ─────────────────────────────────────────
static void klineSend(uint8_t* buf, uint8_t len) {
  ecuSerial.write(buf, len);
  ecuSerial.flush();

  uint32_t t    = millis();
  uint8_t  echo = 0;
  while (echo < len && millis() - t < 80) {
    if (ecuSerial.available()) { ecuSerial.read(); echo++; }
  }
}

// ─────────────────────────────────────────
//  Baca respon ECU
// ─────────────────────────────────────────
static uint8_t klineRead(uint8_t* buf, uint8_t maxLen, uint16_t tms = 150) {
  uint8_t  idx = 0;
  uint32_t t   = millis();
  while (idx < maxLen) {
    if (millis() - t > tms) break;
    if (ecuSerial.available()) {
      buf[idx++] = ecuSerial.read();
      t = millis();
    }
  }
  return idx;
}

// ─────────────────────────────────────────
//  Init Handshake Honda Keihin
// ─────────────────────────────────────────
bool klineInit() {
  uint8_t buf[10];
  uint8_t n;

  klineWakeup();

  // Step 1: Wake Up  FE 04 72 8C → 0E 04 72 7C
  uint8_t wake[] = {0xFE, 0x04, 0x72, 0x8C};
  klineSend(wake, 4);
  n = klineRead(buf, 4, 200);
  if (n < 4 || buf[0] != 0x0E) {
    if (DEBUG_MODE) Serial.printf("[KLINE] Wakeup fail n=%d\n", n);
    return false;
  }
  delay(30);

  // Step 2: Init  72 05 00 F0 99 → 02 04 00 FA
  uint8_t initReq[] = {0x72, 0x05, 0x00, 0xF0, 0x99};
  klineSend(initReq, 5);
  n = klineRead(buf, 4, 200);
  if (n < 4 || buf[0] != 0x02) {
    if (DEBUG_MODE) Serial.printf("[KLINE] Init fail n=%d\n", n);
    return false;
  }

  if (DEBUG_MODE) Serial.println("[KLINE] ECU Terhubung!");
  return true;
}

// ─────────────────────────────────────────
//  Request tabel data
// ─────────────────────────────────────────
static bool klineTable(uint8_t tbl, uint8_t* resp, uint8_t& len) {
  uint8_t req[5];
  req[0] = 0x72;
  req[1] = 0x05;
  req[2] = 0x71;
  req[3] = tbl;
  req[4] = keihinCS(req, 4);

  klineSend(req, 5);
  len = klineRead(resp, 40, 120);
  return (len >= 5 && resp[0] == 0x02);
}

// ─────────────────────────────────────────
//  Parse tabel 0x11 (sensor utama Keihin)
//
//  Offset data Vario 125 KZR:
//  [5-6] RPM      : raw/4 = RPM aktual
//  [7]   TPS      : 0-255 → 0-100%
//  [9]   ECT      : raw×160/255-40 = °C
//  [11]  IAT      : raw×160/255-40 = °C
//  [13-14] MAP    : uint16 / 655.35 = kPa
//  [15]  Lambda   : 0-255 → λ 0.5-1.5 → ×14.7=AFR
//  [16]  Speed    : km/h langsung
//  [20]  Volt raw : ×20/255 = V
// ─────────────────────────────────────────
static void klineParse11(uint8_t* d, uint8_t len, SensorData& out) {
  if (len < 18) return;

  // RPM
  int16_t rawRpm = (int16_t)((d[5] << 8) | d[6]);
  uint16_t rpm   = (uint16_t)abs(rawRpm);
  if (rpm > 15000) rpm /= 4;
  out.rpm      = constrain(rpm, 0, 14000);
  out.rpmValid = true;

  // TPS
  out.tps = d[7] * 100.0f / 255.0f;

  // Suhu mesin (ECT)
  out.engineTempC = d[9] * 160.0f / 255.0f - 40.0f;
  out.tempValid   = true;

  // MAP
  uint16_t rawMap = (d[13] << 8) | d[14];
  out.mapKpa      = rawMap / 655.35f;

  // AFR via Lambda
  float lam   = 0.5f + (d[15] / 255.0f) * 1.0f;
  out.afr     = constrain(lam * 14.7f, 9.0f, 22.0f);
  out.afrValid = true;

  // Speed
  out.speedKmh  = d[16];
  out.speedValid = true;

  // Voltase
  if (len > 20) {
    out.battVolt  = d[20] * 20.0f / 255.0f;
    out.battValid = true;
  }

  out.ecuOnline = true;
  _ecuLastMs    = millis();
}

// ─────────────────────────────────────────
//  Update K-Line (dipanggil dari loop)
// ─────────────────────────────────────────
void klineUpdate(SensorData& d) {
  uint32_t now = millis();

  // Deteksi timeout ECU
  if (_ecuReady && (now - _ecuLastMs > 2000)) {
    if (DEBUG_MODE) Serial.println("[KLINE] Timeout, reconnect...");
    _ecuReady  = false;
    d.ecuOnline = false;
  }

  // Re-init jika perlu
  if (!_ecuReady) {
    if (now - _ecuLastMs < 1000) return;  // jangan terlalu sering retry
    _ecuLastMs = now;
    _ecuReady  = klineInit();
    return;
  }

  // Request tabel 0x11
  uint8_t resp[40];
  uint8_t len = 0;
  if (klineTable(0x11, resp, len)) {
    klineParse11(resp, len, d);
  } else {
    // Fallback tabel 0x17
    delay(20);
    if (klineTable(0x17, resp, len)) {
      klineParse11(resp, len, d);
    } else {
      _ecuLastMs = 0;  // paksa re-init segera
    }
  }
}

// ─────────────────────────────────────────
//  Init fungsi (dipanggil dari setup)
// ─────────────────────────────────────────
void klineBegin() {
  _ecuReady = klineInit();
  if (!_ecuReady && DEBUG_MODE) {
    Serial.println("[KLINE] ECU tidak terdeteksi, akan retry otomatis");
  }
}
