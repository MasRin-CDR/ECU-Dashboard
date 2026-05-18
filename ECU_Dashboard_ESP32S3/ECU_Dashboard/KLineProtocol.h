#pragma once
/**
 * ============================================================
 * KLineProtocol.h
 * K-Line / ISO9141 / KWP2000 Protocol Handler
 * ============================================================
 * Menangani komunikasi low-level dengan ECU melalui L9637D
 * Mendukung: Slow Init, Fast Init, KWP2000, ISO9141
 * ============================================================
 */

#include <Arduino.h>
#include <HardwareSerial.h>

// ─── Pin Definitions ────────────────────────────────────────
#define KLINE_RX_PIN    16
#define KLINE_TX_PIN    17
#define KLINE_BAUD      10400   // Standard K-Line baud rate

// ─── Protocol Constants ──────────────────────────────────────
#define ISO9141_HEADER  0x68
#define KWP2000_HEADER  0x80
#define ECU_ADDRESS     0x10    // Target ECU address
#define TESTER_ADDRESS  0xF1    // Tester (us) address

// ─── Timing Constants (ms) ──────────────────────────────────
#define SLOW_INIT_ADDR_TIME     200   // 200ms per bit for 5-baud
#define FAST_INIT_PULSE         25    // 25ms low pulse
#define P1_MIN                  0
#define P1_MAX                  20
#define P2_MIN                  25
#define P2_MAX                  50
#define P3_MIN                  55
#define P3_MAX                  5000
#define P4_MIN                  5
#define P4_MAX                  20
#define W1                      2    // 2ms - 300ms
#define W2                      25   // ms
#define W3                      0    // 0ms
#define W4                      25   // ms
#define W5                      300  // ms

#define ECU_TIMEOUT_MS          500  // Timeout per response
#define RECONNECT_INTERVAL_MS   3000 // Interval reconnect

// ─── Frame Buffer ────────────────────────────────────────────
#define MAX_FRAME_SIZE  32

// ─── Protocol States ─────────────────────────────────────────
enum class KLineState {
    IDLE,
    INITIALIZING,
    HANDSHAKING,
    CONNECTED,
    REQUESTING,
    WAITING_RESPONSE,
    ERROR,
    RECONNECTING
};

// ─── Protocol Type ───────────────────────────────────────────
enum class ProtocolType {
    UNKNOWN,
    ISO9141,
    KWP2000_SLOW,
    KWP2000_FAST
};

// ─── ECU Frame Structure ─────────────────────────────────────
struct ECUFrame {
    uint8_t header;
    uint8_t target;
    uint8_t source;
    uint8_t length;
    uint8_t data[MAX_FRAME_SIZE];
    uint8_t checksum;
    uint8_t dataLen;
    bool    valid;
};

// ─── KLineProtocol Class ─────────────────────────────────────
class KLineProtocol {
public:
    KLineProtocol();

    // Inisialisasi & koneksi
    bool begin();
    bool slowInit();
    bool fastInit();
    bool handshake();

    // Komunikasi
    bool sendRequest(uint8_t serviceID, uint8_t pid);
    bool sendRawFrame(const uint8_t* data, uint8_t len);
    bool receiveResponse(ECUFrame& frame, uint32_t timeoutMs = ECU_TIMEOUT_MS);

    // Checksum
    uint8_t calculateChecksum(const uint8_t* data, uint8_t len);
    bool    validateChecksum(const ECUFrame& frame);

    // Status
    KLineState getState()        const { return _state; }
    ProtocolType getProtocol()   const { return _protocol; }
    bool isConnected()           const { return _state == KLineState::CONNECTED; }
    uint32_t getLastCommsMs()    const { return _lastCommsMs; }
    uint32_t getErrorCount()     const { return _errorCount; }

    // Reconnect
    bool reconnect();
    void resetState();

    // Debug
    void printFrame(const ECUFrame& frame, bool isRequest = true);
    String getStateString() const;
    String getProtocolString() const;

private:
    HardwareSerial _serial;
    KLineState     _state;
    ProtocolType   _protocol;

    uint8_t  _syncByte;         // Sync byte dari ECU
    uint8_t  _keyByte1;         // Key byte 1
    uint8_t  _keyByte2;         // Key byte 2
    uint32_t _lastCommsMs;      // Timestamp komunikasi terakhir
    uint32_t _lastReconnectMs;  // Timestamp reconnect terakhir
    uint32_t _errorCount;       // Counter error
    uint8_t  _frameSeqNum;      // Frame sequence number

    // Low-level K-Line functions
    void     setBit(bool high);
    void     sendAddressByte(uint8_t addr);
    bool     waitForByte(uint8_t& byte, uint32_t timeoutMs);
    bool     waitForSyncPattern();
    void     flushSerial();

    // Frame builder
    void buildFrame(uint8_t* buf, uint8_t& len,
                    uint8_t service, uint8_t pid);
};
