/**
 * ============================================================
 * KLineProtocol.cpp
 * Implementasi K-Line / ISO9141 / KWP2000 Protocol
 * ============================================================
 */

#include "KLineProtocol.h"

// ─── Constructor ─────────────────────────────────────────────
KLineProtocol::KLineProtocol()
    : _serial(1),           // UART1 pada ESP32-S3
      _state(KLineState::IDLE),
      _protocol(ProtocolType::UNKNOWN),
      _syncByte(0),
      _keyByte1(0),
      _keyByte2(0),
      _lastCommsMs(0),
      _lastReconnectMs(0),
      _errorCount(0),
      _frameSeqNum(0)
{
}

// ─── Begin ────────────────────────────────────────────────────
bool KLineProtocol::begin() {
    Serial.println("[K-Line] Initializing K-Line protocol...");

    // Inisialisasi UART untuk K-Line
    _serial.begin(KLINE_BAUD, SERIAL_8N1, KLINE_RX_PIN, KLINE_TX_PIN);
    _serial.setTimeout(ECU_TIMEOUT_MS);

    // Set TX high (idle state untuk K-Line)
    pinMode(KLINE_TX_PIN, OUTPUT);
    digitalWrite(KLINE_TX_PIN, HIGH);

    delay(300); // Tunggu stabilisasi

    _state = KLineState::INITIALIZING;
    Serial.println("[K-Line] Hardware ready, starting init sequence...");

    return true;
}

// ─── Slow Init (5-baud address) ──────────────────────────────
/**
 * Slow Init: Kirim alamat ECU dengan baud rate 5 baud
 * Setiap bit = 200ms, address = 0x33 (ECU_ADDRESS standar)
 */
bool KLineProtocol::slowInit() {
    Serial.println("[K-Line] Starting Slow Init (5-baud)...");
    _state = KLineState::INITIALIZING;

    // Pastikan line idle selama 300ms
    digitalWrite(KLINE_TX_PIN, HIGH);
    delay(W5); // W5 = 300ms

    // Kirim address byte 0x33 pada 5 baud (200ms per bit)
    sendAddressByte(0x33);

    // Beralih ke baud rate normal untuk komunikasi
    _serial.begin(KLINE_BAUD, SERIAL_8N1, KLINE_RX_PIN, KLINE_TX_PIN);

    // Tunggu sync byte dari ECU (0x55)
    uint8_t syncByte;
    if (!waitForByte(syncByte, 3000)) {
        Serial.println("[K-Line][ERROR] Sync byte timeout");
        _errorCount++;
        _state = KLineState::ERROR;
        return false;
    }

    if (syncByte != 0x55) {
        Serial.printf("[K-Line][ERROR] Bad sync byte: 0x%02X (expected 0x55)\n", syncByte);
        _errorCount++;
        _state = KLineState::ERROR;
        return false;
    }

    Serial.printf("[K-Line] Sync OK: 0x%02X\n", syncByte);
    _syncByte = syncByte;

    // Baca Key Byte 1
    if (!waitForByte(_keyByte1, 500)) {
        Serial.println("[K-Line][ERROR] Key byte 1 timeout");
        _state = KLineState::ERROR;
        return false;
    }
    Serial.printf("[K-Line] Key Byte 1: 0x%02X\n", _keyByte1);

    // Baca Key Byte 2
    if (!waitForByte(_keyByte2, 500)) {
        Serial.println("[K-Line][ERROR] Key byte 2 timeout");
        _state = KLineState::ERROR;
        return false;
    }
    Serial.printf("[K-Line] Key Byte 2: 0x%02X\n", _keyByte2);

    delay(W4); // W4 = 25ms

    // Kirim inversi Key Byte 2 sebagai konfirmasi
    uint8_t invKey2 = ~_keyByte2;
    _serial.write(invKey2);
    _serial.flush();
    Serial.printf("[K-Line] Sent inverted key2: 0x%02X\n", invKey2);

    delay(W4);

    // Baca echo + inversi ECU address
    uint8_t echoAck;
    if (!waitForByte(echoAck, 500)) {
        Serial.println("[K-Line][WARN] No address acknowledge");
        // Tidak fatal, beberapa ECU tidak mengirim ACK
    }

    // Deteksi protocol berdasarkan key bytes
    if (_keyByte1 == 0x8F && _keyByte2 == 0xEA) {
        _protocol = ProtocolType::KWP2000_SLOW;
        Serial.println("[K-Line] Protocol detected: KWP2000 (Slow Init)");
    } else if (_keyByte1 == 0x94 && _keyByte2 == 0x94) {
        _protocol = ProtocolType::ISO9141;
        Serial.println("[K-Line] Protocol detected: ISO9141");
    } else {
        _protocol = ProtocolType::ISO9141; // Default fallback
        Serial.printf("[K-Line] Unknown keys (0x%02X, 0x%02X), defaulting to ISO9141\n",
                      _keyByte1, _keyByte2);
    }

    _state = KLineState::HANDSHAKING;
    _lastCommsMs = millis();
    return true;
}

// ─── Fast Init (pulse) ────────────────────────────────────────
/**
 * Fast Init: Kirim 25ms LOW pulse untuk wake-up ECU
 * Digunakan untuk KWP2000 Fast Init
 */
bool KLineProtocol::fastInit() {
    Serial.println("[K-Line] Starting Fast Init (pulse)...");
    _state = KLineState::INITIALIZING;

    // Pastikan baud rate sudah di-set
    _serial.begin(KLINE_BAUD, SERIAL_8N1, KLINE_RX_PIN, KLINE_TX_PIN);

    // Kirim wake-up pattern: 25ms LOW kemudian 25ms HIGH
    digitalWrite(KLINE_TX_PIN, LOW);
    delay(FAST_INIT_PULSE);
    digitalWrite(KLINE_TX_PIN, HIGH);
    delay(FAST_INIT_PULSE);

    // Kirim Start Communication request
    uint8_t startComm[] = {0x81, 0x10, 0xF1, 0x81, 0x03};
    startComm[4] = calculateChecksum(startComm, 4);

    Serial.print("[K-Line] Sending StartCommunication: ");
    for (int i = 0; i < 5; i++) Serial.printf("0x%02X ", startComm[i]);
    Serial.println();

    for (int i = 0; i < 5; i++) {
        _serial.write(startComm[i]);
        delay(1);
    }
    _serial.flush();

    // Tunggu response
    ECUFrame response;
    if (!receiveResponse(response, 1000)) {
        Serial.println("[K-Line][ERROR] Fast init no response");
        _state = KLineState::ERROR;
        return false;
    }

    if (response.valid) {
        _protocol = ProtocolType::KWP2000_FAST;
        Serial.println("[K-Line] Fast Init OK - KWP2000 Fast detected");
        _state = KLineState::HANDSHAKING;
        _lastCommsMs = millis();
        return true;
    }

    _state = KLineState::ERROR;
    return false;
}

// ─── ECU Handshake ────────────────────────────────────────────
bool KLineProtocol::handshake() {
    Serial.println("[K-Line] Starting ECU handshake...");

    // Coba Fast Init terlebih dahulu
    if (fastInit()) {
        Serial.println("[K-Line] Handshake via Fast Init: SUCCESS");
        _state = KLineState::CONNECTED;
        return true;
    }

    Serial.println("[K-Line] Fast Init failed, trying Slow Init...");
    delay(1000);

    // Fallback ke Slow Init
    if (slowInit()) {
        Serial.println("[K-Line] Handshake via Slow Init: SUCCESS");
        _state = KLineState::CONNECTED;
        return true;
    }

    Serial.println("[K-Line][ERROR] All handshake methods failed");
    _state = KLineState::ERROR;
    return false;
}

// ─── Send PID Request ─────────────────────────────────────────
bool KLineProtocol::sendRequest(uint8_t serviceID, uint8_t pid) {
    if (_state != KLineState::CONNECTED) {
        Serial.println("[K-Line][WARN] Not connected, cannot send request");
        return false;
    }

    uint8_t frame[8];
    uint8_t len = 0;
    buildFrame(frame, len, serviceID, pid);

    Serial.printf("[K-Line][TX] Service:0x%02X PID:0x%02X -> ", serviceID, pid);
    for (int i = 0; i < len; i++) Serial.printf("0x%02X ", frame[i]);
    Serial.println();

    for (int i = 0; i < len; i++) {
        _serial.write(frame[i]);
        delay(1); // Inter-byte delay
    }
    _serial.flush();

    _state = KLineState::WAITING_RESPONSE;
    return true;
}

// ─── Send Raw Frame ───────────────────────────────────────────
bool KLineProtocol::sendRawFrame(const uint8_t* data, uint8_t len) {
    for (int i = 0; i < len; i++) {
        _serial.write(data[i]);
        delayMicroseconds(500);
    }
    _serial.flush();
    return true;
}

// ─── Receive Response ─────────────────────────────────────────
bool KLineProtocol::receiveResponse(ECUFrame& frame, uint32_t timeoutMs) {
    frame.valid   = false;
    frame.dataLen = 0;

    uint8_t buf[MAX_FRAME_SIZE + 8];
    uint8_t idx = 0;

    uint32_t start = millis();

    // Baca minimal 4 byte: header, target, source, length
    while (idx < 4 && (millis() - start) < timeoutMs) {
        if (_serial.available()) {
            buf[idx++] = _serial.read();
        }
        yield(); // Beri waktu untuk RTOS task
    }

    if (idx < 4) {
        Serial.println("[K-Line][ERROR] Frame header timeout");
        _errorCount++;
        return false;
    }

    // Parse header
    frame.header = buf[0];
    frame.target = buf[1];
    frame.source = buf[2];
    frame.length = buf[3];

    if (frame.length == 0 || frame.length > MAX_FRAME_SIZE) {
        Serial.printf("[K-Line][ERROR] Invalid frame length: %d\n", frame.length);
        _errorCount++;
        return false;
    }

    // Baca data bytes
    uint32_t dataStart = millis();
    uint8_t  dataIdx   = 0;
    while (dataIdx < frame.length && (millis() - dataStart) < 500) {
        if (_serial.available()) {
            frame.data[dataIdx++] = _serial.read();
        }
        yield();
    }
    frame.dataLen = dataIdx;

    if (dataIdx < frame.length) {
        Serial.printf("[K-Line][ERROR] Data incomplete: got %d/%d\n", dataIdx, frame.length);
        _errorCount++;
        return false;
    }

    // Baca checksum
    uint32_t csStart = millis();
    while (!_serial.available() && (millis() - csStart) < 200) yield();

    if (!_serial.available()) {
        Serial.println("[K-Line][ERROR] Checksum timeout");
        _errorCount++;
        return false;
    }
    frame.checksum = _serial.read();

    // Validasi checksum
    frame.valid = validateChecksum(frame);
    if (!frame.valid) {
        Serial.printf("[K-Line][ERROR] Checksum mismatch: got 0x%02X\n", frame.checksum);
        _errorCount++;
        return false;
    }

    // Print debug frame
    printFrame(frame, false);

    _lastCommsMs = millis();
    _state = KLineState::CONNECTED;

    // Reset serial buffer setelah echo bytes (ISO9141 echo)
    flushSerial();

    return true;
}

// ─── Calculate Checksum ───────────────────────────────────────
uint8_t KLineProtocol::calculateChecksum(const uint8_t* data, uint8_t len) {
    uint16_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return (uint8_t)(sum & 0xFF);
}

// ─── Validate Checksum ────────────────────────────────────────
bool KLineProtocol::validateChecksum(const ECUFrame& frame) {
    uint16_t sum = frame.header + frame.target + frame.source + frame.length;
    for (int i = 0; i < frame.dataLen; i++) sum += frame.data[i];
    return ((sum & 0xFF) == frame.checksum);
}

// ─── Reconnect ────────────────────────────────────────────────
bool KLineProtocol::reconnect() {
    uint32_t now = millis();
    if (now - _lastReconnectMs < RECONNECT_INTERVAL_MS) return false;

    _lastReconnectMs = now;
    Serial.println("[K-Line] Attempting reconnect...");

    resetState();
    delay(500);
    return handshake();
}

// ─── Reset State ─────────────────────────────────────────────
void KLineProtocol::resetState() {
    _state = KLineState::IDLE;
    _protocol = ProtocolType::UNKNOWN;
    _syncByte  = 0;
    _keyByte1  = 0;
    _keyByte2  = 0;
    flushSerial();
    Serial.println("[K-Line] State reset");
}

// ─── Print Frame (Debug) ─────────────────────────────────────
void KLineProtocol::printFrame(const ECUFrame& frame, bool isRequest) {
    Serial.printf("[K-Line][%s] Header:0x%02X Tgt:0x%02X Src:0x%02X Len:%d Data:",
                  isRequest ? "TX" : "RX",
                  frame.header, frame.target, frame.source, frame.length);
    for (int i = 0; i < frame.dataLen; i++) Serial.printf("0x%02X ", frame.data[i]);
    Serial.printf("CS:0x%02X [%s]\n", frame.checksum, frame.valid ? "OK" : "FAIL");
}

// ─── Get State String ─────────────────────────────────────────
String KLineProtocol::getStateString() const {
    switch (_state) {
        case KLineState::IDLE:             return "IDLE";
        case KLineState::INITIALIZING:     return "INITIALIZING";
        case KLineState::HANDSHAKING:      return "HANDSHAKING";
        case KLineState::CONNECTED:        return "CONNECTED";
        case KLineState::REQUESTING:       return "REQUESTING";
        case KLineState::WAITING_RESPONSE: return "WAITING_RESPONSE";
        case KLineState::ERROR:            return "ERROR";
        case KLineState::RECONNECTING:     return "RECONNECTING";
        default:                           return "UNKNOWN";
    }
}

// ─── Get Protocol String ──────────────────────────────────────
String KLineProtocol::getProtocolString() const {
    switch (_protocol) {
        case ProtocolType::ISO9141:       return "ISO9141";
        case ProtocolType::KWP2000_SLOW:  return "KWP2000-Slow";
        case ProtocolType::KWP2000_FAST:  return "KWP2000-Fast";
        default:                          return "UNKNOWN";
    }
}

// ─── Private: Send 5-baud address ────────────────────────────
void KLineProtocol::sendAddressByte(uint8_t addr) {
    // Kirim start bit (LOW)
    setBit(false);
    // Kirim 8 data bits (LSB first)
    for (int i = 0; i < 8; i++) {
        setBit((addr >> i) & 0x01);
    }
    // Kirim stop bit (HIGH)
    setBit(true);
}

// ─── Private: Set K-Line bit ─────────────────────────────────
void KLineProtocol::setBit(bool high) {
    digitalWrite(KLINE_TX_PIN, high ? HIGH : LOW);
    delay(SLOW_INIT_ADDR_TIME); // 200ms per bit = 5 baud
}

// ─── Private: Wait for byte ───────────────────────────────────
bool KLineProtocol::waitForByte(uint8_t& byte, uint32_t timeoutMs) {
    uint32_t start = millis();
    while (!_serial.available()) {
        if (millis() - start > timeoutMs) return false;
        yield();
    }
    byte = _serial.read();
    return true;
}

// ─── Private: Flush serial ────────────────────────────────────
void KLineProtocol::flushSerial() {
    while (_serial.available()) _serial.read();
}

// ─── Private: Build Frame ─────────────────────────────────────
void KLineProtocol::buildFrame(uint8_t* buf, uint8_t& len,
                                uint8_t service, uint8_t pid) {
    len = 0;
    if (_protocol == ProtocolType::KWP2000_FAST ||
        _protocol == ProtocolType::KWP2000_SLOW) {
        // KWP2000 frame format
        buf[len++] = 0xC2;           // Header (physical addressing, 2 data bytes)
        buf[len++] = ECU_ADDRESS;    // Target
        buf[len++] = TESTER_ADDRESS; // Source
        buf[len++] = service;        // Service ID
        buf[len++] = pid;            // PID
        buf[len++] = calculateChecksum(buf, len);
    } else {
        // ISO9141 frame format
        buf[len++] = ISO9141_HEADER; // 0x68
        buf[len++] = ECU_ADDRESS;    // 0x10
        buf[len++] = TESTER_ADDRESS; // 0xF1
        buf[len++] = service;        // 0x01 = Mode 1 (current data)
        buf[len++] = pid;
        buf[len++] = calculateChecksum(buf, len);
    }
}
