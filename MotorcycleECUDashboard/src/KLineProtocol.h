#ifndef KLINEPROTOCOL_H
#define KLINEPROTOCOL_H

#include <Arduino.h>
#include <HardwareSerial.h>

class KLineProtocol {
public:
    KLineProtocol(HardwareSerial &serial);
    
    void begin();
    void slowInit();
    void fastInit();
    bool ecuHandshake();
    void requestPID(uint8_t pid);
    bool validateChecksum(uint8_t *data, size_t length);
    void handleTimeout();
    void reconnect();

private:
    HardwareSerial &serial;
    uint8_t calculateChecksum(uint8_t *data, size_t length);
    void sendRequest(uint8_t *data, size_t length);
    void readResponse(uint8_t *buffer, size_t length);
};

#endif // KLINEPROTOCOL_H