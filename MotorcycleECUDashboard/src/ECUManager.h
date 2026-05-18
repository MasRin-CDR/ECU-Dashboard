#ifndef ECU_MANAGER_H
#define ECU_MANAGER_H

#include <Arduino.h>
#include "KLineProtocol.h"

class ECUManager {
public:
    ECUManager(KLineProtocol* kLineProtocol);
    void begin();
    void update();
    float getRPM();
    float getSpeed();
    float getBatteryVoltage();
    float getAFR();
    float getAverageFuelConsumption();
    String getEngineHealthStatus();

private:
    KLineProtocol* kLineProtocol;
    float rpm;
    float speed;
    float batteryVoltage;
    float afr;
    float averageFuelConsumption;
    String engineHealthStatus;

    void readECUData();
    void processECUData();
    void calculateEngineHealth();
};

#endif // ECU_MANAGER_H