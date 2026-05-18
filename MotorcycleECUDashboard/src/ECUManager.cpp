#include "ECUManager.h"
#include "KLineProtocol.h"
#include "Arduino.h"

// Constructor for ECUManager
ECUManager::ECUManager(KLineProtocol* kLineProtocol) {
    this->kLineProtocol = kLineProtocol;
}

// Method to initialize ECU communication
void ECUManager::begin() {
    kLineProtocol->slowInit();
    delay(100);
    kLineProtocol->fastInit();
    delay(100);
}

// Method to read RPM from ECU
int ECUManager::readRPM() {
    uint8_t response[8];
    if (kLineProtocol->requestPID(0x0C, response)) { // PID 0x0C for RPM
        int A = response[3]; // High byte
        int B = response[4]; // Low byte
        return ((A * 256) + B) / 4; // Calculate RPM
    }
    return -1; // Error reading RPM
}

// Method to read vehicle speed from ECU
int ECUManager::readSpeed() {
    uint8_t response[8];
    if (kLineProtocol->requestPID(0x0D, response)) { // PID 0x0D for Speed
        return response[3]; // Speed is in response[3]
    }
    return -1; // Error reading speed
}

// Method to read battery voltage from ECU
float ECUManager::readBatteryVoltage() {
    uint8_t response[8];
    if (kLineProtocol->requestPID(0x42, response)) { // PID 0x42 for Battery Voltage
        return response[3] * 0.1; // Convert to volts
    }
    return -1; // Error reading battery voltage
}

// Method to read AFR from ECU
float ECUManager::readAFR() {
    // Implement AFR reading logic based on O2 sensor, MAP, TPS, and RPM
    // Placeholder for actual implementation
    return 14.7; // Return a dummy value for now
}

// Method to read average fuel consumption
float ECUManager::readAverageFuelConsumption() {
    // Implement fuel consumption calculation logic
    // Placeholder for actual implementation
    return 20.0; // Return a dummy value for now
}

// Method to analyze engine health
String ECUManager::analyzeEngineHealth() {
    // Implement engine health analysis logic
    // Placeholder for actual implementation
    return "Excellent"; // Return a dummy status for now
}

// Method to update ECU data
void ECUManager::update() {
    rpm = readRPM();
    speed = readSpeed();
    batteryVoltage = readBatteryVoltage();
    afr = readAFR();
    averageFuelConsumption = readAverageFuelConsumption();
    engineHealthStatus = analyzeEngineHealth();
}

// Getters for the data
int ECUManager::getRPM() {
    return rpm;
}

int ECUManager::getSpeed() {
    return speed;
}

float ECUManager::getBatteryVoltage() {
    return batteryVoltage;
}

float ECUManager::getAFR() {
    return afr;
}

float ECUManager::getAverageFuelConsumption() {
    return averageFuelConsumption;
}

String ECUManager::getEngineHealthStatus() {
    return engineHealthStatus;
}