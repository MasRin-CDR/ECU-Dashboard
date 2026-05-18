#include <Arduino.h>
#include "KLineProtocol.h"
#include "ECUManager.h"
#include "DashboardUI.h"
#include "config.h"

// Create instances of the classes
KLineProtocol kLineProtocol;
ECUManager ecuManager;
DashboardUI dashboardUI;

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    
    // Initialize K-Line communication
    kLineProtocol.begin();
    
    // Initialize the ECU Manager
    ecuManager.begin();
    
    // Initialize the Dashboard UI
    dashboardUI.begin();
}

void loop() {
    // Update K-Line communication
    kLineProtocol.update();
    
    // Read data from ECU
    ecuManager.update();
    
    // Update the Dashboard UI with real-time data
    dashboardUI.update(ecuManager.getRPM(), ecuManager.getSpeed(), ecuManager.getBatteryVoltage(), 
                        ecuManager.getAFR(), ecuManager.getAverageFuelConsumption(), 
                        ecuManager.getEngineHealthStatus());
    
    // Add a small delay for stability
    delay(100);
}