#include "AFRAnalyzer.h"

// Constructor for AFRAnalyzer
AFRAnalyzer::AFRAnalyzer() {
    // Initialize variables
    this->o2SensorValue = 0;
    this->mapValue = 0;
    this->tpsValue = 0;
    this->rpmValue = 0;
    this->afrStatus = "Normal";
}

// Method to update sensor values
void AFRAnalyzer::updateSensorValues(float o2, float map, float tps, float rpm) {
    this->o2SensorValue = o2;
    this->mapValue = map;
    this->tpsValue = tps;
    this->rpmValue = rpm;
    calculateAFR();
}

// Method to calculate AFR based on sensor data
void AFRAnalyzer::calculateAFR() {
    // Example calculation for AFR (this should be replaced with actual logic)
    float afr = (this->o2SensorValue / (this->mapValue + 1)) * 14.7; // Simplified formula

    // Determine AFR status
    if (afr < 14.0) {
        this->afrStatus = "Lean";
    } else if (afr >= 14.0 && afr <= 15.0) {
        this->afrStatus = "Normal";
    } else {
        this->afrStatus = "Rich";
    }
}

// Method to get the current AFR status
String AFRAnalyzer::getAFRStatus() {
    return this->afrStatus;
}

// Method to get the current AFR value (for debugging or display purposes)
float AFRAnalyzer::getCurrentAFR() {
    return (this->o2SensorValue / (this->mapValue + 1)) * 14.7; // Simplified formula
}