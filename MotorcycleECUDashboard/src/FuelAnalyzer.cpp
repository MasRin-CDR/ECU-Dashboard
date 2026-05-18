#include "FuelAnalyzer.h"

// Constructor for FuelAnalyzer
FuelAnalyzer::FuelAnalyzer() {
    // Initialize variables
    averageConsumption = 0.0;
    realtimeConsumption = 0.0;
    totalDistance = 0.0;
    totalFuelUsed = 0.0;
}

// Method to update fuel consumption based on parameters
void FuelAnalyzer::updateConsumption(float rpm, float throttlePosition, float speed, float injectorPulse) {
    // Calculate fuel consumption based on RPM, throttle position, speed, and injector pulse
    // This is a simplified estimation formula
    realtimeConsumption = (rpm * throttlePosition * injectorPulse) / (speed + 1); // Avoid division by zero
    totalDistance += speed / 3600.0; // Convert speed from km/h to km/s
    totalFuelUsed += realtimeConsumption / 3600.0; // Convert consumption to liters per second

    // Calculate average consumption in km/l
    if (totalDistance > 0) {
        averageConsumption = totalDistance / totalFuelUsed;
    } else {
        averageConsumption = 0.0; // Avoid division by zero
    }
}

// Method to get the average fuel consumption
float FuelAnalyzer::getAverageConsumption() {
    return averageConsumption;
}

// Method to get the real-time fuel consumption
float FuelAnalyzer::getRealtimeConsumption() {
    return realtimeConsumption;
}