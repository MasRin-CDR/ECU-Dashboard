#include "EngineHealthAnalyzer.h"

// Constructor for EngineHealthAnalyzer
EngineHealthAnalyzer::EngineHealthAnalyzer() {
    // Initialize default values
    af_stability = 0;
    idle_rpm_stability = 0;
    battery_voltage = 0;
    engine_temperature = 0;
    throttle_consistency = 0;
    dtc_existence = false;
}

// Method to analyze engine health
void EngineHealthAnalyzer::analyzeEngineHealth(float afr, float idleRPM, float batteryVoltage, float engineTemp, float throttlePos, bool dtc) {
    af_stability = calculateAFRStability(afr);
    idle_rpm_stability = calculateIdleRPMStability(idleRPM);
    battery_voltage = batteryVoltage;
    engine_temperature = engineTemp;
    throttle_consistency = calculateThrottleConsistency(throttlePos);
    dtc_existence = dtc;

    health_percentage = calculateHealthPercentage();
}

// Method to calculate AFR stability
float EngineHealthAnalyzer::calculateAFRStability(float afr) {
    // Implement logic to determine AFR stability
    // Placeholder logic for demonstration
    return (afr >= 14.7) ? 100 : (afr < 14.0) ? 50 : 75; // Example values
}

// Method to calculate idle RPM stability
float EngineHealthAnalyzer::calculateIdleRPMStability(float idleRPM) {
    // Implement logic to determine idle RPM stability
    // Placeholder logic for demonstration
    return (idleRPM >= 800 && idleRPM <= 1200) ? 100 : 50; // Example values
}

// Method to calculate throttle consistency
float EngineHealthAnalyzer::calculateThrottleConsistency(float throttlePos) {
    // Implement logic to determine throttle consistency
    // Placeholder logic for demonstration
    return (throttlePos >= 0 && throttlePos <= 100) ? 100 : 0; // Example values
}

// Method to calculate overall engine health percentage
float EngineHealthAnalyzer::calculateHealthPercentage() {
    // Combine all factors to determine overall health percentage
    float total = af_stability + idle_rpm_stability + (battery_voltage > 12.0 ? 100 : 0) + 
                  (engine_temperature < 100 ? 100 : 0) + throttle_consistency;
    return total / 5; // Average of the factors
}

// Method to get health status as a string
String EngineHealthAnalyzer::getHealthStatus() {
    if (health_percentage >= 80) {
        return "Excellent";
    } else if (health_percentage >= 60) {
        return "Good";
    } else if (health_percentage >= 40) {
        return "Warning";
    } else {
        return "Critical";
    }
}

// Method to get health percentage
float EngineHealthAnalyzer::getHealthPercentage() {
    return health_percentage;
}