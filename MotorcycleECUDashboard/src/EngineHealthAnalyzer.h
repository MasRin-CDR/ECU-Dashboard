#ifndef ENGINEHEALTHANALYZER_H
#define ENGINEHEALTHANALYZER_H

#include <Arduino.h>

class EngineHealthAnalyzer {
public:
    EngineHealthAnalyzer();

    // Method to analyze engine health based on various parameters
    void analyzeHealth(float afrStability, float idleRPMStability, float batteryVoltage, 
                       float engineTemperature, float throttleConsistency, bool dtcExistence);

    // Method to get the health status as a string
    String getHealthStatus() const;

    // Method to get the health percentage
    float getHealthPercentage() const;

private:
    // Health status values
    enum HealthStatus {
        EXCELLENT,
        GOOD,
        WARNING,
        CRITICAL
    };

    HealthStatus currentStatus;
    float healthPercentage;

    // Method to calculate health percentage based on input parameters
    void calculateHealthPercentage(float afrStability, float idleRPMStability, 
                                    float batteryVoltage, float engineTemperature, 
                                    float throttleConsistency, bool dtcExistence);
};

#endif // ENGINEHEALTHANALYZER_H