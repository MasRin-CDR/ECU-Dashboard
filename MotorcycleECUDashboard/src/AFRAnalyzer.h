#ifndef AFRANALYZER_H
#define AFRANALYZER_H

#include <Arduino.h>

class AFRAnalyzer {
public:
    // Constructor
    AFRAnalyzer();

    // Method to calculate AFR based on sensor data
    float calculateAFR(float o2SensorValue, float mapValue, float tpsValue, float rpmValue);

    // Method to determine AFR status
    String getAFRStatus(float afrValue);

    // Method to update the AFR values
    void updateAFR(float o2SensorValue, float mapValue, float tpsValue, float rpmValue);

    // Getters for the calculated values
    float getCurrentAFR() const;
    String getCurrentStatus() const;

private:
    float currentAFR; // Current Air-Fuel Ratio
    String currentStatus; // Current status of the AFR (Lean, Normal, Rich)

    // Constants for AFR thresholds
    const float leanThreshold = 14.7; // Example threshold for lean
    const float richThreshold = 12.5; // Example threshold for rich
};

#endif // AFRANALYZER_H