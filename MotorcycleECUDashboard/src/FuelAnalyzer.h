#ifndef FUELANALYZER_H
#define FUELANALYZER_H

#include <Arduino.h>

class FuelAnalyzer {
public:
    FuelAnalyzer();
    
    void update(float rpm, float throttlePosition, float speed);
    float getAverageConsumption() const;
    float getRealtimeConsumption() const;

private:
    float averageConsumption; // in km/l
    float realtimeConsumption; // in km/l
    float calculateConsumption(float rpm, float throttlePosition, float speed);
};

#endif // FUELANALYZER_H