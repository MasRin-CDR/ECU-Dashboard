#ifndef DASHBOARDUI_H
#define DASHBOARDUI_H

#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include "config.h"

class DashboardUI {
public:
    DashboardUI();
    void begin();
    void update(float rpm, float speed, float batteryVoltage, float afr, float avgFuelConsumption, float engineHealth);
    void drawGauge(float value, const char* label, int x, int y, int width, int height);
    void drawText(const char* text, int x, int y, int size);
    void showWarning(const char* message);
    void clearScreen();

private:
    TFT_eSPI tft; // TFT display object
    XPT2046_Touchscreen ts; // Touchscreen object
    void drawRPMGauge(float rpm);
    void drawSpeed(float speed);
    void drawAFR(float afr);
    void drawBatteryVoltage(float voltage);
    void drawAvgFuelConsumption(float avgConsumption);
    void drawEngineHealth(float health);
    void drawStatus(const char* status);
};

#endif // DASHBOARDUI_H