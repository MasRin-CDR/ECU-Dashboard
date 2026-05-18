/**
 * ============================================================
 * ECU_Dashboard.ino
 * Motorcycle ECU Dashboard - ESP32-S3
 * ============================================================
 *
 * Sistem dashboard ECU motor realtime berbasis ESP32-S3
 * dengan komunikasi K-Line (ISO9141 / KWP2000) dan TFT 5"
 *
 * Hardware:
 *   - ESP32-S3
 *   - TFT 5" ILI9488
 *   - Touchscreen XPT2046
 *   - ECU Interface L9637D
 *
 * Features:
 *   - RPM gauge dengan shift indicator
 *   - Speed digital
 *   - AFR realtime + status
 *   - Battery voltage
 *   - Average fuel consumption
 *   - Engine health analyzer (0-100%)
 *   - Warning popup system
 *   - K-Line auto reconnect
 *   - Watchdog timer
 *
 * ============================================================
 * Pin Configuration:
 *   K-Line RX: GPIO16
 *   K-Line TX: GPIO17
 *   TFT MOSI:  GPIO23
 *   TFT MISO:  GPIO19
 *   TFT SCK:   GPIO18
 *   TFT CS:    GPIO5
 *   TFT DC:    GPIO2
 *   TFT RST:   GPIO4
 *   Touch CS:  GPIO15
 *   Touch IRQ: GPIO27
 *   Batt ADC:  GPIO34
 * ============================================================
 *
 * Penulis: ECU Dashboard Project
 * Platform: ESP32-S3 / Arduino Framework
 * ============================================================
 */

#include <Arduino.h>
#include <esp_task_wdt.h>   // ESP32 Watchdog
#include <SPI.h>
#include <Wire.h>

#include "KLineProtocol.h"
#include "ECUManager.h"
#include "Analyzers.h"
#include "DashboardUI.h"

// ─── Watchdog Configuration ───────────────────────────────────
#define WDT_TIMEOUT_SEC     10      // Reset jika tidak ada respons 10 detik

// ─── Task Configuration ───────────────────────────────────────
// ESP32-S3 memiliki 2 core, gunakan untuk pemisahan tugas
#define CORE_ECU        1           // Core untuk komunikasi ECU
#define CORE_DISPLAY    0           // Core untuk display update
#define STACK_ECU       8192        // Stack size untuk ECU task
#define STACK_DISPLAY   16384       // Stack size untuk display task
#define PRIO_ECU        2           // Prioritas ECU task (lebih tinggi)
#define PRIO_DISPLAY    1           // Prioritas display task

// ─── System Instances ─────────────────────────────────────────
ECUManager          ecuManager;
AFRAnalyzer         afrAnalyzer;
FuelAnalyzer        fuelAnalyzer;
EngineHealthAnalyzer healthAnalyzer;
DashboardUI         dashboard;

// ─── FreeRTOS Handles ─────────────────────────────────────────
TaskHandle_t ecuTaskHandle     = NULL;
TaskHandle_t displayTaskHandle = NULL;
SemaphoreHandle_t dataMutex    = NULL;

// ─── Shared Data (protected by mutex) ────────────────────────
volatile ECUData    sharedECUData;
volatile bool       ecuDataReady = false;

// ─── Uptime tracking ──────────────────────────────────────────
uint32_t systemStartMs = 0;

// ─── Forward Declarations ─────────────────────────────────────
void ecuTask(void* pvParameters);
void displayTask(void* pvParameters);
void printSystemInfo();
void printPeriodicDebug();

// ============================================================
// SETUP
// ============================================================
void setup() {
    // Serial debug init
    Serial.begin(115200);
    while (!Serial && millis() < 3000); // Tunggu serial max 3 detik
    delay(500);

    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║     MOTORCYCLE ECU DASHBOARD v1.0        ║");
    Serial.println("║     ESP32-S3 | K-Line | TFT 5\"           ║");
    Serial.println("╚══════════════════════════════════════════╝");

    systemStartMs = millis();
    printSystemInfo();

    // ─── Inisialisasi Watchdog ────────────────────────────────
    esp_task_wdt_init(WDT_TIMEOUT_SEC, true); // true = panic on timeout
    esp_task_wdt_add(NULL); // Daftarkan main task
    Serial.printf("[SYS] Watchdog initialized (%d sec timeout)\n", WDT_TIMEOUT_SEC);

    // ─── Mutex untuk shared data ──────────────────────────────
    dataMutex = xSemaphoreCreateMutex();
    if (!dataMutex) {
        Serial.println("[SYS][FATAL] Mutex creation failed!");
        while(1);
    }

    // ─── Inisialisasi SPI ─────────────────────────────────────
    // TFT_eSPI menggunakan konfigurasi dari User_Setup.h
    // SPI: MOSI=23, MISO=19, SCK=18
    Serial.println("[SYS] SPI initialized");

    // ─── Inisialisasi Display (harus di core display) ─────────
    Serial.println("[SYS] Starting display task...");
    xTaskCreatePinnedToCore(
        displayTask,
        "DisplayTask",
        STACK_DISPLAY,
        NULL,
        PRIO_DISPLAY,
        &displayTaskHandle,
        CORE_DISPLAY
    );

    delay(2000); // Tunggu display init selesai

    // ─── Inisialisasi ECU Task ────────────────────────────────
    Serial.println("[SYS] Starting ECU communication task...");
    xTaskCreatePinnedToCore(
        ecuTask,
        "ECUTask",
        STACK_ECU,
        NULL,
        PRIO_ECU,
        &ecuTaskHandle,
        CORE_ECU
    );

    Serial.println("[SYS] All tasks started. Main loop running.");
}

// ============================================================
// MAIN LOOP (Main task - watchdog feed)
// ============================================================
void loop() {
    // Feed watchdog dari main task
    esp_task_wdt_reset();

    // Periodic debug print setiap 5 detik
    static uint32_t lastDebugMs = 0;
    if (millis() - lastDebugMs > 5000) {
        lastDebugMs = millis();
        printPeriodicDebug();
    }

    // Main task tidak melakukan banyak - semua ada di FreeRTOS tasks
    vTaskDelay(pdMS_TO_TICKS(100));
}

// ============================================================
// ECU TASK - Core 1
// ============================================================
/**
 * Task dedicated untuk komunikasi K-Line dengan ECU
 * Berjalan di Core 1 untuk tidak mengganggu display
 */
void ecuTask(void* pvParameters) {
    Serial.printf("[ECUTask] Started on Core %d\n", xPortGetCoreID());

    // Init ECU Manager
    bool ecuInitialized = ecuManager.begin();
    if (!ecuInitialized) {
        Serial.println("[ECUTask][WARN] ECU init failed, will retry...");
    }

    // Tambahkan task ke watchdog
    esp_task_wdt_add(NULL);

    uint32_t loopCount = 0;

    while (true) {
        // Feed watchdog
        esp_task_wdt_reset();

        // Update ECU Manager (mengirim request PID secara bergilir)
        ecuManager.update();

        // Ambil data terbaru dari ECU
        const ECUData& rawData = ecuManager.getData();

        // Update analyzers dengan data baru
        if (rawData.valid) {
            afrAnalyzer.update(rawData);

            // Update ECUData dengan AFR hasil analisis
            ECUData updatedData = rawData;
            updatedData.afr = afrAnalyzer.getAFR();

            fuelAnalyzer.update(updatedData);
            healthAnalyzer.update(updatedData, afrAnalyzer.getStability());

            updatedData.fuelConsumption    = fuelAnalyzer.getInstantConsumption();
            updatedData.avgFuelConsumption = fuelAnalyzer.getAverageConsumption();
            updatedData.engineHealth       = healthAnalyzer.getHealthPercent();
            updatedData.engineStatus       = healthAnalyzer.getStatusString();

            // Update shared data dengan mutex protection
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                memcpy((void*)&sharedECUData, &updatedData, sizeof(ECUData));
                ecuDataReady = true;
                xSemaphoreGive(dataMutex);
            }
        } else {
            // Data tidak valid, tandai
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                sharedECUData.valid = false;
                ecuDataReady = true;
                xSemaphoreGive(dataMutex);
            }
        }

        loopCount++;

        // Yield ke task lain
        vTaskDelay(pdMS_TO_TICKS(20)); // ~50 Hz max untuk ECU task
    }
}

// ============================================================
// DISPLAY TASK - Core 0
// ============================================================
/**
 * Task dedicated untuk update TFT display
 * Berjalan di Core 0 untuk tidak memblokir ECU communication
 */
void displayTask(void* pvParameters) {
    Serial.printf("[DispTask] Started on Core %d\n", xPortGetCoreID());

    // Init display
    if (!dashboard.begin()) {
        Serial.println("[DispTask][ERROR] Display init failed!");
        vTaskDelete(NULL);
        return;
    }

    // Local copy data untuk rendering
    ECUData localData;
    memset(&localData, 0, sizeof(ECUData));
    localData.batteryVoltage = 12.0f; // Default
    localData.engineStatus   = "Initializing";
    localData.valid          = false;

    // Tambahkan ke watchdog
    esp_task_wdt_add(NULL);

    uint32_t frameCount = 0;
    uint32_t lastFpsMs  = 0;
    uint16_t fps        = 0;

    while (true) {
        esp_task_wdt_reset();

        uint32_t frameStart = millis();

        // Copy shared data ke local (mutex protected)
        if (ecuDataReady) {
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                memcpy(&localData, (void*)&sharedECUData, sizeof(ECUData));
                xSemaphoreGive(dataMutex);
            }
        }

        // Handle touch
        dashboard.handleTouch();

        // Update display
        dashboard.update(localData, afrAnalyzer, fuelAnalyzer, healthAnalyzer);

        // FPS counter
        frameCount++;
        if (millis() - lastFpsMs >= 1000) {
            fps = frameCount;
            frameCount = 0;
            lastFpsMs = millis();
            Serial.printf("[DispTask] FPS: %d\n", fps);
        }

        // Target ~30 FPS untuk dashboard smooth
        uint32_t elapsed = millis() - frameStart;
        int32_t delay = 33 - elapsed; // 33ms = ~30fps
        if (delay > 0) {
            vTaskDelay(pdMS_TO_TICKS(delay));
        } else {
            vTaskDelay(pdMS_TO_TICKS(1)); // Minimal yield
        }
    }
}

// ============================================================
// UTILITY FUNCTIONS
// ============================================================

// ─── Print System Info ────────────────────────────────────────
void printSystemInfo() {
    Serial.println("\n[SYS] === SYSTEM INFORMATION ===");
    Serial.printf("[SYS] ESP32-S3 CPU Freq: %d MHz\n", getCpuFrequencyMhz());
    Serial.printf("[SYS] Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("[SYS] Flash Size: %d MB\n", ESP.getFlashChipSize() / 1024 / 1024);
    Serial.printf("[SYS] SDK Version: %s\n", ESP.getSdkVersion());
    Serial.println("[SYS] ================================");

    Serial.println("\n[SYS] === PIN CONFIGURATION ===");
    Serial.printf("[SYS] K-Line RX: GPIO%d\n", KLINE_RX_PIN);
    Serial.printf("[SYS] K-Line TX: GPIO%d\n", KLINE_TX_PIN);
    Serial.printf("[SYS] K-Line BAUD: %d\n", KLINE_BAUD);
    Serial.printf("[SYS] TFT: MOSI=23, MISO=19, SCK=18, CS=5, DC=2, RST=4\n");
    Serial.printf("[SYS] Touch: CS=%d, IRQ=%d\n", TOUCH_CS_PIN, TOUCH_IRQ_PIN);
    Serial.printf("[SYS] Battery ADC: GPIO%d\n", BATTERY_ADC_PIN);
    Serial.println("[SYS] ================================\n");
}

// ─── Periodic Debug Print ─────────────────────────────────────
void printPeriodicDebug() {
    uint32_t uptime = (millis() - systemStartMs) / 1000;

    Serial.printf("\n[DBG][%lu s] === DASHBOARD STATUS ===\n", uptime);
    Serial.printf("[DBG] ECU: %s | Errors: %lu\n",
                  ecuManager.getConnectionStatus().c_str(),
                  ecuManager.getErrorCount());
    Serial.printf("[DBG] Free Heap: %d bytes\n", ESP.getFreeHeap());

    if (ecuDataReady && sharedECUData.valid) {
        Serial.printf("[DBG] RPM: %d | Speed: %d km/h\n",
                      sharedECUData.rpm, sharedECUData.speed);
        Serial.printf("[DBG] AFR: %.2f | Battery: %.2fV\n",
                      sharedECUData.afr, sharedECUData.batteryVoltage);
        Serial.printf("[DBG] Fuel: %.1f km/L | Health: %d%%\n",
                      sharedECUData.fuelConsumption,
                      sharedECUData.engineHealth);
        Serial.printf("[DBG] Engine Status: %s\n",
                      sharedECUData.engineStatus.c_str());
    } else {
        Serial.println("[DBG] ECU data not available");
    }
    Serial.println("[DBG] ============================\n");
}
