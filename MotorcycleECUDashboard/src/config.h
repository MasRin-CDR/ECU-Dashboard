#ifndef CONFIG_H
#define CONFIG_H

// Pin configuration for ESP32-S3
#define KLINE_RX_PIN       16  // K-Line RX pin
#define KLINE_TX_PIN       17  // K-Line TX pin

// TFT display pin configuration
#define TFT_MOSI_PIN       23  // TFT MOSI pin
#define TFT_MISO_PIN       19  // TFT MISO pin
#define TFT_SCK_PIN        18  // TFT SCK pin
#define TFT_CS_PIN         5   // TFT Chip Select pin
#define TFT_DC_PIN         2   // TFT Data/Command pin
#define TFT_RST_PIN        4   // TFT Reset pin

// Touchscreen pin configuration
#define TOUCH_CS_PIN       15  // Touchscreen Chip Select pin
#define TOUCH_IRQ_PIN      27  // Touchscreen Interrupt pin

// Communication settings
#define KLINE_SLOW_INIT    0x01  // Slow initialization command
#define KLINE_FAST_INIT    0x02  // Fast initialization command
#define ECU_HANDSHAKE_CMD  0x03  // ECU handshake command

// Dashboard update settings
#define DASHBOARD_UPDATE_INTERVAL 100  // Update interval in milliseconds

// Engine health thresholds
#define BATTERY_LOW_THRESHOLD      11.5  // Battery voltage low threshold
#define AFR_LEAN_THRESHOLD         14.7  // AFR lean threshold
#define AFR_RICH_THRESHOLD         12.5  // AFR rich threshold

// Engine health status levels
#define ENGINE_HEALTH_EXCELLENT    100
#define ENGINE_HEALTH_GOOD         75
#define ENGINE_HEALTH_WARNING       50
#define ENGINE_HEALTH_CRITICAL      25

#endif // CONFIG_H