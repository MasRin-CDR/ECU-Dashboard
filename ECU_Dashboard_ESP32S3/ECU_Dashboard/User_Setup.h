/**
 * ============================================================
 * User_Setup.h
 * Konfigurasi TFT_eSPI untuk ILI9488 5" pada ESP32-S3
 * ============================================================
 * LETAKKAN FILE INI DI:
 *   ~/.arduino15/libraries/TFT_eSPI/User_Setup.h
 *   ATAU
 *   Arduino/libraries/TFT_eSPI/User_Setup.h
 * ============================================================
 */

//                            USER DEFINED SETTINGS
//   Set driver type, fonts to be loaded, pins used and SPI control method etc.
//
//   See the User_Setup_Select.h file if you wish to be able to define multiple
//   setups and then easily select which setup file is used by the compiler.
//
//   If this file is edited correctly then all the library example sketches should
//   run without any additional changes being needed.

// ─── Driver Selection ─────────────────────────────────────────
// Only define one driver, multiple definitions will produce an error

#define ILI9488_DRIVER      // 5" TFT ILI9488

// ─── Display Dimensions ───────────────────────────────────────
#define TFT_WIDTH   480
#define TFT_HEIGHT  320

// ─── Pins ────────────────────────────────────────────────────
#define TFT_MOSI    23
#define TFT_MISO    19
#define TFT_SCLK    18
#define TFT_CS       5   // Chip select
#define TFT_DC       2   // Data/Command
#define TFT_RST      4   // Reset

// ─── SPI Frequency ────────────────────────────────────────────
// ILI9488 max SPI clock: 40 MHz untuk write, 20 MHz untuk read
#define SPI_FREQUENCY       27000000    // 27 MHz - stabil untuk ILI9488
#define SPI_READ_FREQUENCY  20000000    // 20 MHz untuk read
#define SPI_TOUCH_FREQUENCY  2500000   // 2.5 MHz untuk touch XPT2046

// ─── Font Loading ─────────────────────────────────────────────
#define LOAD_GLCD    // Font 1. Original Adafruit 8 pixel font
#define LOAD_FONT2   // Font 2. Small 16 pixel high font
#define LOAD_FONT4   // Font 4. Medium 26 pixel high font
#define LOAD_FONT6   // Font 6. Large 48 pixel high font
#define LOAD_FONT7   // Font 7. 7 segment 48 pixel high font
#define LOAD_FONT8   // Font 8. Large 75 pixel font
#define LOAD_GFXFF   // FreeFonts. Include access to the 48 Adafruit_GFX free fonts

#define SMOOTH_FONT  // Enable smooth font rendering

// ─── Color Depth ──────────────────────────────────────────────
// ILI9488 native adalah 18-bit, library menghandle konversi
#define COLOR_ILI9488

// ─── Touch (XPT2046) dikonfigurasi via library terpisah ───────
// XPT2046_Touchscreen library: CS=GPIO15, IRQ=GPIO27

// ─── SPI Port ─────────────────────────────────────────────────
// ESP32-S3: SPI2 (FSPI) atau SPI3 (HSPI)
// Default menggunakan VSPI (SPI3): MOSI=23, MISO=19, SCK=18

// ─── Colour order BGR ─────────────────────────────────────────
// ILI9488 menggunakan BGR, bukan RGB
#define TFT_BGR_ORDER  // ILI9488 BGR colour order
