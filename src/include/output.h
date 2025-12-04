/*
 * ======================================================================================================================
 * output.h - OLED and Serial Console Definations
 * ======================================================================================================================
 */
#include <Particle.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*
 * ======================================================================================================================
 *  OLED Display
 * ======================================================================================================================
 */
#define SCREEN_WIDTH        128 // OLED display width, in pixels
#define OLED32_I2C_ADDRESS  0x3C // 128x32 - https://www.adafruit.com/product/4440
#define OLED64_I2C_ADDRESS  0x3D // 128x64 - https://www.adafruit.com/product/326
#define OLED_RESET          -1 // -1 = Not in use
#define OLED32              (oled_type == OLED32_I2C_ADDRESS)
#define OLED64              (oled_type == OLED64_I2C_ADDRESS)

// Extern variables
extern int  SCE_PIN;
extern bool SerialConsoleEnabled;
extern bool DisplayEnabled;
extern char oled_lines[8][23];
extern int  oled_type;

// Function prototypes
void OLED_sleepDisplay();
void OLED_wakeDisplay();
void OLED_ClearDisplayBuffer();
void OLED_spin();
void OLED_update();
void OLED_write(const char *str);
void OLED_write_noscroll(const char *str);
void OLED_initialize();
void Serial_write(const char *str);
void Serial_writeln(const char *str);
void Serial_Initialize();
void Output(const char *str);
void OutputNS(const char *str);
void Output_Initialize();