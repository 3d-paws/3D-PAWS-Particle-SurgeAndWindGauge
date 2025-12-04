/*
 * ======================================================================================================================
 *  support.h - Support Functions Definations
 * ======================================================================================================================
 */
#include <Particle.h>

// Extern variables
extern int LED_PIN;

// Function prototypes
bool I2C_Device_Exist(byte address);
void Blink(int count, int between);
void FadeOn(unsigned int time,int increament);
void FadeOff(unsigned int time,int decreament);
void mysort(unsigned int a[], unsigned int n);
bool isnumeric(char *s);
bool isValidNumberString(const char *str);
bool isValidHexString(const char *hexString, size_t expectedLength);
bool hexStringToUint32(const char *hexString, uint32_t *result);
void hexStringToByteArray(const char *hexString, uint8_t *byteArray, int len);