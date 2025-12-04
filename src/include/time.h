/*
 * ======================================================================================================================
 *  time.h - Time Management Definations
 * ======================================================================================================================
 */
#include <RTClib.h>

#define PCF8523_ADDRESS 0x68       // I2C address for PCF8523
#define MS_BTWN_TIMEUPDATES (2*3600*1000)    // 2 hours

// Extern variables
extern RTC_PCF8523 rtc;
extern DateTime now;
extern char timestamp[128];
extern bool RTC_valid;
extern bool RTC_exists;

// Function prototypes
void stc_timestamp();
void rtc_timestamp();
void rtc_initialize();
void RTC_UpdateCheck();