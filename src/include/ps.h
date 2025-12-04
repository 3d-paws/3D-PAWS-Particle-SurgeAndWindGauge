/*
 * ======================================================================================================================
 *  ps.h - Particle Support Definations
 * ======================================================================================================================
 */

// Extern variables
#if (PLATFORM_ID == PLATFORM_BORON) || (PLATFORM_ID == PLATFORM_MSOM)
extern char imsi[16];
#endif


// Function prototype
void GetPinName(pin_t pin, char *pinname);
void OutputResetReason();
void Output_CellBatteryInfo();
void DeviceReset();
int Function_DoAction(String s);

#if PLATFORM_ID == PLATFORM_ARGON
void WiFiChangeCheck();
#endif

#if (PLATFORM_ID == PLATFORM_ARGON) || (PLATFORM_ID == PLATFORM_MSOM)
void WiFiPrintCredentials();
#else
void SimChangeCheck();
#endif

#if (PLATFORM_ID == PLATFORM_BORON) || (PLATFORM_ID == PLATFORM_MSOM)
int callback_imsi(int type, const char* buf, int len, char* cimi);
#endif