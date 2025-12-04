/*
 * ======================================================================================================================
 *  main.h - Main Code Definations
 * ======================================================================================================================
 */
#include <Particle.h>

/*
 * ======================================================================================================================
 *  Loop Timers
 * ======================================================================================================================
 */
#define DELAY_NO_RTC              1000*60    // Loop delay when we have no valided RTC
#define CLOUD_CONNECTION_TIMEOUT  90         // Wait for N seconds to connect to the Cell Network
#define TIME_IN_ULP_MODE          14min      // Plus 1 minute is spent in connection overhead = 15 minute obs period

/*
 * ======================================================================================================================
 *  Relay Power Control Pin
 * ======================================================================================================================
 */
#define REBOOT_PIN            A0  // Trigger Watchdog or external relay to cycle power
#define HEARTBEAT_PIN         A1  // Watchdog Heartbeat Keep Alive

#define MAX_MSGBUF_SIZE 1024

// Extern variables
extern char versioninfo[];
extern char msgbuf[MAX_MSGBUF_SIZE];
extern char *msgp;
extern char Buffer32Bytes[32];
extern int LED_PIN;
extern bool JustPoweredOn;
extern bool SendSystemInformation;
extern uint64_t LastTimeUpdate;
extern uint64_t lastOBS;
extern int DailyRebootCountDownTimer;

#if PLATFORM_ID == PLATFORM_BORON
extern PMIC pmic; // Power Management IC (bq24195) I2C 0x6B
#endif

// Function prototypes
void HeartBeat();