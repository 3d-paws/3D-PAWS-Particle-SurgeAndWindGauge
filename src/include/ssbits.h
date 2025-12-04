/*
 * ======================================================================================================================
 *  ssbits.h - System Status Bits Definations  - Sent ast part of the observation as hth (Health)
 * ======================================================================================================================
 */

/*
 * ======================================================================================================================
 * OFF= SSB &= ~SSB_PWRON
 * ON = SSB |= SSB_PWROFF
 * 0  = OK
 * ======================================================================================================================
 */

#define SSB_PWRON           0x1       // Set at power on, but cleared after first observation
#define SSB_SD              0x2       // Set if SD missing at boot or other SD related issues
#define SSB_N2S             0x4       // Set when Need to Send observations exist
#define SSB_FROM_N2S        0x8       // Set in transmitted N2S observation when finally transmitted
#define SSB_RTC             0x10      // Set if RTC missing at boot

// Extern variables
extern unsigned long SystemStatusBits;

// Function prototypes
extern void JPO_ClearBits();