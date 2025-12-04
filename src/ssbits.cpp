/*
 * ======================================================================================================================
 *  ssbits.cpp - System Status Bits Definations  - Sent ast part of the observation as hth (Health)
 * ======================================================================================================================
 */
#include "include/main.h"
#include "include/ssbits.h"

/*
 * ======================================================================================================================
 * Variables and Data Structures
 * =======================================================================================================================
 */
unsigned long SystemStatusBits = SSB_PWRON; // Set bit 1 to 1 for initial value power on. Is set to 0 after first obs

/*
 * ======================================================================================================================
 * Fuction Definations
 * =======================================================================================================================
 */

/*
 * ======================================================================================================================
 * JPO_ClearBits() - Clear System Status Bits related to initialization
 * ======================================================================================================================
 */
void JPO_ClearBits() {
  if (JustPoweredOn) {
    JustPoweredOn = false;
    SystemStatusBits &= ~SSB_PWRON;   // Turn Off Power On Bit
    // SystemStatusBits &= ~SB_SD;    // Turn Off SD Missing Bit - Required keep On
    // SystemStatusBits &= ~SSB_RTC;  // Turn Off RTC Missing Bit - Required keep On
  }
}