/*
 * ======================================================================================================================
 *  time.cpp - Time Management Functions
 * ====================================================================================================================== 
 */
#include <RTClib.h>

#include "include/ssbits.h"
#include "include/output.h"
#include "include/support.h"
#include "include/main.h"
#include "include/time.h"


/*
 * ======================================================================================================================
 * Variables and Data Structures
 * =======================================================================================================================
 */
RTC_PCF8523 rtc;              // RTC_PCF8523 rtc;
DateTime now;
char timestamp[128];
bool RTC_valid = false;
bool RTC_exists = false;

/*
 * ======================================================================================================================
 * Fuction Definations
 * =======================================================================================================================
 */

/* 
 *=======================================================================================================================
 * stc_timestamp() - Read from System Time Clock and set timestamp string
 *=======================================================================================================================
 */
void stc_timestamp() {
  time32_t t = Time.now();

  // ISO_8601 Time Format
  sprintf (timestamp, "%d-%02d-%02dT%02d:%02d:%02d", 
    Time.year(t), Time.month(t), Time.day(t),
    Time.hour(t), Time.minute(t), Time.second(t));
}

/* 
 *=======================================================================================================================
 * rtc_timestamp() - Read from RTC and set timestamp string
 *=======================================================================================================================
 */
void rtc_timestamp() {
  now = rtc.now(); //get the current date-time

  // ISO_8601 Time Format
  sprintf (timestamp, "%d-%02d-%02dT%02d:%02d:%02d", 
    now.year(), now.month(), now.day(),
    now.hour(), now.minute(), now.second());
}

/* 
 *=======================================================================================================================
 * rtc_initialize()
 *=======================================================================================================================
 */
void rtc_initialize() {

  if (!rtc.begin()) { // Always returns true
     Output("ERR:RTC NOT FOUND");
     SystemStatusBits |= SSB_RTC; // Turn on Bit
     return;
  }
  
  if (!I2C_Device_Exist(PCF8523_ADDRESS)) {
    Output("ERR:RTC-I2C NOTFOUND");
    SystemStatusBits |= SSB_RTC; // Turn on Bit
    delay (5000);
    return;
  }

  RTC_exists = true; // We have a clock hardware connected

  // Particle.io library is 1.2.1 (Old). The below are supported in 1.12.4

  //we don't need the 32K Pin, so disable it
  // rtc.disable32K();

  // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
  // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
  // rtc.clearAlarm(1);
  // rtc.clearAlarm(2);

  // stop oscillating signals at SQW Pin
  // otherwise setAlarm1 will fail
  // rtc.writeSqwPinMode(DS3231_OFF);

  // turn off alarm 1, 2 (in case it isn't off already)
  // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
  // rtc.disableAlarm(1);
  // rtc.disableAlarm(2);
  
  // now = rtc.now(); //get the current date-time - done in timestamp().

  rtc_timestamp();
  sprintf (msgbuf, "%s*", timestamp);
  Output (msgbuf);

  // Do a validation check on the year. 
  // Asumption is: If RTC not set, it will not have the current year.
  
  if (now.year() >= 2023) {
    now = rtc.now();
    Time.setTime(now.unixtime()); // If RTC valid, we set STC.
    RTC_valid = true;
  }
  else {
    Output ("NEED GSM TIME->RTC");
    delay (5000); // Give the user some time to see this problem.
  }
}

/*
 * ======================================================================================================================
 * RTC_UpdateCheck() - Check if we need to Set or Update the RTC clock from the Cell Network
 *                     Does a Particle.syncTime() with will force system clock to be updated
 * ======================================================================================================================
 */
void RTC_UpdateCheck() {
  static bool DoSyncTime = false;
  static int waitcounter=0;

  if (RTC_exists && Particle.connected()) { 
    // We have a RTC and We have connected to the Cell network at some point

    // Ask Particle to update system clock, then wait for this to complete, then update RTC for first time
    if (LastTimeUpdate == 0){

      // Ask Particle for time update
      if (!DoSyncTime) {
        Particle.syncTime();
        DoSyncTime = true;  // Set to true so for the next 2 minutes we aviod repeating
        Output ("1ST SYNC NET TIME");       
      }

      // Monitor if completed / ask again in 10 minutes
      else {
        // Time update completed
        if (Particle.syncTimeDone()) { // Returns true if Time sync completed
          // We have never updated RTC from Cell network time 
          rtc.adjust(DateTime(Time.year(), Time.month(), Time.day(), Time.hour(), Time.minute(), Time.second() ));
          RTC_valid = true;
          
          Output("RTC: 1ST SYNC");
          rtc_timestamp();
          sprintf (msgbuf, "%s*", timestamp);
          Output (msgbuf);
          DoSyncTime = false;
          LastTimeUpdate = System.millis();
        }
        // Time to Ask again
        else if (++waitcounter >= 300) {  // 300 loops is 5 minutes
          Particle.syncTime();  // Try again
          waitcounter = 0;
          Output ("1ST SYNC NET TIME+");
        }
      }
    }

    // 2 minutes before we update the RTC clock from system clock, get network time so system clock is current
    else if (!DoSyncTime && (System.millis() - LastTimeUpdate) >= (MS_BTWN_TIMEUPDATES-(60*2*1000)) ) {  
      Particle.syncTime();
      DoSyncTime = true;  // Set to true so for the next 2 minutes we aviod repeating
      Output ("SYNC NET TIME");
    }

    // Update the RTC clock from system clock. This assumes system clock was updated from the Particle call 2 minutes ago
    else if ((System.millis() - LastTimeUpdate) >= MS_BTWN_TIMEUPDATES) {
      DoSyncTime = false;
      rtc.adjust(DateTime(Time.year(), Time.month(), Time.day(), Time.hour(), Time.minute(), Time.second() ));

      Output("RTC: 2HR SYNC");
      rtc_timestamp();
      sprintf (msgbuf, "%s*", timestamp);
      Output (msgbuf);

      DoSyncTime = false;
      LastTimeUpdate = System.millis();
    }
  }
}
