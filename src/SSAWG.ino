PRODUCT_VERSION(6);
#define COPYRIGHT "Copyright [2024] [University Corporation for Atmospheric Research]"
#define VERSION_INFO "SSAWG-20240711"

/*
 *======================================================================================================================
 * StormSurgeAndWindGauge (SSAWG)
 *   Board Type : Particle BoronLTE
 *   Description: Storm Surge Gauge follows NOAA's method to determine water level 
 *   Author: Robert Bubon
 *   Date:  2021-07-28 RJB Based on SSG and Full station's Wind
 *          2021-08-02 RJB Take 5 samples and use the median for the reading 
 *          2021-10-07 RJB Production Release to Particle V1
 *          2021-11-12 RJB Version 2 
 *                         Battery Charger Fault Register, Variable name "cfr" Added
 *                         Adafruit BMP388 Support Added
 *                         Remote Reboot Added
 *                         PMIC was changed to match other code, low battery shutdown conditional changed
 *                         Bug: msgbuf 256 was too small increased to 1024
 *                         Added CloudDisconnectOptions().graceful(true).timeout(5s)
 *          2023-01-22 RJB Bug HTU had humidity and Temp flipped
 *          2023-07-30 RJB Fixed bug to get RTC updated from network time
 *          2023-08-22 RJB Moved from 3.0.0 Firmware to 4.0.2
 *                         Update BMX sensor code
 *                         Added Min Max Quality Controls to sensor values
 *                         Added WITH_ACK to Particle Publish
 *                         Moved Wind Speed pin from D6 to A2
 *                         Added EEPROM support for N2S position
 *                         Added support for Watchdog Monitor
 *                         Brought over output initialization code from FSAC - now support 8 line oleds
 *                         Added MCP and SHT sensor support
 *          2023-08-28 RJB Bug Fix was sending sht for mcp reading
 *          2023-09-12 RJB Hardened Time handeling
 *          2024-02-23 RJB Version 5 At boot look for a file "5MDIST.TXT" on the SD.  
 *                         If file exists then we multiply by 1.25. If no file, then we multiply by 2.5 for the 10m Sensor. 
 *                         Particle console will support 3 DoAction commands, "REBOOT", "10MDIST", "5MDIST". 
 *          2024-04-30 RJB Version 6 
 *                         Added Argon support
 *                         Added 22hr reboot
 *                         Added 3rd party sim support
 *                         Moved SCE Pin from A4 to D8
 *                         Improved Station Monitor output
 *          2024-05-20 RJB Version 6 Cont.
 *                         Improved Station Monitor output
 *                         Dynamic display support added.
 *                         SHT, HIH, LUX, SI1145 sensor support added 
 *                         Updated I2C_Check_Sensors()
 *          2024-06-23 RJB Added Copyright
 *          2024-07-11 RJB Broke code down to #include files
 * 
 *  https://tidesandcurrents.noaa.gov/publications/CO-OPS_Measurement_Spec.pdf
 *  Air acoustic sensor mounted in protective well
 *   181 one-second water level samples centered on each tenth of an hour are averaged, a three standard deviation 
 *   outlier rejection test applied, the mean and standard deviation are recalculated and reported along with 
 *   the number of outliers. (3 minute water level average).
 *
 * NOTES:
 * When there is a successful transmission of an observation any need to send obersavations will be sent. 
 * On transmit a failure of these need to send observations, processing is stopped and the file is deleted.
 * 
 * Gauge Calibration
 * Adding serial console jumper after boot will cause gauge to be read every 1 second and value printed.
 * Removing serial console jumper will resume normal operation
 * 
 * Requires Library
 *  SdFat                   by Bill Greiman
 *  RTCLibrary
 *  Adafruit_SSD1306_RK     I2C ADDRESS 0x3C
 *  Adafruit_BM(PE)280      I2C ADDRESS 0x77  (SD0 to GND = 0x76)
 *  adafruit-htu21df        I2C ADDRESS 0x40
 *  Adafruit_BMP3XX         I2C ADDRESS 0x77 and (SD0 to GND = 0x76)
 *  Adafruit_SHT31          I2C ADDRESS 0x44 and 0x45 when ADR Pin High
 *  Adafruit_MCP9808        I2C ADDRESS 0x18
 * 
 * System.batteryState()
 *  0 = BATTERY_STATE_UNKNOWN
 *  1 = BATTERY_STATE_NOT_CHARGING
 *  2 = BATTERY_STATE_CHARGING
 *  3 = BATTERY_STATE_CHARGED
 *  4 = BATTERY_STATE_DISCHARGING
 *  5 = BATTERY_STATE_FAULT
 *  6 = BATTERY_STATE_DISCONNECTED
 * 
 * Publish to Particle
 *  Event Name: SS
 *  Event Variables:
 *   at     timestamp
 *   wl     water_level
 *   wld    water_level_stdev
 *   wlo    water_level_outliers
 *   wlm    water_level_mean
 *   wlr    water_level_raw
 *   ws     wind_speed
 *   wd     wind_direction
 *   wg     wind_gust
 *   wgd    wind_gust_direction
 *   bp1    bmx_pressure
 *   bt1    bmx_temp
 *   bh1    bmx_humid
 *   bp2    bmx_pressure
 *   bt2    bmx_temp
 *   bh2    bmx_humid
 *   hh1    htu_humid
 *   ht1    htu_temp
 *   sh1    sht_humid
 *   st1    sht_temp
 *   sh2    sht_humid
 *   st2    sht_temp
 *   mt1    mcp_temp
 *   mt2    mcp_temp
 *   bcs    Battery Charger Status
 *   bpc    Battery Percent Charge
 *   cfr    Charger Fault Register
 *   css    Cell Signal Strength
 *   hth    Health 16bits - See System Status Bits in below define statements
 * 
 * AN002 Device Powerdown
 * https://support.particle.io/hc/en-us/articles/360044252554?input_string=how+to+handle+low+battery+and+recovery
 * 
 * Distance Sensors
 * The 5-meter sensors (MB7360, MB7369, MB7380, and MB7389) use a scale factor of (Vcc/5120) per 1-mm.
 * Particle 12bit resolution (0-4095),  Sensor has a resolution of 0 - 5119mm,  Each unit of the 0-4095 resolution is 1.25mm
 * 
 * The 10-meter sensors (MB7363, MB7366, MB7383, and MB7386) use a scale factor of (Vcc/10240) per 1-mm.
 * Particle 12bit resolution (0-4095),  Sensor has a resolution of 0 - 10239mm,  Each unit of the 0-4095 resolution is 2.5mm
 * 
 * NOTE: Compile Issues
 * If you have compile issues like multiple definations of functions then you need to clean the compile directory out
 *    ~/.particle/toolchains/deviceOS/2.0.1/build/target/user/...
 * 
 * ========================================================
 * Support for 3rd Party Sim 
 * ========================================================
 *   SEE https://support.particle.io/hc/en-us/articles/360039741113-Using-3rd-party-SIM-cards
 *   SEE https://docs.particle.io/cards/firmware/cellular/setcredentials/
 *   Logic
 *     Output how sim is configured (internal or external)
 *     If console is enabled and SD found and SIM.TXT exists at the top level of SD card
 *       Read 1st line from SIM.TXT. Parse line for one of the below patterns
 *        INTERNAL
 *        AUP epc.tmobile.com username passwd
 *        UP username password
 *        APN epc.tmobile.com
 *      Perform appropriate actions to set sim
 *      Rename file to SIMOLD.TXT, so we don't do this on next boot
 *      Output notification to user to reboot then flash board led forever
 *
 * ========================================================
 * Support for Argon WiFi Boards
 * ========================================================
 * At the top level of the SD card make a file called WIFI.TXT
 * Add one line to the file
 * This line has 3 items that are comma separated Example
 * 
 * AuthType,ssid,password
 * 
 * Where AuthType is one of these keywords (WEP WPA WPA2)
 * ======================================================================================================================
 */

#define W4SC false   // Setk true to Wait for Serial Console to be connected

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_HTU21DF.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_SI1145.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_VEML7700.h>
#include <RTClib.h>
#include <SdFat.h>

/*
 * ======================================================================================================================
 *  Relay Power Control Pin
 * ======================================================================================================================
 */
#define REBOOT_PIN            A0  // Trigger Watchdog or external relay to cycle power
#define HEARTBEAT_PIN         A1  // Watchdog Heartbeat Keep Alive

/*
 * ======================================================================================================================
 *  Timers
 * ======================================================================================================================
 */
#define DELAY_NO_RTC              1000*60    // Loop delay when we have no valided RTC
#define CLOUD_CONNECTION_TIMEOUT  90         // Wait for N seconds to connect to the Cell Network
#define TIME_IN_ULP_MODE          14min      // Plus 1 minute is spent in connection overhead = 15 minute obs period

/*
 * ======================================================================================================================
 * System Status Bits used for report health of systems - 0 = OK
 * 
 * OFF =   SSB &= ~SSB_PWRON
 * ON =    SSB |= SSB_PWROFF
 * 
 * ======================================================================================================================
 */
#define SSB_PWRON            0x1   // Set at power on, but cleared after first observation
#define SSB_SD               0x2   // Set if SD missing at boot or other SD related issues
#define SSB_RTC              0x4   // Set if RTC missing at boot
#define SSB_OLED             0x8   // Set if OLED missing at boot, but cleared after first observation
#define SSB_N2S             0x10   // Set when Need to Send observations exist
#define SSB_FROM_N2S        0x20   // Set in transmitted N2S observation when finally transmitted
#define SSB_AS5600          0x40   // Set if wind direction sensor AS5600 not found or has issues                   
#define SSB_BMX_1           0x80   // Set if Barometric Pressure & Altitude Sensor missing
#define SSB_BMX_2          0x100   // Set if Barometric Pressure & Altitude Sensor missing
#define SSB_HTU21DF        0x200   // Set if Humidity & Temp Sensor missing
#define SSB_SI1145         0x400   // Set if UV index & IR & Visible Sensor missing
#define SSB_MCP_1          0x800   // Set if Precision I2C Temperature Sensor missing
#define SSB_MCP_2         0x1000   // Set if Precision I2C Temperature Sensor missing
#define SSB_SHT_1         0x2000   // Set if SHTX1 Sensor missing
#define SSB_SHT_2         0x4000   // Set if SHTX2 Sensor missing
#define SSB_HIH8          0x8000   // Set if HIH8000 Sensor missing
#define SSB_LUX          0x10000   // Set if VEML7700 Sensor missing
#define SSB_PM25AQI      0x20000   // Set if PM25AQI Sensor missing


unsigned long SystemStatusBits = SSB_PWRON; // Set bit 0 for initial value power on. Bit 0 is cleared after first obs
bool JustPoweredOn = true;         // Used to clear SystemStatusBits set during power on device discovery

/*
 * ======================================================================================================================
 *  Globals
 * ======================================================================================================================
 */
#define MAX_MSGBUF_SIZE 1024
char msgbuf[MAX_MSGBUF_SIZE]; // Used to hold messages
char *msgp;                   // Pointer to message text
char Buffer32Bytes[32];       // General storage

int  LED_PIN = D7;            // Built in LED
bool PostedResults;           // How we did in posting Observation and Need to Send Observations

uint64_t lastOBS = 0;         // time of next observation
int countdown = 1800;         // Exit calibration mode when reaches 0 - protects against burnt out pin or forgotten jumper

uint64_t LastTimeUpdate = 0;
uint64_t StartedConnecting = 0;
bool ParticleConnecting = false;

int  cf_reboot_countdown_timer = 79200; // There is overhead transmitting data so take off 2 hours from 86400s
                                        // Set to 0 to disable feature
int DailyRebootCountDownTimer;

bool firmwareUpdateInProgress = false;

#if PLATFORM_ID == PLATFORM_BORON
/*
 * ======================================================================================================================
 *  Power Management IC (bq24195)
 * ======================================================================================================================
 */
PMIC pmic;
#endif

/*
 * ======================================================================================================================
 *  Observation Timing
 *   Check current time to find which of the 10 obs periods we are in.
 *   If last_obs_time is not in this period, do an observerion and update the last_obs_time
 *   Report times centered around 0,6,12,18,24,30,36,42,48,54. Add 90s after each to get below report times
 * ======================================================================================================================
 */
struct {
  int    sath[10]          = { 90, 450, 810, 1170, 1530, 1890, 2250, 2610, 2790, 3330};  // Seconds After The Hour
  time_t last_obs_time[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
} OBS_Period;


/*
 * ======================================================================================================================
 *  SD Card
 * ======================================================================================================================
 */
#define SD_ChipSelect D5                // GPIO 10 is Pin 10 on Feather and D5 on Particle Boron Board
SdFat SD;                               // File system object.
File SD_fp;
char SD_obsdir[] = "/OBS";              // Store our obs in this directory. At Power on, it is created if does not exist
bool SD_exists = false;                     // Set to true if SD card found at boot
char SD_n2s_file[] = "N2SOBS.TXT";          // Need To Send Observation file
uint32_t SD_n2s_max_filesz = 200 * 8 * 24;  // Keep a little over 2 days. When it fills, it is deleted and we start over.

char SD_sim_file[] = "SIM.TXT";         // File used to set Ineternal or External sim configuration
char SD_simold_file[] = "SIMOLD.TXT";   // SIM.TXT renamed to this after sim configuration set

char SD_wifi_file[] = "WIFI.TXT";         // File used to set WiFi configuration

/*
 * ======================================================================================================================
 * HeartBeat() - Burns 250 ms 
 * ======================================================================================================================
 */
void HeartBeat() {
  digitalWrite(HEARTBEAT_PIN, HIGH);
  delay(250);
  digitalWrite(HEARTBEAT_PIN, LOW);
}

/*
 * ======================================================================================================================
 *  Local Code Includes - Do not change the order of the below 
 * ======================================================================================================================
 */
#include "QC.h"                   // Quality Control Min and Max Sensor Values on Surface of the Earth
#include "SF.h"                   // Support Functions
#include "OP.h"                   // OutPut support for OLED and Serial Console
#include "CF.h"                   // Configuration File Variables
#include "TM.h"                   // Time Management
#include "Sensors.h"              // I2C Based Sensors
#include "WRD.h"                  // Wind Rain Distance
#include "EP.h"                   // EEPROM
#include "SDC.h"                  // SD Card
#include "OBS.h"                  // Do Observation Processing
#include "SM.h"                   // Station Monitor
#include "PS.h"                   // Particle Support Functions

/*
 * ======================================================================================================================
 * OBS_TimeCheck() - See if it is time to make an observation
 * ======================================================================================================================
 */
bool OBS_TimeCheck() {
  int obs_period=9; // assume last period so we don't have to check roll over on the hour
  time_t current_time = Time.now();
  int sath = current_time % 3600; // Seconds After the Hour

  // Locate what observation period in the hour we are in
  for (int p=0; p<9; p++) {
    if ((p<5) && ((sath >= OBS_Period.sath[p]) && (sath < OBS_Period.sath[p+1])) ) {
      obs_period = p;
      break;
    }
  }
 
  // Use last_obs_time with current time to determine if we have done a observation or need to do - for this period
  if ( (current_time - OBS_Period.last_obs_time[obs_period]) > 360) {
    OBS_Period.last_obs_time[obs_period] = current_time;
    return (true);
  } 
  return (false); // Under 600s so we have already done the observation for this period
}


// You must use SEMI_AUTOMATIC or MANUAL mode so the battery is properly reconnected on
// power-up. If you use AUTOMATIC, you may be unable to connect to the cloud, especially
// on a 2G/3G device without the battery.
SYSTEM_MODE(SEMI_AUTOMATIC);

// https://docs.particle.io/cards/firmware/system-thread/system-threading-behavior/
SYSTEM_THREAD(ENABLED);

/*
 * ======================================================================================================================
 * setup() - runs once, when the device is first turned on.
 * ======================================================================================================================
 */
void setup() {
  // The device has booted, reconnect the battery.
#if PLATFORM_ID == PLATFORM_BORON
	pmic.enableBATFET();
#endif

  // Set Default Time Format
  Time.setFormat(TIME_FORMAT_ISO8601_FULL);

  // WatchDog 
  pinMode (REBOOT_PIN, OUTPUT);
  pinMode (HEARTBEAT_PIN, OUTPUT);

  // Put initialization like pinMode and begin functions here.
  pinMode (LED_PIN, OUTPUT);
  Output_Initialize();
  delay(2000); // Prevents usb driver crash on startup, Arduino needed this so keeping for Particle

  // Set up Stream gauge pin for reading 
  pinMode(DISTANCEGAUGE, INPUT);

  Serial_write(COPYRIGHT);
  Output (VERSION_INFO); // Doing it one more time for the OLED
  delay(4000);

  // The System.on() function is used to subscribe to system-level events and configure 
  // how the device should behave when these events occur.
  // Firmware update handler sets a global variable when the firmware update starts.
  // We do not want to go into low power mode during this update
  // System.on(firmware_update, firmwareUpdateHandler);

  // Set Daily Reboot Timer
  DailyRebootCountDownTimer = cf_reboot_countdown_timer;  

  // Initialize SD card if we have one.
  SD_initialize();

  // Report if we have Need to Send Observations
  if (SD_exists && SD.exists(SD_n2s_file)) {
    SystemStatusBits |= SSB_N2S; // Turn on Bit
    Output("N2S:Exists");
  }
  else {
    SystemStatusBits &= ~SSB_N2S; // Turn Off Bit
    Output("N2S:NF");
  }

  if (SD_exists && SD.exists(CF_NAME)) {
    SD_ReadConfigFile();
  }
  else {
    sprintf(msgbuf, "CF:NO %s", CF_NAME); Output (msgbuf);
    Output(msgbuf);
  }

  // Display EEPROM Information 
  EEPROM_Dump();
  
  // Check if correct time has been maintained by RTC
  // Uninitialized clock would be 2000-01-00T00:00:00
  stc_timestamp();
  sprintf (msgbuf, "%s+", timestamp);
  Output(msgbuf);

  // Read RTC and set system clock if RTC clock valid
  rtc_initialize();

  if (Time.isValid()) {
    Output("STC: Valid");
  }
  else {
    Output("STC: Not Valid");
  }

  stc_timestamp();
  sprintf (msgbuf, "%s=", timestamp);
  Output(msgbuf);

  // Adafruit i2c Sensors
  bmx_initialize();
  htu21d_initialize();
  mcp9808_initialize();
  sht_initialize();
  as5600_initialize();
  hih8_initialize();
  si1145_initialize();
  lux_initialize();

  if (SD.exists(SD_5M_DIST_FILE)) {
    od_adjustment = 1.25;
    Output ("DIST=5M");
  }
  else {
    od_adjustment = 2.5;
    Output ("DIST=10M");
  } 

  if (AS5600_exists) {
    // Initialize Wind Speed Interrupt Based Sensor - Optipolar Hall Effect Sensor SS451A
    anemometer_interrupt_count = 0;
    anemometer_interrupt_stime = System.millis();
    attachInterrupt(ANEMOMETER_IRQ_PIN, anemometer_interrupt_handler, FALLING); // Was D6 now A2
    DoWindOBS = true;
  }
 
#if PLATFORM_ID == PLATFORM_ARGON
  //==================================================
  // Check if we need to program for WiFi change
  //==================================================
  WiFiChangeCheck();
#else
  //==================================================
  // Check if we need to program for Sim change
  //==================================================
  SimChangeCheck();
#endif

  // Note if we call Particle.connect() and are not truely connected to the Cell network, Code blocks in particle call
  Particle.setDisconnectOptions(CloudDisconnectOptions().graceful(true).timeout(5s));
  Particle.connect();

  // Setup Remote Function to DoAction, Expects a parameter to be passed from Particle to control what action
  if (Particle.function("DoAction", Function_DoAction)) {
    Output ("DoAction:OK");
  }
  else {
    Output ("DoAction:ERR");
  }

  OBS_WindAndDistance_Fill();
}

/*
 * ======================================================================================================================
 * loop() runs over and over again, as quickly as it can execute.
 * ======================================================================================================================
 */
void loop() {
  uint64_t LastConnectCheckTime = 0;
  static bool FirstParticleConnect = true;

  // Take 1sec Water Level Sample - Do this all the time so we have the 181 samples when it is time to make observation
  OBS_Distance_Do();
  Wind_TakeReading();

  // If Serial Console Pin LOW then call StationMonitor()
  // Used for calibrating Stream Gauge at Installation
  // Only stay in this mode for countdown seconds, this protects against burnt out pin or forgotten jumper
  if (countdown && digitalRead(SCE_PIN) == LOW) {
    StationMonitor();
    countdown--; // Exit out if we forget to remove the jumper
  }
  else { // Normal Operation - Main Work
    // This will be invalid if the RTC was bad at poweron and we have not connected to Cell network
    // Upon connection to cell network system Time is set and this becomes valid
    if (Time.isValid()) {  

      // Set RTC from Cell network time.
      RTC_UpdateCheck();

      if (!eeprom_valid) {
        // We now a a valid clock so we can initialize the EEPROM
        EEPROM_Initialize();
      }

      // See if Time to Make Observation
      if (OBS_TimeCheck()) {
        Check_I2C_Sensors(); // Make sure Sensors are online
        OBS_Do(); // If Particle connected we save OBS to N2S

        // Shutoff System Status Bits related to initialization after we have logged first observation 
        JPO_ClearBits();
      }

      if (Particle.connected()) { 
        // Do some work
        LastConnectCheckTime = System.millis();
        if (FirstParticleConnect) {
          Output ("Connected");
          FirstParticleConnect = false;
        }
      }
#if PLATFORM_ID == PLATFORM_ARGON
      // See if it's been an hour without a network connection and transmission of data
      // With Argon WiFi we have seen it stuck in Breathing Green - Trying to connect
      else if (System.millis() - LastConnectCheckTime > (3600 * 1000)) {  
        // Been too long with out a network connection, lets reboot
        Output("1HR W/O NW: Rebooting");
        delay(5000);
        System.reset();
      }
#endif
    }
    else {
      stc_timestamp();
      Output(timestamp);
      Output("ERR: No Clock");
      delay (DELAY_NO_RTC);
    }

    // Reboot Boot Every 22+ hours - Not using time but a loop counter.
    if ((cf_reboot_countdown_timer>0) && (--DailyRebootCountDownTimer<=0)) {
      Output ("Daily Reboot");

      delay(1000);

      // Lets not rip the rug out from the modem. Do a graceful shutdown.
      Particle.disconnect();
      waitFor(Particle.disconnected, 1000);  // Returns true when disconnected from the Cloud.

 #if PLATFORM_ID == PLATFORM_BORON     
      // Be kind to the cell modem and try to shut it down
      Cellular.disconnect();
      delay(1000);
      Cellular.off();
#endif

      Output("Rebooting");  
      delay(1000);
   
      DeviceReset();

      // We should never get here, but just incase 
      Output("I'm Alive! Why?");  

#if PLATFORM_ID == PLATFORM_BORON
		  Cellular.on();
      delay(1000);
#endif

		  Particle.connect();

      DailyRebootCountDownTimer = cf_reboot_countdown_timer; // Reset count incase reboot fails

      // We need to reinitialize our wind readings before we can move on.
      OBS_WindAndDistance_Fill();
    }

#if PLATFORM_ID == PLATFORM_BORON
    // Check if we are not connected to a charging source and our battery is at a low level. 
    // If so then power down the display and board. Wait for power to return.
    //
    // We do this at a high enough battery level to avoid the board from powering
    // itself down out of our control. The goal is to leave enough battery for the sensors to
    // chew on and still be able, when power returns, to charge the battery and transmit 
    // with out a current drop causing the board to reset or power down out of our control.

    // We are on battery and have 10% or less percent left then turnoff and wait for power to return
    if (!pmic.isPowerGood() && (System.batteryCharge() <= 10.0)) {

      Output("Low Power!");

      // Disconnect from the cloud and power down the modem.
      Particle.disconnect();
      waitFor(Particle.disconnected, 1000);  // Returns true when disconnected from the Cloud.

      Cellular.disconnect();
      delay(1000);
      Cellular.off();

      Output("Powering Down");

      OLED_sleepDisplay();
      delay(5000);

      // Disabling the BATFET disconnects the battery from the PMIC. Since there
      // is no longer external power, this will turn off the device.
		  pmic.disableBATFET();

		  // This line should not be reached. When power is applied again, the device
		  // will cold boot starting with setup().

		  // However, there is a potential for power to be re-applied while we were in
		  // the process of shutting down so if we're still running, enable the BATFET
		  // again and reconnect to the cloud. Wait a bit before doing this so the
		  // device has time to actually power off.
		  delay(2000);

      OLED_wakeDisplay();   // May need to toggle the Display reset pin.
		  delay(2000);
		  Output("Power Re-applied");

		  pmic.enableBATFET();
		  Cellular.on();
		  Particle.connect();

      OBS_WindAndDistance_Fill();
    } // powerdown
#endif
  } // normal operation

  HeartBeat();  // Burns 250ms
  delay (710);  // 40ms was taken by median reading
} // loop
