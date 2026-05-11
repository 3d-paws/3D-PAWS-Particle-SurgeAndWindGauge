PRODUCT_VERSION(13);
#define COPYRIGHT "Copyright [2025] [University Corporation for Atmospheric Research]"
#define VERSION_INFO "SSWG-20260511v13"

/*
 *======================================================================================================================
 * StormSurgeAndWindGauge (SSWG)
 *   Board Type : Particle BoronLTE / Particle Argon
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
 * 
 *          Version 5 Released on 2024-05-21
 *          2024-02-23 RJB At boot look for a file "5MDIST.TXT" on the SD.  
 *                         If file exists then we multiply by 1.25. If no file, then we multiply by 2.5 for the 10m Sensor. 
 *                         Particle console will support 3 DoAction commands, "REBOOT", "10MDIST", "5MDIST". 
 *          2024-04-30 RJB Added Argon support
 *                         Added 22hr reboot
 *                         Added 3rd party sim support
 *                         Moved SCE Pin from A4 to D8
 *                         Improved Station Monitor output
 *          2024-05-20 RJB Improved Station Monitor output
 *                         Dynamic display support added.
 *                         SHT, HIH, LUX, SI1145 sensor support added 
 *                         Updated I2C_Check_Sensors()
 * 
 *          Version 7 Released on 2024-12-18
 *          2024-06-23 RJB Added Copyright
 *          2024-07-11 RJB Broke code down to #include files
 *          2024-07-23 RJB Bug 5MDIST cmd was setting od_adjustment to 8.
 *                         Also set odg_adjustment after adding/removing 5MDIST.TXT file
 *          2024-09-09 RJB Tweek Station Monitor added 5M 10M to output
 *          2024-09-11 RJB Wind Initialization and Station Monitor changed to provide
 *                         better output about wind speed and direction.
 *                         When setting SIM to INTERNAL we now set changed = true to
 *                         report success and reboot message.
 *          2024-09-14 RJB Modified WiFi Support for UNSEC allowing no password Ex:  "UNSEC,ssid,"
 *          2024-11-05 RJB Discovered BMP390 first pressure reading is bad. Added read pressure to bmx_initialize()
 *                         Bug fixes for 2nd BMP sensor in bmx_initialize() using first sensor data structure
 *                         Now will only send humidity if bmx sensor supports it.
 *          2024-12-18 RJB Compiled with deviceOS6.1.1
 * 
 *          Version 8 Released on 2024-12-22
 *          2024-12-19 RJB Updated Adafruit_VEML7700 library to 2.1.6
 *          2024-12-22 RJB Added INFO support feature
 *                         Removed unnecessary globals and functions not used
 *                            bool ParticleConnecting = false; 
 *                            int StartedConnecting = 0;
 *                            bool PostedResults;
 *                            bool firmwareUpdateInProgress = false;
 *                            firmwareUpdateHandler()
 * 
 *          Version 9 Released on 2025-11-12
 *          2025-04-08 RJB Removed the check for serial console in SimChangeCheck()
 *          2025-11-12 RJB Bug Fix SHT temp sensor was also reporting as humidity.
 *                         Updated the OS to 6.3.3
 * 
 *          Version 10 Released on 2025-12-03
 *          2025-11-19 RJB Code cleanup aka .cpp
 *                         Add MSLP support
 *                         Tweeked EEPROM_Valid()
 *                         VEML7700 lux now reports as vlx
 *                         variable od_ changed to dg_
 *                         DISTANCEGAUGE moved from A3 to A4 
 *                         Added obtaining imsi and logging via INFO
 *                         Updated all the libraries.
 *                         See ReleaseNotes.txt file
 *                         Updated to ParticleOS 6.3.4
 * 
 *          Version 11 & 12 Released on 2026-05-07 - Skipped had a bug in as5600 being set true by default
 * 
 *          Version 13 Released on 2026-05-11
 *          2026-05-11 RJB Updated to ParticleOS 6.4.1
 *                         Added DoAction NOWIND (Default) & DOWIND
 *                         Now following the NOAA standard for Distance of 181s
 *                         OP1 will always be Distance. OP2 = OP2RAW | OP2VBV (Voltaic) | OP2CLR
 *                         Added BMP581 and SHT45 sensor support
 *                         Updated the handling of the BMx sensors.
 *                         Removed support for HIH sensor
 *                         Bug Fix SETELEV
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
 *   bp1    BME280,BMP280,BMP3XX,BMP4XX Pressure
 *   bt1    BME280,BMP280,BMP3XX,BMP4XX Temperature
 *   bh1    BME280,BMP280,BMP3XX,BMP4XX Humidity 
 *   bp2    BME280,BMP280,BMP3XX,BMP4XX Pressure
 *   bt2    BME280,BMP280,BMP3XX,BMP4XX Temperature
 *   bh2    BME280,BMP280,BMP3XX,BMP4XX Humidity
 *   bp3    BMP4XX Pressure
 *   bt3    BMP4XX Temperature
 *   bh3    BMP4XX Humidity
 *   bp4    BMP4XX Pressure
 *   bt4    BMP4XX Temperature
 *   bh4    BMP4XX Humidity
 *   hh1    htu_humid
 *   ht1    htu_temp
 *   sh1    sht_humid
 *   st1    sht_temp
 *   sh2    sht_humid
 *   st2    sht_temp
 *   mt1    mcp_temp
 *   mt2    mcp_temp
 *   hi     heat index
 *   mlsp   mean sea level pressure
 *   bcs    Battery Charger Status
 *   bpc    Battery Percent Charge
 *   cfr    Charger Fault Register
 *   css    Cell Signal Strength
 *   hth    Health 16bits - See System Status Bits in below define statements
 * 
 * 
 * ========================================================
 * Boron/Argon PIN Assignments
 * ========================================================
 * D8   = Serial Console (Ground Pin to Enable) - Not on Grove Shield
 * D7   = On Board LED - Blinks when console connection needed
 * D6   = Future Reserved for Lora IRQ - Not on Grove Shield
 * D5   = SD Card Chip Select
 * D4   = SPI1 MSIO - Future Reserved for LoRa
 * D3   = SPI1 MOSI - Future Reserved for LoRa
 * D2   = SPI1 SCK  - Future Reserved for LoRa
 * D1   = I2C SCL
 * D0   = I2C SDA
 * 
 * A0   = WatchDog Monitor/Relay Reset Trigger
 * A1   = WatchDog Monitor Heartbeat
 * A2   = Wind Speed IRQ
 * A3   = Not in use
 * A4   = Distance Sensor
 * A5   = OPTION 2 Pin - Do Actions: OP2RAW, OP2VBV, OP2CLR 
 * D13  = SPIO CLK   SD Card
 * D12  = SPI0 MOSI  SD Card
 * D11  = SPI0 MISO  SD Card
 * D10  = UART1 RX - Future Reserved for LoRa CS
 * D9   = UART1 TX - Future Reserved for LoRa RESET
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
 * Where AuthType is one of these keywords (WEP WPA WPA2 UNSEC)
 * Blank password is supported for UNSEC
 * ======================================================================================================================
 */

/* 
 *=======================================================================================================================
 * Local Includes
 *=======================================================================================================================
 */
#include "include/ssbits.h"         // System Status Bits
#include "include/qc.h"             // Quality Control Min and Max Sensor Values on Surface of the Earth
#include "include/support.h"        // Support Functions
#include "include/sdcard.h"         // SD Card Functions
#include "include/cf.h"             // Configuration File Variables
#include "include/eeprom.h"         // EEPROM Functions
#include "include/output.h"         // Serial and OLED Output Functions
#include "include/wrda.h"           // Wind Rain Distance Air Functions
#include "include/time.h"           // Time Management Functions
#include "include/ps.h"             // Particle Support Functions
#include "include/sensors_i2c_44_47.h" // Handle i2c Sensors in this address range
#include "include/sensors.h"        // I2C Based Sensor Functions
#include "include/info.h"           // Station Information Functions
#include "include/statmon.h"        // Station Monitor Functions
#include "include/obs.h"            // Observation Functions
#include "include/main.h"

/*
 * ======================================================================================================================
 * Variables and Data Structures 
 * =======================================================================================================================
 */
char versioninfo[sizeof(VERSION_INFO)];  // allocate enough space including null terminator
char msgbuf[MAX_MSGBUF_SIZE]; // Used to hold messages
char *msgp;                   // Pointer to message text
char Buffer32Bytes[32];       // General storage
int  LED_PIN = D7;            // Built in LED
bool JustPoweredOn = true;         // Used to clear SystemStatusBits set during power on device discovery
uint64_t lastOBS = 0;         // time of next observation
int countdown = 1800;         // Exit calibration mode when reaches 0 - protects against burnt out pin or forgotten jumper
uint64_t LastTimeUpdate = 0;
int  cf_reboot_countdown_timer = 79200; // There is overhead transmitting data so take off 2 hours from 86400s                                        // Set to 0 to disable feature
int DailyRebootCountDownTimer;
bool SendSystemInformation = true; // Send System Information to Particle Cloud. True means we will send at boot.


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
 *   Report times 0,6,12,18,24,30,36,42,48,54. Add 90s after each to get below report times
 * ======================================================================================================================
 */
struct {
  int    sath[10]          = { 90, 450, 810, 1170, 1530, 1890, 2250, 2610, 2790, 3330};  // Seconds After The Hour
  time_t last_obs_time[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
} OBS_Period;

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
 * BackGroundWork() - Take Sensor Reading, Check LoRa for Messages, Delay 1 Second for use as timming delay            
 * ======================================================================================================================
 */
void BackGroundWork() {
  // Anything that needs sampling every second add below. Example Wind Speed and Direction, StreamGauge

  uint64_t OneSecondFromNow = System.millis() + 1000;

  if (DoWind) {
    Wind_TakeReading();
  }
  DistanceGauge_TakeReading();

  HeartBeat();  // Provides a 250ms delay

  int64_t TimeRemaining = (OneSecondFromNow - System.millis());
  if ((TimeRemaining > 0) && (TimeRemaining < 1000)) {
    delay (TimeRemaining);
  }
}

/*
 * ======================================================================================================================
 * OBS_TimeCheck() - See if it is time to make an observation
 * ======================================================================================================================
 */
bool OBS_TimeCheck() {
  int obs_period=9; // assume last period so we don't have to check roll over on the hour
  time_t current_time = Time.now();
  int sath = current_time % 3600; // Seconds After the Hour

  if (sath < 90) {
    obs_period = 9;  // before period 0 starts
  }
  else {
    obs_period= (sath - 90) / 360;   // 6 minutes = 360 seconds
                                     // Subtract 90s from Seconds After the Hour, then we can divid by 6 minutes
                                     // with a int result giving us the period
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
// SYSTEM_THREAD(ENABLED); // Default Behavior as of 6.2.0

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
  strcpy(versioninfo, VERSION_INFO);
  Output (versioninfo);

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

  // If config file exists it is opened and read
  SD_ReadConfigFile();

  // If elevation file exists it is opened, read and elevation set
  SD_ReadElevationFile();

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
  bmx_initialize();  // This needs to run before sensor_initialize_i2c_44_47() so we know 
                     // what obs tag name to assign to bmp581 if it exists.

  // Scan for sensors BMP581 SHT31 SHT45 HDC302x and initialize
  sensor_initialize_i2c_44_47();
 
  htu21d_initialize();
  mcp9808_initialize();
  si1145_initialize();
  vlx_initialize();

  if (SD.exists(SD_5M_DIST_FILE)) {
    dg_adjustment = 1.25;
    Output ("DIST=5M");
  }
  else {
    dg_adjustment = 2.5;
    Output ("DIST=10M");
  }

  OP2_Initialize(); // Check for files to determine OP2 Pin Configuration (Raw, Voltaic Voltage)
  

  CheckNoWindFile(); // if NOWIND.TXT found then DoWind is set false
  if (DoWind) {
    // Optipolar Hall Effect Sensor SS451A - Wind Speed
    pinMode(ANEMOMETER_IRQ_PIN, INPUT);
    anemometer_interrupt_count = 0;
    anemometer_interrupt_stime = System.millis();
    attachInterrupt(ANEMOMETER_IRQ_PIN, anemometer_interrupt_handler, FALLING);
  }

  as5600_initialize(); // Still check for this sensor even if we have NOWIND set.

  // Derived Observations
  hi_initialize();
  mslp_initialize();
 
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

#if (PLATFORM_ID == PLATFORM_BORON) || (PLATFORM_ID == PLATFORM_MSOM)
  // Get International Mobile Subscriber Identity
  if ((RESP_OK == Cellular.command(callback_imsi, imsi, 10000, "AT+CIMI\r\n")) && (strcmp(imsi,"") != 0)) {
    sprintf (msgbuf, "IMSI:%s", imsi);
    Output (msgbuf);
  }
  else {
    Output("IMSI:NF");
  }
#endif

  WindAndDistance_Fill();
}

/*
 * ======================================================================================================================
 * loop() runs over and over again, as quickly as it can execute.
 * ======================================================================================================================
 */
void loop() {
  [[maybe_unused]] uint64_t LastConnectCheckTime = 0;
  static bool FirstParticleConnect = true;

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

      if (SendSystemInformation && Particle.connected()) {
        INFO_Do(); // Function sets SendSystemInformation back to false.
      }

      // See if Time to Make Observation
      if (OBS_TimeCheck()) {
        // We take obs 90s afer each observertion period. doe for 0m->0m+90s 1m30s, 6m->6m+90s 7m30s... 
        // The distance buffer is 181 buckets.  We sample every second.

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
      WindAndDistance_Fill();
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

      if (Particle.connected()) {
        INFO_Do(); // Function sets SendSystemInformation back to false.
      }
      
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

      WindAndDistance_Fill();
    } // powerdown
#endif
  } // normal operation

  BackGroundWork();
} // loop
