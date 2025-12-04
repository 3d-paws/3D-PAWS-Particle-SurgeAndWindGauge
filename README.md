# Storm Surge Wind Gauge (SSWG)

### Storm Surge Support

The station storm surge calculation is based on performing the following.

#### 3 minute water level average
91 one-second water level samples are taken at (0,6,12,18,24,30,36,42,48,54). These samples are averaged, 
a three standard deviation outlier rejection test applied, 
the mean and standard deviation are recalculated and reported along with the number of outliers.

We can use the 91 observations to capture the mean and standard deviation and then multiply the value by 3. 

Reference:  https://tidesandcurrents.noaa.gov/publications/CO-OPS_Measurement_Spec.pdf

Adding 91 seconds after each (0,6,12,18,24,30,36,42,48,54) sample period will be the report time.

#### To calculate the standard deviation of those numbers:

 * Work out the Mean (the simple average of the numbers)
 * Then for each number: subtract the Mean and square the result.
 * Then work out the mean of those squared differences.
 * Take the square root of that and we are done!

#### A sigma value is a statistical term otherwise known as a standard deviation. The three-sigma process:

 * 1 Calculate the mean
 * 2 Calculate the standard deviation
 * 3 Multiply the standard deviation by 3
 * 4 Subtract the product in step 3 from the mean

#### Distance Sensors

 * The 5-meter sensors (MB7360, MB7369, MB7380, and MB7389) use a scale factor of (Vcc/5120) per 1-mm. Particle 12bit resolution (0-4095),  Sensor has a resolution of 0 - 5119mm,  Each unit of the 0-4095 resolution is 1.25mm

 * The 10-meter sensors (MB7363, MB7366, MB7383, and MB7386) use a scale factor of (Vcc/10240) per 1-mm. Particle 12bit resolution (0-4095),  Sensor has a resolution of 0 - 10239mm,  Each unit of the 0-4095 resolution is 2.5mm

### Wind Support

If you add a AS5600 for Wind direction then the station will report wind.

### DoAction Functions on Particle Cloud Console

On the view device screen on the Particle Console there is a FUNCTIONS area at the lower right. This is used to send commands to the online device.

* REBOOT - Reboot device. Toggle pin A0. If no relay/watchdog is connected to A0, then a soft boot on the Particle board is performed.
* INFO - Trigger station to send station information. Event type "INFO".
* 5MDIST - Configure 5m Sensor. Creates file 5MDIST.TXT. Value read from pin A3 is multiplied by 1.25mm.
* 10MDIST - Configure 10m Sensor. Removes file 5MDIST.TXT. Value read from pin A3 is multiplied by 2.5mm. (Default)
* SETELEV:xxxx - Set station elevation. Replace xxxx with elevation in meters. Creates file ELEV.TXT

### File and Directory Overview

| File/Directory    | Purpose                                                                                 |
|-------------------|-----------------------------------------------------------------------------------------|
| `/OBS/`           | Directory containing observation files.                                                 |
| `/OBS/20251201.LOG` | Daily observation file in JSON format (one file per day).                             |
| `/N2SOBS.TXT`     | "Need to Send" file storing unsent observations. Resets if larger than specified size.  |
| `/SIM.TXT`        | Support file for third-party SIM configurations.                                        |
| `/WIFI.TXT`       | Stores WiFi configuration information for Argon WiFi and Muon boards.                   |
| `/INFO.TXT`       | Station info file. Overwritten with every INFO call.                                    |
| `/ELEV.TXT`       | Station elevation. File contents is elevation in meters.                                |

### Argon WiFi Support

At the top level of the SD card make a file called WIFI.TXT. Add one line to the file. This line has 3 items that are comma separated.
Example

 - AuthType,ssid,password
 
Where AuthType is one of these keywords (WEP WPA WPA2 UNSEC). A blank password is supported for UNSEC

### Boron 3rd Party Sim Support

At the top level of the SD card make a file called SIM.TXT. The first line of the file is read for the below patterns.

 - INTERNAL
 - AUP epc.tmobile.com username passwd
 - UP username password
 - APN epc.tmobile.com

After power on the file is read. The configuration is set in non volatile memory. Then the file is renamed to SIMOLD.TXT, so we don't do this on next boot. Reboot necessary.

### Argon and Boron Board Pin Mappings

| Pin  | Function / Usage                                                                 |
|------|----------------------------------------------------------------------------------|
|D8   | Serial Console (Ground Pin to Enable) - Not on Grove Shield |
|D7   | On Board LED - Blinks when console connection needed|
|D6   | Future Reserved for Lora IRQ - Not on Grove Shield|
|D5   | SD Card Chip Select|
|D4   | SPI1 MSIO - Future Reserved for LoRa|
|D3   | SPI1 MOSI - Future Reserved for LoRa|
|D2   | SPI1 SCK  - Future Reserved for LoRa|
|D1   | I2C SCL|
|D0   | I2C SDA|
|A0   | WatchDog Monitor/Relay Reset Trigger|
|A1   | WatchDog Monitor Heartbeat|
|A2   | Wind Speed IRQ|
|A3   | Not in use|
|A4   | Distance Sensor|
|A5   | Not in use |
|D13  | SPIO CLK   SD Card|
|D12  | SPI0 MOSI  SD Card|
|D11  | SPI0 MISO  SD Card|
|D10  | UART1 RX - Future Reserved for LoRa CS|
|D9   | UART1 TX - Future Reserved for LoRa RESET|


### System Health Bits
A register is maintained where its individual bits correspond to various state information and sensor state. This register is reported in each observation as a decimal number with tag id "hth". Many of the below bits will be set at initialization. Then cleared after the first observation is made after startup. 
<div style="overflow:auto; white-space:pre; font-family: monospace; font-size: 8px; line-height: 1.5; height: 120px; border: 1px solid black; padding: 10px;">
<pre>
#define SSB_PWRON           0x1       // Set at power on, but cleared after first observation
#define SSB_SD              0x2       // Set if SD missing at boot or other SD related issues
#define SSB_N2S             0x4       // Set when Need to Send observations exist
#define SSB_FROM_N2S        0x8       // Set in transmitted N2S observation when finally transmitted
#define SSB_RTC             0x10      // Set if RTC missing at boot
</pre>
</div>

### Particle Publish

<div style="overflow:auto; white-space:pre; font-family: monospace; font-size: 8px; line-height: 1.5; height: 500px; border: 1px solid black; padding: 10px;">
<pre>
Event Name: SS

Event Variables:
at     timestamp
wl     water_level
wld    water_level_stdev
wlo    water_level_outliers
wlm    water_level_mean
wlr    water_level_raw
ws     wind_speed
wd     wind_direction
wg     wind_gust
wgd    wind_gust_direction
bp1    bmx_pressure
bt1    bmx_temp
bh1    bmx_humid
bp2    bmx_pressure
bt2    bmx_temp
bh2    bmx_humid
hh1    htu_humid
ht1    htu_temp
sh1    sht_humid
st1    sht_temp
sh2    sht_humid
st2    sht_temp
mt1    mcp_temp
mt2    mcp_temp
hi     heat index
mlsp   mean sea level pressure
bcs    Battery Charger Status
bpc    Battery Percent Charge
cfr    Charger Fault Register
css    Cell Signal Strength
hth    Health 16bits - See System Status Bits in below define statements

Battery Charger Status
0 = BATTERY_STATE_UNKNOWN
1 = BATTERY_STATE_NOT_CHARGING
2 = BATTERY_STATE_CHARGING
3 = BATTERY_STATE_CHARGED
4 = BATTERY_STATE_DISCHARGING
5 = BATTERY_STATE_FAULT
6 = BATTERY_STATE_DISCONNECTED
</pre>
</div>