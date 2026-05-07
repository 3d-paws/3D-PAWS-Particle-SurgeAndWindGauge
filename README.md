# Storm Surge Wind Gauge (SSWG)

### Storm Surge

The station used the below storm surge calculation.

#### 3 minute water level average
181 one-second water level samples are centered at (0,6,12,18,24,30,36,42,48,54). These samples are averaged, 
a three standard deviation outlier rejection test applied, 
the mean and standard deviation are recalculated and reported along with the number of outliers.

We can use the 181 observations to capture the mean and standard deviation and then multiply the value by 3. 

Reference:  https://tidesandcurrents.noaa.gov/publications/CO-OPS_Measurement_Spec.pdf

Add 90 seconds after each (0,6,12,18,24,30,36,42,48,54) sample period for the reported time.

#### To calculate the standard deviation of those numbers:

 - Work out the Mean (the simple average of the numbers)
 - Then for each number: subtract the Mean and square the result.
 - Then work out the mean of those squared differences.
 - Take the square root of that and we are done!

#### A sigma value is a statistical term otherwise known as a standard deviation. The three-sigma process:

 - 1 Calculate the mean
 - 2 Calculate the standard deviation
 - 3 Multiply the standard deviation by 3
 - 4 Subtract the product in step 3 from the mean

#### Distance Sensors

 - The 5-meter sensors (MB7360, MB7369, MB7380, and MB7389) use a scale factor of (Vcc/5120) per 1-mm. Particle 12bit resolution (0-4095),  Sensor has a resolution of 0 - 5119mm,  Each unit of the 0-4095 resolution is 1.25mm

 - The 10-meter sensors (MB7363, MB7366, MB7383, and MB7386) use a scale factor of (Vcc/10240) per 1-mm. Particle 12bit resolution (0-4095),  Sensor has a resolution of 0 - 10239mm,  Each unit of the 0-4095 resolution is 2.5mm

### Wind Support

Wind is enabled by default. Meaning you should add a AS5600 for direction readings. Or disable able wind with NOWIND.TXT file.

### Sensor Support
| Sensor Model                  | Type                    | Description                                                                                              | Company URL                             |
|-------------------------------|-------------------------|----------------------------------------------------------------------------------------------------------|------------------------------------------|
| Adafruit HTU21DF              | Temp/Humidity           | Digital I2C sensor, ±2% RH, ±0.3°C, compact, low power.                                                  | [Adafruit HTU21DF](https://www.adafruit.com/product/1899)   |
| Adafruit MCP9808              | Temperature             | High-precision digital temp sensor, ±0.25°C, 2.7-5.5V, I2C.                                              | [Adafruit MCP9808](https://www.adafruit.com/product/1782)   |
| Adafruit BME280               | Temp/Humidity/Pressure  | Measures temp, humidity, pressure, I2C/SPI, -40 to +85°C, 0-100% RH, 1.8-5V.                             | [Adafruit BME280](https://www.adafruit.com/product/2652)    |
| Adafruit BMP280               | Temp/Pressure           | Measures pressure (300–1100 hPa), temperature, I2C/SPI, small, low-power.                                | [Adafruit BMP280](https://www.adafruit.com/product/2651)    |
| Adafruit BMP3XX               | Temp/Pressure           | Gen3 barometric, high-precision, ±0.5m alt., I2C/SPI.                                                    | [BMP388/BMP3XX](https://shop.pimoroni.com/en-us/products/adafruit-bmp388-precision-barometric-pressure-and-altimeter) |
| Adafruit_BMP5xx               | Temp/Pressure           | BMP5xx series (BMP581) is a high-precision barometric pressure sensors from Bosch Sensortec | [Adafruit BMP5xx](hhttps://www.adafruit.com/product/6407) |
| Adafruit SHT31                | Temp/Humidity           | High accuracy, ±2% RH, ±0.3°C, I2C, fast response, 2.4–5.5V.                                             | [Adafruit SHT31](https://www.adafruit.com/product/2857)     |
| Adafruit_SHT4x                | Temp/Humidity           | Adafruit Sensirion SHT45 Precision Temp & Humidity with PTFE                                | [Adafruit SHT45](https://www.adafruit.com/product/5665)     |
| Adafruit PM25AQI              | PM2.5 Air Quality       | Measures particulate (0.3–10um), I2C (fixed 0x12), 0-500μg/m³ range, 5V.                                 | [PM25AQI](https://www.adafruit.com/product/4632)            |
| AS5600L                       | Magnetic Encoder        | Same as AS5600, but designed for integration in new designs (ex: Particle Muon).                         | [SS451A Datasheet](https://sensing.honeywell.com)           |
| SS451A (Anemometer)           | Hall Effect Switch      | Used as wind anemometer sensor (rotation detection).                                                     | [TMP112A Sensor](https://www.ti.com/product/TMP112A)        |
| Adafruit VEML7700             | Ambient Light           | Precise lux (0–120klux), I2C, auto-range, digital output, 3.3/5V.                                        | [Adafruit VEML7700](https://www.adafruit.com/product/4162)  |
| Adafruit SI1145               | Light/UV/Proximity      | I2C, measures visible, IR, and digital UV index, 3-5V.                                                   | [Adafruit SI1145](https://www.adafruit.com/product/1777)    |
| MB7360, MB7369, MB7380, MB7389| 5m Distance             | Maxbotix Ultrasonic Distance Sensor                                                                      | [Maxbotix](https://maxbotix.com/products)        |
| MB7363, MB7366, MB7383, MB7386| 10m Distance            | Maxbotix Ultrasonic Distance Sensor                                                                      | [Maxbotix](https://maxbotix.com/products)        |

### Derived Observations
- Heat Index (hi) is reported if sensor SHT_1 exists.
- Mean Sea Level Pressure (mlsp) if station elevation is set "ELEV.TXT" and station is configured with a SHT and BMP sensors.

### Sensor Tag Name Assignments for BMPxxx, BMExxx SHT31, SHT45, HDC302x
These sensors have multiple address options.
<pre>
bmx  0x77 BMP280, BME280, BMP388, BMP390    Default Address for the BMP
bmx  0x76 BMP280, BME280, BMP388, BMP390    Default Address for the BME

sht3 0x44 Default
sht3 0x45 with jumper  

sht4 0x44 Default and only address

b58  0x47 default      BMP581
b58  0x46 with jumper  BMP581
</pre>
When multiple sensor combinations are used with dynamic discovery, tag name assignment can become complex. The following rules define how tag names are assigned.

For BMP280, BME280, BMP388, and BMP390 sensors, detection is prioritized first. If one of these sensors is found at I2C address 0x77, it is assigned the tag bx1 (where x represents p, t, or h). If found at address 0x76, it is assigned bx2.

For the BMP581 sensor, assignment depends on availability. If a BMP581 is found at address 0x46 and bx1 is not already assigned, it becomes bx1. Otherwise, if bx2 is not assigned, it becomes bx2. If both bx1 and bx2 are already assigned, it becomes bx3. The same logic applies to address 0x47, with the next available tag being bx4 if bx3 is already in use.

For SHT sensors (SHT31 or SHT45), assignment depends on address priority. If a sensor is found at address 0x44, it is assigned st1 and sh1. If no sensor is present at 0x44 but an SHT31 is found at 0x45, it is assigned st1 and sh1. If a sensor is already present at 0x44, then an SHT31 at 0x45 is assigned st2 and sh2.

### DoAction Functions on Particle Cloud Console

On the view device screen on the Particle Console there is a FUNCTIONS area at the lower right. This is used to send commands to the online device.

- REBOOT - Reboot device. Toggle pin A0. If no relay/watchdog is connected to A0, then a soft boot on the Particle board is performed.
- INFO - Trigger station to send station information                . Event type "INFO".
- 5MDIST - Configure 5m Sensor. Creates file 5MDIST.TXT. Value read from pin A4 is multiplied by 1.25mm.
- 10MDIST - Configure 10m Sensor. Removes file 5MDIST.TXT. Value read from pin A4 is multiplied by 2.5mm. (Default)
- SETELEV:xxxx - Set station elevation. Replace xxxx with elevation in meters. Creates file ELEV.TXT
- NOWIND - Disable wind. Created file NOWIND.TXT.
- DOWIND - Removes file NOWIND.TXT if it exists, you should rebot after this.
- OP2RAW - Enable OP2 pin for raw readings. Creates OP2RAW.TXT.
- OP2VBV - Enable OP2 pin for reading Voltaic Battery Voltage.  Creates OP2VBV.TXT.
- OP2CLR - Disable OP2 pin configurations. Removes OP2RAW.TXT OP2VBV.TXT files.

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
| `/NOWIND.TXT`     | Disable wind.                                                                           |
| `/OP2RAW.TXT`     | Configures OP2 for raw readings (avg. 5 samples spaced 10ms).|
| `/OP2VBV.TXT`     | Configures OP2 for reading Voltaic Battery Voltage.|

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
|A5   | OPTION 2 Pin - Do Actions: OP2RAW, OP2VBV, OP2CLR |
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

### Particle Publish Event Name "SS" (Storm Surge)
<div style="overflow:auto; white-space:pre; font-family: monospace; font-size: 8px; line-height: 1.5; height: 650px; border: 1px solid black; padding: 10px;">
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
bp1    BME280,BMP280,BMP3XX,BMP4XX Pressure
bt1    BME280,BMP280,BMP3XX,BMP4XX Temperature
bh1    BME280,BMP280,BMP3XX,BMP4XX Humidity
bp2    BME280,BMP280,BMP3XX,BMP4XX Pressure
bt2    BME280,BMP280,BMP3XX,BMP4XX Temperature
bh2    BME280,BMP280,BMP3XX,BMP4XX Humidity
bp3    BMP4XX Pressure
bt3    BMP4XX Temperature
bh3    BMP4XX Humidity
bp4    BMP4XX Pressure
bt4    BMP4XX Temperature
bh4    BMP4XX Humidity
hh1    htu_humid
ht1    htu_temp
sh1    sht_humid
st1    sht_temp
sh2    sht_humid
st2    sht_temp
mt1    mcp_temp
mt2    mcp_temp
hi     heat index
sv1    SI1145 VIS
si1    SI1145 IR
su1    SI1145 UV
vlx    VEML7700 LUX
mlsp   mean sea level pressure
vbv    voltaic battery voltage
vpc    voltaic percent charge
op2r   option 2 raw reading
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
# Serial Monitor

Adding a jumper wire between Particle pin D8 (Boron & Argon), A2 (Muon) and ground will enable serial text output on the USB port at boot time.

A serial monitor from Arduino's IDE can be used. TIP: With the Arduino serial monitor. Select "Both NL & CR" on the pull down menu at the bottom.  On a Mac with Visual Studio installed with Particle's Development Environment; the shell command "particle serial monitor" can be used.

Upon Particle board boot with the jumper wire connected, software will wait 60 seconds for you to connect the serial monitor. Flashing the board led.  After 60 seconds the software will continue the boot process. Below is an example of what you might see as the software initializes and discovers connected devices.

If the jumper remains connected after boot. You will enter the Station Monitor. The station monitor will run for 30 minutes. Removing the jumper will exit you from the station monitor.



### Station Information - (Particle Message type "INFO")
At boot the station will send a event message of type "INFO" to Particle. This message contains configuration and status information. You can also request an INFO event message to be sent from the device via DoAction Function on the Particle Console. Use keyword "INFO". Every INFO execution will create/rewrite file INFO.TXT with INFO information. The information reported will vary based on the board type (Boron, Argon, Muon). Below are examples.

INFO event message from a Boron
<div style="overflow:auto; white-space:pre; font-family: monospace; font-size: 8px; line-height: 1.5; height: 425px; border: 1px solid black; padding: 10px;">
<pre>
{
"devid":"e00fce68bde8f63590a3b118"
"devos":"6.4.1"
"freemem":82440
"uptime":143
"board":"boron"
"at":"2026-05-07T21:39:24"
"ver":"SSWG-20260507v11"
"hth":1
"rr":"120-0"
"obsi":"6m"
"obsti":"6m"
"drct":79200
"n2s":"NF"
"ps":"USB_HOST"
"bcs":"MISSING"
"bpc":-1
"css":52.4987
"csq":12.4987
"imsi":"234103519249568"
"actsim":"INTERNAL"
"op1":"DIST 10M(a4)"
"op2":"VBV(A5)"
"elev":1560
"sensors":"BMX1(BMP390),SHT45(44-E1856B9),SHT31(45-118DF311),BMP581(47),MCP1,AS5600,HI,MSLP,WS(A2)"
"rtc":"OK"
"oled":"32"
"scepin":"DISABLED"
"sce":"TRUE"
}
</pre>
</div>

### WatchDog Board Support
The WatchDog is a external device that can turn off power to your weather station's microcontroller and sensors. Power remains off for 10 seconds. Ways the WatchDog can be triggered to turn off power
- If the station microcontroller's heartbeat pulse stops for 5 minutes.
- If a trigger pulse is sent from the weather station's microcontroller to the WatchDog.
- Weather station software does a daily reboot and will send a trigger pulse.
- Particle Console for this device can send a DoAction function "REBOOT" down to the weather station's microcontroller. To send the trigger pulse.

For more information see GutHub Site [3d-paws/3D-PAWS-WatchDog](https://github.com/3d-paws/3D-PAWS-WatchDog/blob/master/README.md).