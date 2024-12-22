/*
 * ======================================================================================================================
 *  Sensors.h
 * ======================================================================================================================
 */

/*
 * ======================================================================================================================
 *  BMX280 humidity - I2C - Temperature, pressure sensor & altitude - Support 2 of any combination
 * 
 *  https://www.asknumbers.com/PressureConversion.aspx
 *  Pressure is returned in the SI units of Pascals. 100 Pascals = 1 hPa = 1 millibar. 
 *  Often times barometric pressure is reported in millibar or inches-mercury. 
 *  For future reference 1 pascal = 0.000295333727 inches of mercury, or 1 inch Hg = 3386.39 Pascal. 
 *
 *  Looks like you divide by 100 and you get millibars which matches NWS page
 * 
 *  Surface Observations and Station Elevation 
 *  https://forecast.weather.gov/product.php?issuedby=BOU&product=OSO&site=bou 
 * 
 * Forced mode
 * In forced mode, a single measurement is performed according to selected measurement and filter options. 
 * When the measurement is finished, the sensor returns to sleep mode and the measurement results can be obtained 
 * from the data registers. For a next measurement, forced mode needs to be selected again. 
 * ======================================================================================================================
 */
// #define BMX_STATION_ELEVATION 1017.272  // default 1013.25
#define BMX_ADDRESS_1         0x77  // BMP Default Address - Connecting SDO to GND will change BMP to 0x76
#define BMX_ADDRESS_2         0x76  // BME Default Address - Connecting SDO to GND will change BME to 0x77
#define BMP280_CHIP_ID        0x58
#define BME280_BMP390_CHIP_ID 0x60
#define BMP388_CHIP_ID        0x50
#define BMX_TYPE_UNKNOWN      0
#define BMX_TYPE_BMP280       1
#define BMX_TYPE_BME280       2
#define BMX_TYPE_BMP388       3
#define BMX_TYPE_BMP390       4
Adafruit_BMP280 bmp1;
Adafruit_BMP280 bmp2;
Adafruit_BME280 bme1;
Adafruit_BME280 bme2;
Adafruit_BMP3XX bm31;
Adafruit_BMP3XX bm32;
byte BMX_1_chip_id = 0x00;
byte BMX_2_chip_id = 0x00;
bool BMX_1_exists = false;
bool BMX_2_exists = false;
byte BMX_1_type=BMX_TYPE_UNKNOWN;
byte BMX_2_type=BMX_TYPE_UNKNOWN;
const char *bmxtype[] = {"UNKN", "BMP280", "BME280", "BMP388", "BMP390"};

/*
 * ======================================================================================================================
 *  HTU21D-F - I2C - Humidity & Temp Sensor
 * ======================================================================================================================
 */
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
bool HTU21DF_exists = false;

/*
 * ======================================================================================================================
 *  MCP9808 - I2C - Temperature sensor
 * 
 * I2C Address is:  0011,A2,A1,A0
 *                  0011000 = 0x18  where A2,1,0 = 0 MCP9808_I2CADDR_DEFAULT  
 *                  0011001 = 0x19  where A0 = 1
 * ======================================================================================================================
 */
#define MCP_ADDRESS_1     0x18
#define MCP_ADDRESS_2     0x19        // A0 set high, VDD
Adafruit_MCP9808 mcp1;
Adafruit_MCP9808 mcp2;
bool MCP_1_exists = false;
bool MCP_2_exists = false;

/*
 * ======================================================================================================================
 *  SHTX - I2C - Temperature & Humidity sensor (SHT31)  - Note the SHT40, SHT45 use same i2c address
 * ======================================================================================================================
 */
#define SHT_ADDRESS_1     0x44
#define SHT_ADDRESS_2     0x45        // ADR pin set high, VDD
Adafruit_SHT31 sht1;
Adafruit_SHT31 sht2;
bool SHT_1_exists = false;
bool SHT_2_exists = false;

/*
 * ======================================================================================================================
 *  HIH8 - I2C - Temperature & Humidity sensor (HIH8000)  - 
 * ======================================================================================================================
 */
#define HIH8000_ADDRESS   0x27
bool HIH8_exists = false;

/*
 * ======================================================================================================================
 *  Si1145 - I2C - UV/IR/Visible Light Sensor
 *  The SI1145 has a fixed I2C address (0x60), you can only connect one sensor per microcontroller!
 * ======================================================================================================================
 */
Adafruit_SI1145 uv = Adafruit_SI1145();
bool SI1145_exists = false;
// When we do a read of all three and we get zeros. If these llast readings are not zero, we will reinitialize the
// chip. When does a reset on it and then read again.
float si_last_vis = 0.0;
float si_last_ir = 0.0;
float si_last_uv = 0.0;

/*
 * ======================================================================================================================
 *  VEML7700 - I2C - Lux Sensor
 * ======================================================================================================================
 */
#define VEML7700_ADDRESS   0x10
Adafruit_VEML7700 veml = Adafruit_VEML7700();
bool VEML7700_exists = false;

/* 
 *=======================================================================================================================
 * get_Bosch_ChipID ()  -  Return what Bosch chip is at specified address
 *   Chip ID BMP280 = 0x58 temp, preasure           - I2C ADDRESS 0x77  (SD0 to GND = 0x76)  
 *   Chip ID BME280 = 0x60 temp, preasure, humidity - I2C ADDRESS 0x77  (SD0 to GND = 0x76)  Register 0xE0 = Reset
 *   Chip ID BMP388 = 0x50 temp, preasure           - I2C ADDRESS 0x77  (SD0 to GND = 0x76)
 *   Chip ID BMP390 = 0x60 temp, preasure           - I2C ADDRESS 0x77  (SD0 to GND = 0x76)
 *=======================================================================================================================
 */
byte get_Bosch_ChipID (byte address) {
  byte chip_id = 0;
  byte error;

  Output ("get_Bosch_ChipID()");
  // The i2c_scanner uses the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.

  // Important! Need to check the 0x00 register first. Doing a 0x0D (not chip id loaction) on a bmp388 
  // will return a value that could match one of the IDs 

  // Check Register 0x00
  sprintf (msgbuf, "  I2C:%02X Reg:%02X", address, 0x00);
  Output (msgbuf);
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(0x00);  // BM3 CHIPID REGISTER
  error = Wire.endTransmission();
    //  0:success
    //  1:data too long to fit in transmit buffer
    //  2:received NACK on transmit of address
    //  3:received NACK on transmit of data
    //  4:other error 
  if (error) {
    sprintf (msgbuf, "  ERR_ET:%d", error);
    Output (msgbuf);
  }
  else if (Wire.requestFrom(address, 1)) {  // Returns the number of bytes returned from the slave device 
    chip_id = Wire.read();
    if (chip_id == BMP280_CHIP_ID) { // 0x58
      sprintf (msgbuf, "  CHIPID:%02X BMP280", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!
    }
    else if (chip_id == BMP388_CHIP_ID) {  // 0x50
      sprintf (msgbuf, "  CHIPID:%02X BMP388", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!   
    }
    else if (chip_id == BME280_BMP390_CHIP_ID) {  // 0x60
      sprintf (msgbuf, "  CHIPID:%02X BME/390", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!   
    }
    else {
      sprintf (msgbuf, "  CHIPID:%02X InValid", chip_id);
      Output (msgbuf);      
    }
  }
  else {
    sprintf (msgbuf, "  ERR_RF:0");
    Output (msgbuf);
  }

  // Check Register 0xD0
  chip_id = 0;
  sprintf (msgbuf, "  I2C:%02X Reg:%02X", address, 0xD0);
  Output (msgbuf);
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(0xD0);  // BM2 CHIPID REGISTER
  error = Wire.endTransmission();
    //  0:success
    //  1:data too long to fit in transmit buffer
    //  2:received NACK on transmit of address
    //  3:received NACK on transmit of data
    //  4:other error 
  if (error) {
    sprintf (msgbuf, "  ERR_ET:%d", error);
    Output (msgbuf);
  }
  else if (Wire.requestFrom(address, 1)) {  // Returns the number of bytes returned from the slave device 
    chip_id = Wire.read(); 
    if (chip_id == BMP280_CHIP_ID) { // 0x58
      sprintf (msgbuf, "  CHIPID:%02X BMP280", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!
    }
    else if (chip_id == BMP388_CHIP_ID) {  // 0x50
      sprintf (msgbuf, "  CHIPID:%02X BMP388", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!   
    }
    else if (chip_id == BME280_BMP390_CHIP_ID) {  // 0x60
      sprintf (msgbuf, "  CHIPID:%02X BME/390", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!   
    }
    else {
      sprintf (msgbuf, "  CHIPID:%02X InValid", chip_id);
      Output (msgbuf);   
    }
  }
  else {
    sprintf (msgbuf, "  ERR_RF:0");
    Output (msgbuf);
  }
  return(0);
}

/* 
 *=======================================================================================================================
 * bmx_initialize() - Bosch sensor initialize
 *=======================================================================================================================
 */
void bmx_initialize() {
  Output("BMX:INIT");
  
  // 1st Bosch Sensor - Need to see which (BMP, BME, BM3) is plugged in
  BMX_1_chip_id = get_Bosch_ChipID(BMX_ADDRESS_1);

  switch (BMX_1_chip_id) {
    case BMP280_CHIP_ID :
      if (!bmp1.begin(BMX_ADDRESS_1)) { 
        msgp = (char *) "BMP1 ERR";
        BMX_1_exists = false;
        SystemStatusBits |= SSB_BMX_1;  // Turn On Bit          
      }
      else {
        BMX_1_exists = true;
        BMX_1_type = BMX_TYPE_BMP280;
        msgp = (char *) "BMP1 OK";
        float p = bmp1.readPressure();
      }
    break;

    case BME280_BMP390_CHIP_ID :
      if (!bme1.begin(BMX_ADDRESS_1)) { 
        if (!bm31.begin_I2C(BMX_ADDRESS_1)) {  // Perhaps it is a BMP390
          msgp = (char *) "BMX1 ERR";
          BMX_1_exists = false;
          SystemStatusBits |= SSB_BMX_1;  // Turn On Bit          
        }
        else {
          BMX_1_exists = true;
          BMX_1_type = BMX_TYPE_BMP390;
          msgp = (char *) "BMP390_1 OK"; 
          float p = bm31.readPressure();       
        }      
      }
      else {
        BMX_1_exists = true;
        BMX_1_type = BMX_TYPE_BME280;
        msgp = (char *) "BME280_1 OK";
        float p = bme1.readPressure();
      }
    break;

    case BMP388_CHIP_ID :
      if (!bm31.begin_I2C(BMX_ADDRESS_1)) { 
        msgp = (char *) "BM31 ERR";
        BMX_1_exists = false;
        SystemStatusBits |= SSB_BMX_1;  // Turn On Bit          
      }
      else {
        BMX_1_exists = true;
        BMX_1_type = BMX_TYPE_BMP388;
        msgp = (char *) "BM31 OK";
        float p = bm31.readPressure();
      }
    break;

    default:
      msgp = (char *) "BMX_1 NF";
    break;
  }
  Output (msgp);

  // 2nd Bosch Sensor - Need to see which (BMP, BME, BM3) is plugged in
  BMX_2_chip_id = get_Bosch_ChipID(BMX_ADDRESS_2);
  switch (BMX_2_chip_id) {
    case BMP280_CHIP_ID :
      if (!bmp2.begin(BMX_ADDRESS_2)) { 
        msgp = (char *) "BMP2 ERR";
        BMX_2_exists = false;
        SystemStatusBits |= SSB_BMX_2;  // Turn On Bit          
      }
      else {
        BMX_2_exists = true;
        BMX_2_type = BMX_TYPE_BMP280;
        msgp = (char *) "BMP2 OK";
        float p = bmp2.readPressure();
      }
    break;

    case BME280_BMP390_CHIP_ID :
      if (!bme2.begin(BMX_ADDRESS_2)) { 
        if (!bm32.begin_I2C(BMX_ADDRESS_2)) {  // Perhaps it is a BMP390
          msgp = (char *) "BMX2 ERR";
          BMX_2_exists = false;
          SystemStatusBits |= SSB_BMX_2;  // Turn On Bit          
        }
        else {
          BMX_2_exists = true;
          BMX_2_type = BMX_TYPE_BMP390;
          msgp = (char *) "BMP390_2 OK"; 
          float p = bm32.readPressure();         
        }
      }
      else {
        BMX_2_exists = true;
        BMX_2_type = BMX_TYPE_BME280;
        msgp = (char *) "BME280_2 OK";
        float p = bme2.readPressure();
      }
    break;

    case BMP388_CHIP_ID :
      if (!bm32.begin_I2C(BMX_ADDRESS_2)) { 
        msgp = (char *) "BM32 ERR";
        BMX_2_exists = false;
        SystemStatusBits |= SSB_BMX_2;  // Turn On Bit          
      }
      else {
        BMX_2_exists = true;
        BMX_2_type = BMX_TYPE_BMP388;
        msgp = (char *) "BM32 OK";
        float p = bm32.readPressure();
      }
    break;

    default:
      msgp = (char *) "BMX_2 NF";
    break;
  }
  Output (msgp);
}

/* 
 *=======================================================================================================================
 * htu21d_initialize() - HTU21D sensor initialize
 *=======================================================================================================================
 */
void htu21d_initialize() {
  Output("HTU21D:INIT");
  
  // HTU21DF Humidity & Temp Sensor (I2C ADDRESS = 0x40)
  if (!htu.begin()) {
    msgp = (char *) "HTU NF";
    HTU21DF_exists = false;
    SystemStatusBits |= SSB_HTU21DF;  // Turn On Bit
  }
  else {
    HTU21DF_exists = true;
    msgp = (char *) "HTU OK";
  }
  Output (msgp);
}

/* 
 *=======================================================================================================================
 * mcp9808_initialize() - MCP9808 sensor initialize
 *=======================================================================================================================
 */
void mcp9808_initialize() {
  Output("MCP9808:INIT");
  
  // 1st MCP9808 Precision I2C Temperature Sensor (I2C ADDRESS = 0x18)
  mcp1 = Adafruit_MCP9808();
  if (!mcp1.begin(MCP_ADDRESS_1)) {
    msgp = (char *) "MCP1 NF";
    MCP_1_exists = false;
    SystemStatusBits |= SSB_MCP_1;  // Turn On Bit
  }
  else {
    MCP_1_exists = true;
    msgp = (char *) "MCP1 OK";
  }
  Output (msgp);

  // 2nd MCP9808 Precision I2C Temperature Sensor (I2C ADDRESS = 0x19)
  mcp2 = Adafruit_MCP9808();
  if (!mcp2.begin(MCP_ADDRESS_2)) {
    msgp = (char *) "MCP2 NF";
    MCP_2_exists = false;
    SystemStatusBits |= SSB_MCP_2;  // Turn On Bit
  }
  else {
    MCP_2_exists = true;
    msgp = (char *) "MCP2 OK";
  }
  Output (msgp);
}

/* 
 *=======================================================================================================================
 * sht_initialize() - SHT31 sensor initialize
 *=======================================================================================================================
 */
void sht_initialize() {
  Output("SHT:INIT");
  
  // 1st SHT31 I2C Temperature/Humidity Sensor (I2C ADDRESS = 0x44)
  sht1 = Adafruit_SHT31();
  if (!sht1.begin(SHT_ADDRESS_1)) {
    msgp = (char *) "SHT1 NF";
    SHT_1_exists = false;
    SystemStatusBits |= SSB_SHT_1;  // Turn On Bit
  }
  else {
    SHT_1_exists = true;
    msgp = (char *) "SHT1 OK";
  }
  Output (msgp);

  // 2nd SHT31 I2C Temperature/Humidity Sensor (I2C ADDRESS = 0x45)
  sht2 = Adafruit_SHT31();
  if (!sht2.begin(SHT_ADDRESS_2)) {
    msgp = (char *) "SHT2 NF";
    SHT_2_exists = false;
    SystemStatusBits |= SSB_SHT_2;  // Turn On Bit
  }
  else {
    SHT_2_exists = true;
    msgp = (char *) "SHT2 OK";
  }
  Output (msgp);
}

/* 
 *=======================================================================================================================
 * hih8_initialize() - HIH8000 sensor initialize
 *=======================================================================================================================
 */
void hih8_initialize() {
  Output("HIH8:INIT");

  if (I2C_Device_Exist(HIH8000_ADDRESS)) {
    HIH8_exists = true;
    msgp = (char *) "HIH8 OK";
  }
  else {
    msgp = (char *) "HIH8 NF";
    HIH8_exists = false;
    SystemStatusBits |= SSB_HIH8;  // Turn On Bit
  }
  Output (msgp);
}

/* 
 *=======================================================================================================================
 * hih8_getTempHumid() - Get Temp and Humidity
 *   Call example:  status = hih8_getTempHumid(&t, &h);
 *=======================================================================================================================
 */
bool hih8_getTempHumid(float *t, float *h) {
  if (HIH8_exists) {
    uint16_t humidityBuffer    = 0;
    uint16_t temperatureBuffer = 0;
  
    Wire.begin();
    Wire.beginTransmission(HIH8000_ADDRESS);

    Wire.write(0x00); // set the register location for read request

    delayMicroseconds(200); // give some time for sensor to process request

    if (Wire.requestFrom(HIH8000_ADDRESS, 4) == 4) {

      // Get raw humidity data
      humidityBuffer = Wire.read();
      humidityBuffer <<= 8;
      humidityBuffer |= Wire.read();
      humidityBuffer &= 0x3FFF;   // 14bit value, get rid of the upper 2 status bits

      // Get raw temperature data
      temperatureBuffer = Wire.read();
      temperatureBuffer <<= 8;
      temperatureBuffer |= Wire.read();
      temperatureBuffer >>= 2;  // Remove the last two "Do Not Care" bits (shift left is same as divide by 4)

      Wire.endTransmission();

      *h = humidityBuffer * 6.10e-3;
      *t = temperatureBuffer * 1.007e-2 - 40.0;

      // QC Check
      *h = (isnan(*h) || (*h < QC_MIN_RH) || (*h >QC_MAX_RH)) ? QC_ERR_RH : *h;
      *t = (isnan(*t) || (*t < QC_MIN_T)  || (*t >QC_MAX_T))  ? QC_ERR_T  : *t;
      return (true);
    }
    else {
      Wire.endTransmission();
      return(false);
    }
  }
  else {
    return (false);
  }
}

/* 
 *=======================================================================================================================
 * si1145_initialize() - SI1145 sensor initialize
 *=======================================================================================================================
 */
void si1145_initialize() {
  Output("SI1145:INIT");
  
  // SSB_SI1145 UV index & IR & Visible Sensor (I2C ADDRESS = 0x60)
  if (! uv.begin(&Wire)) {
    Output ("SI:NF");
    SI1145_exists = false;
    SystemStatusBits |= SSB_SI1145;  // Turn On Bit
  }
  else {
    SI1145_exists = true;
    Output ("SI:OK");
    si_last_vis = uv.readVisible();
    si_last_ir = uv.readIR();
    si_last_uv = uv.readUV()/100.0;

    sprintf (msgbuf, "SI:VI[%d.%02d]", (int)si_last_vis, (int)(si_last_vis*100.0)%100); 
    Output (msgbuf);
    sprintf (msgbuf, "SI:IR[%d.%02d]", (int)si_last_ir, (int)(si_last_ir*100.0)%100); 
    Output (msgbuf);
    sprintf (msgbuf, "SI:UV[%d.%02d]", (int)si_last_uv, (int)(si_last_uv*100.0)%100); 
    Output (msgbuf);
  }
}

/* 
 *=======================================================================================================================
 * lux_initialize() - VEML7700 sensor initialize
 * 
 * SEE https://learn.microsoft.com/en-us/windows/win32/sensorsapi/understanding-and-interpreting-lux-values
 * 
 * This data set is for illustration and may not be completely accurate for all users or situations.
 * 
 * Lighting condition     From (lux)     To (lux)     Mean value (lux)     Lighting step
 * Pitch Black            0              10           5                    1
 * Very Dark              11             50           30                   2
 * Dark Indoors           51             200          125                  3
 * Dim Indoors            201            400          300                  4
 * Normal Indoors         401            1000         700                  5
 * Bright Indoors         1001           5000         3000                 6
 * Dim Outdoors           5001           10,000       7500                 7
 * Cloudy Outdoors        10,001         30,000       20,000               8
 * Direct Sunlight        30,001         100,000      65,000               9
 * 
 * From www.vishay.com - Designing the VEML7700 Into an Application
 * 1    lx Full moon overhead at tropical latitudes
 * 3.4  lx Dark limit of civil twilight under a clear sky
 * 50   lx Family living room
 * 80   lx Hallway / bathroom
 * 100  lx Very dark overcast day
 * 320  lx to 500 lx Office lighting
 * 400  lx Sunrise or sunset on a clear day
 * 1000 lx Overcast day; typical TV studio lighting
 * 
 *=======================================================================================================================
 */
void lux_initialize() {
  Output("LUX:INIT");

  if (veml.begin()) {
    VEML7700_exists = true;
    msgp = (char *) "LUX OK";
  }
  else {
    msgp = (char *) "LUX NF";
    VEML7700_exists = false;
    SystemStatusBits |= SSB_LUX;  // Turn On Bit
  }
  Output (msgp);
}
