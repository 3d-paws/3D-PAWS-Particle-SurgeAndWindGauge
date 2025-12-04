/*
 * ======================================================================================================================
 *  sensors.h - I2C Sensor Definations
 * ======================================================================================================================
 */
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_HTU21DF.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_SI1145.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_VEML7700.h>

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

// Extern declarations for sensor objects
extern Adafruit_BMP280 bmp1;
extern Adafruit_BMP280 bmp2;
extern Adafruit_BME280 bme1;
extern Adafruit_BME280 bme2;
extern Adafruit_BMP3XX bm31;
extern Adafruit_BMP3XX bm32;

// Extern declarations for variables
extern byte BMX_1_chip_id;
extern byte BMX_2_chip_id;
extern bool BMX_1_exists;
extern bool BMX_2_exists;
extern byte BMX_1_type;
extern byte BMX_2_type;
extern const char *bmxtype[];

/*
 * ======================================================================================================================
 *  HTU21D-F - I2C - Humidity & Temp Sensor
 * ======================================================================================================================
 */
extern Adafruit_HTU21DF htu;
extern bool HTU21DF_exists;

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

extern Adafruit_MCP9808 mcp1;
extern Adafruit_MCP9808 mcp2;
extern bool MCP_1_exists;
extern bool MCP_2_exists;

/*
 * ======================================================================================================================
 *  SHTX - I2C - Temperature & Humidity sensor (SHT31)  - Note the SHT40, SHT45 use same i2c address
 * ======================================================================================================================
 */
#define SHT_ADDRESS_1     0x44
#define SHT_ADDRESS_2     0x45        // ADR pin set high, VDD

extern Adafruit_SHT31 sht1;
extern Adafruit_SHT31 sht2;
extern bool SHT_1_exists;
extern bool SHT_2_exists;

/*
 * ======================================================================================================================
 *  HIH8 - I2C - Temperature & Humidity sensor (HIH8000)  - 
 * ======================================================================================================================
 */
#define HIH8000_ADDRESS   0x27
extern bool HIH8_exists;

/*
 * ======================================================================================================================
 *  Si1145 - I2C - UV/IR/Visible Light Sensor
 *  The SI1145 has a fixed I2C address (0x60), you can only connect one sensor per microcontroller!
 * ======================================================================================================================
 */
extern Adafruit_SI1145 uv;
extern bool SI1145_exists;
extern float si_last_vis;
extern float si_last_ir;
extern float si_last_uv;

/*
 * ======================================================================================================================
 *  VEML7700 - I2C - Lux Sensor
 * ======================================================================================================================
 */
#define VEML7700_ADDRESS   0x10
extern Adafruit_VEML7700 veml;
extern bool VEML7700_exists;

/*
 * ======================================================================================================================
 *  Heat Index Temperature - Derived from Temperature and Humidity Sensors
 * ======================================================================================================================
 */
extern bool HI_exists;

/* 
 *=======================================================================================================================
 * MSLP - Mean sea level pressure 
 *=======================================================================================================================
 */
extern bool MSLP_exists;

// Function prototype
byte get_Bosch_ChipID (byte address);
void bmx_initialize();
void htu21d_initialize();
void mcp9808_initialize();
void sht_initialize();
void hih8_initialize();
bool hih8_getTempHumid(float *t, float *h);
void si1145_initialize();
void vlx_initialize();
void hi_initialize();
float hi_calculate(float T, float RH);
void mslp_initialize();
double mslp_calculate(float Ts, float RH, float ps, int station_height);
