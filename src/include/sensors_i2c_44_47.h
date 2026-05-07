/*
 * ======================================================================================================================
 *  sensors_i2c_44_47.h - Discover I2C Sensors on addresses 0x44 to 0x47
 * ======================================================================================================================
 */
#include <Adafruit_SHT31.h>
#include <Adafruit_SHT4x.h>
#include <Adafruit_BMP5xx.h>


/*
 * sht3 0x44 Default
 * sht3 0x45 with jumper  
 * 
 * sht4 0x44 Default and only address
 * 
 * b58  0x47 default           ChipID BMP580 = 0x50, BMP581 = 0x51
 * b58  0x46 with jumper
 */

#define I2C_44_47_SENSOR_COUNT 4
#define I2C_44_77_SN_LEN 10

typedef enum {
    SENSOR_UNKNOWN,
    SENSOR_SHT31,
    SENSOR_SHT45,
    SENSOR_BMP581
} I2C_44_47_SENSOR_TYPE;

typedef struct {
    I2C_44_47_SENSOR_TYPE type;
    uint8_t i2c_address;
    uint8_t id;
    char sn[I2C_44_77_SN_LEN];
    Adafruit_SHT31 sht3;
    Adafruit_SHT4x sht4;
    Adafruit_BMP5xx bmp5;
} I2C_44_47_SENSOR_SLOT;


// Extern variables
extern bool SHT_1_exists;
extern float sht1_humid ;
extern float sht1_temp;

// Function prototypes
void sensor_i2c_44_47_info(char *rest, int size, const char *&comma);
void sensor_i2c_44_47_statmon(int idx, char *buf);
bool sensor_i2c_44_47_sht1_do();
void sensor_i2c_44_47_obs_do(JSONBufferWriter &writer);
void sensor_initialize_i2c_44_47();