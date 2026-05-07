/*
 * ======================================================================================================================
 *  sensors_i2c_44_47.cpp - Discover I2C Sensors on addresses 0x44 to 0x47
 * ======================================================================================================================
 */
#include "include/qc.h"
#include "include/sensors.h"
#include "include/sensors_i2c_44_47.h"
#include "include/support.h"
#include "include/output.h"
#include "include/obs.h"
#include "include/main.h"

/*
 * ======================================================================================================================
 * Variables and Data Structures
 * =======================================================================================================================
 */
I2C_44_47_SENSOR_SLOT i2c_44_47_sensors[I2C_44_47_SENSOR_COUNT];

bool SHT_1_exists = false;
float sht1_humid = 0.0;
float sht1_temp = 0.0;


/*
 * ======================================================================================================================
 * Fuction Definations
 * =======================================================================================================================
 */

/*
 * ======================================================================================================================
 * readBytes()
 * =======================================================================================================================
 */
bool readBytes(uint8_t addr, uint8_t *buf, size_t len, uint16_t timeoutMs = 1000) {
  Wire.requestFrom(addr, (uint8_t)len);
  unsigned long start = millis();
  size_t i = 0;
  while (i < len && (millis() - start) < timeoutMs) {
    if (Wire.available()) {
      buf[i++] = Wire.read();
    }
  }
  return i == len;
}

/*
 * ======================================================================================================================
 * sht31_probe()
 * =======================================================================================================================
 */
bool sht31_probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x37);
  Wire.write(0x80);
  if (Wire.endTransmission(false) != 0) return false;

  delay(1);

  uint8_t data[6];
  if (Wire.requestFrom(addr, (uint8_t)6) != 6) return false;

  for (int i = 0; i < 6; i++) data[i] = Wire.read();

  uint16_t sn1 = ((uint16_t)data[0] << 8) | data[1];
  uint16_t sn2 = ((uint16_t)data[3] << 8) | data[4];

  return (sn1 != 0x0000 || sn2 != 0x0000);
}

/*
 * ======================================================================================================================
 * sht45_probe()
 * =======================================================================================================================
 */
bool sht45_probe() {
  Adafruit_SHT4x sht4;
  return sht4.begin();
}

/*
 * ======================================================================================================================
 * bmp581_probe()
 * =======================================================================================================================
 */
bool bmp581_probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x01);                    // CHIP_ID register = 0x50
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;

  uint8_t chipid = Wire.read();
  return (chipid == 0x50);
}

/*
 * ======================================================================================================================
 * i2c_scan_sensor_type() - SHT31 SHT45 BMP581 and HDC302X all live in this address space
 * =======================================================================================================================
 */
I2C_44_47_SENSOR_TYPE i2c_scan_sensor_type(uint8_t addr) {

  if (addr == 0x44) {
    // SHT31
    if (sht31_probe(addr)) {
      sprintf (Buffer32Bytes, "[%02X] SHT31", addr);
      Output (Buffer32Bytes);
      return (SENSOR_SHT31);
    }
    // SHT45
    else if (sht45_probe()) {
      sprintf (Buffer32Bytes, "[%02X] SHT45", addr);
      Output (Buffer32Bytes);
      return (SENSOR_SHT45);
    }
  }

  if (addr == 0x45) {
    // SHT31
    if (sht31_probe(addr)) {
      sprintf (Buffer32Bytes, "[%02X] SHT31", addr);
      Output (Buffer32Bytes);
      return (SENSOR_SHT31);
    }
  }

  if ((addr == 0x46) || (addr == 0x47)) {
    // BMP581
    if (bmp581_probe(addr)) {
      sprintf (Buffer32Bytes, "[%02X] BMP581", addr);
      Output (Buffer32Bytes);
      return (SENSOR_BMP581);
    }
  }

  // If we get here then not the above, it could be a HDC that can live on all 4 i2c addresses
  sprintf (Buffer32Bytes, "[%02X] NF", addr);
  Output (Buffer32Bytes);
  return (SENSOR_UNKNOWN);
}

/* 
 *=======================================================================================================================
 * readSHT31SerialNumber() - display sht3x details
 *=======================================================================================================================
 */
uint32_t readSHT31SerialNumber(uint8_t i2cAddr) {
  uint8_t buffer[6];
  const uint16_t READ_SERIAL_CMD = 0x3780;

  // 1. Send the command
  Wire.beginTransmission(i2cAddr);
  Wire.write(highByte(READ_SERIAL_CMD));
  Wire.write(lowByte(READ_SERIAL_CMD));
  
  // Return 0 if the transmission fails (sensor not present or bus busy)
  if (Wire.endTransmission() != 0) {
    return 0;
  }

  delay(1);

  // 2. Request the 6-byte data packet
  if (Wire.requestFrom(i2cAddr, (uint8_t)6) != 6) {
    return 0; // Return 0 if we didn't receive exactly 6 bytes
  }

  for (int i = 0; i < 6; i++) {
    buffer[i] = Wire.read();
  }

  // 3. Reconstruct and return the 32-bit serial number
  return ((uint32_t)buffer[0] << 24) | 
         ((uint32_t)buffer[1] << 16) | 
         ((uint32_t)buffer[3] << 8)  | 
         ((uint32_t)buffer[4]);
}

/* 
 *=======================================================================================================================
 * sht3_detail() - display sht3x details
 *=======================================================================================================================
 */
void sht3_detail(int idx) {
  Adafruit_SHT31 &sht3 = i2c_44_47_sensors[idx].sht3;

  Output (" SHT3x Information");
  sprintf (Buffer32Bytes, " Serial Number:%s", i2c_44_47_sensors[idx].sn);
  Output(Buffer32Bytes);

  if (sht3.isHeaterEnabled()) {
    Output(" Heater:ON");
  }
  else {
    Output(" Heater:OFF");
  }
}

/* 
 *=======================================================================================================================
 * sht4_detail() - display sht4x details
 *=======================================================================================================================
 */
void sht4_detail(int idx) {
  Adafruit_SHT4x &sht4 = i2c_44_47_sensors[idx].sht4;  // Create a Alias

  Output (" SHT4x Information");
  sprintf (Buffer32Bytes, " Serial Number:%s", i2c_44_47_sensors[idx].sn);
  Output(Buffer32Bytes);

  switch (sht4.getPrecision()) {
     case SHT4X_HIGH_PRECISION:  // default
       Output(" High precision");
       break;
     case SHT4X_MED_PRECISION: 
       Output(" Med precision");
       break;
     case SHT4X_LOW_PRECISION: 
       Output(" Low precision");
       break;
  }

  switch (sht4.getHeater()) {
     case SHT4X_NO_HEATER:   // default
       Output(" No heater");
       break;
     case SHT4X_HIGH_HEATER_1S: 
       Output(" High heat for 1 second");
       break;
     case SHT4X_HIGH_HEATER_100MS: 
       Output( "High heat for 0.1 second");
       break;
     case SHT4X_MED_HEATER_1S: 
       Output(" Medium heat for 1 second");
       break;
     case SHT4X_MED_HEATER_100MS: 
       Output(" Medium heat for 0.1 second");
       break;
     case SHT4X_LOW_HEATER_1S: 
       Output(" Low heat for 1 second");
       break;
     case SHT4X_LOW_HEATER_100MS: 
       Output(" Low heat for 0.1 second");
       break;
  }
}

/* 
 *=======================================================================================================================
 * sensor_i2c_44_47_info() - 
 *=======================================================================================================================
 */
void sensor_i2c_44_47_info(char *rest, int size, const char *&comma) {
  for (uint8_t addr = 0x44; addr <= 0x47; addr++) {
    int idx = addr - 0x44;
    const char *name = nullptr;

    switch (i2c_44_47_sensors[idx].type) {
      case SENSOR_SHT31:   name = "SHT31";   break;
      case SENSOR_SHT45:   name = "SHT45";   break;
      case SENSOR_BMP581:  name = "BMP581";  break;
      default: break;
    }

    if (name) {
      int used = strlen(rest);
      int n;

      if (i2c_44_47_sensors[idx].sn[0]) {
        n = sprintf(Buffer32Bytes, "%s%s(%02x-%s)", comma, name, i2c_44_47_sensors[idx].i2c_address, i2c_44_47_sensors[idx].sn);
      }
      else {
        n = sprintf(Buffer32Bytes, "%s%s(%02x)", comma, name, i2c_44_47_sensors[idx].i2c_address);
      }

      // can we add with out over flowing?
      if ((used + n) < size) {
        sprintf(rest + used, "%s", Buffer32Bytes);
        comma = ",";
      }
      else {
        return;
      }
    }
  }
}

/* 
 *=======================================================================================================================
 * sensor_i2c_44_47_statmon() - pass in 0 - 3 for idx
 *=======================================================================================================================
 */
void sensor_i2c_44_47_statmon(int idx, char *buf) {
  float t,p,h;

  if ((idx <0) || (idx>=I2C_44_47_SENSOR_COUNT)) {
    sprintf (buf, "INVAL IDX %d", idx);
    return;
  }

  int id =i2c_44_47_sensors[idx].id;

  switch (i2c_44_47_sensors[idx].type) {
    case SENSOR_SHT31: {
      Adafruit_SHT31 &sht3 = i2c_44_47_sensors[idx].sht3; // Create a Alias
      t = sht3.readTemperature();
      h = sht3.readHumidity();
      t = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
      h = (isnan(h) || (h < QC_MIN_RH) || (h > QC_MAX_RH)) ? QC_ERR_RH : h;
      sprintf (buf, "SHT31-%d T%.2f H%.2f", id, t, h);
      break;
    }
    case SENSOR_SHT45: {
      Adafruit_SHT4x &sht4 = i2c_44_47_sensors[idx].sht4;  // Create a Alias
      sensors_event_t humidity, temp;
      sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
      t = temp.temperature;
      h = humidity.relative_humidity;
      t = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
      h = (isnan(h) || (h < QC_MIN_RH) || (h > QC_MAX_RH)) ? QC_ERR_RH : h;
      sprintf (buf, "SHT45-%d T%.2f H%.2f", id, t, h);
      break;
    }
    case SENSOR_BMP581:{
      Adafruit_BMP5xx &bmp5 = i2c_44_47_sensors[idx].bmp5;
      t = bmp5.readTemperature();
      p = bmp5.readPressure();
      t = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
      p = (isnan(p) || (p < QC_MIN_P)  || (p > QC_MAX_P))  ? QC_ERR_P  : p;
      sprintf (buf, "BMP5-%d T%.2f P%.2f", id, t, p);
      break;
    }
    case SENSOR_UNKNOWN :
    default: {
      sprintf (buf, "@%02X NF", 0x44+idx);
      break;
    }
  }
}

/* 
 *=======================================================================================================================
 * sensor_i2c_44_47_sht1_do() - loop through 4 addresses and look for SHT first occurance - set sht1_temp, sht1_humid
 *=======================================================================================================================
 */
bool sensor_i2c_44_47_sht1_do() {
  for (uint8_t addr = 0x44; addr <= 0x47; addr++) {
    int idx = addr - 0x44;

    switch (i2c_44_47_sensors[idx].type) {
      case SENSOR_SHT31 : {
        if (i2c_44_47_sensors[idx].id == 1) {
          float t,h;
          Adafruit_SHT31 &sht3 = i2c_44_47_sensors[idx].sht3; // Create a Alias

          t = sht3.readTemperature();
          t = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
          h = sht3.readHumidity();
          h = (isnan(h) || (h < QC_MIN_RH) || (h > QC_MAX_RH)) ? QC_ERR_RH : h;
          sht1_temp = t;
          sht1_humid = h;
          return (true);
        }
        break;
      }

      case SENSOR_SHT45 : {
        if (i2c_44_47_sensors[idx].id == 1) {
          float t,h;
          sensors_event_t humidity, temp;
          Adafruit_SHT4x &sht4 = i2c_44_47_sensors[idx].sht4;  // Create a Alias
          
          sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
          t = temp.temperature;
          h = humidity.relative_humidity;
          t = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
          h = (isnan(h) || (h < QC_MIN_RH) || (h > QC_MAX_RH)) ? QC_ERR_RH : h;
          sht1_temp = t;
          sht1_humid = h;
          return (true);
        }
        break;
      }

      case SENSOR_UNKNOWN :
      default : {
        break;
      }
    } // switch
  } // for
  return (false);
}

/* 
 *=======================================================================================================================
 * sensor_i2c_44_47_obs_do() - 
 *=======================================================================================================================
 */
void sensor_i2c_44_47_obs_do(JSONBufferWriter &writer) {
  for (uint8_t addr = 0x44; addr <= 0x47; addr++) {
    int idx = addr - 0x44;

    switch (i2c_44_47_sensors[idx].type) {
      case SENSOR_SHT31 : {
        Adafruit_SHT31 &sht3 = i2c_44_47_sensors[idx].sht3; // Create a Alias
        int id = i2c_44_47_sensors[idx].id;
        float t = 0.0;
        float h = 0.0;

        // SHT3 Temperature
        sprintf (Buffer32Bytes, "st%d", id);
        t = sht3.readTemperature();
        t = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
        writer.name(Buffer32Bytes).value(t, 2);
    
        // SHT3 Humidity   
        sprintf (Buffer32Bytes, "sh%d", id);
        h = sht3.readHumidity();
        h = (isnan(h) || (h < QC_MIN_RH) || (h > QC_MAX_RH)) ? QC_ERR_RH : h;
        writer.name(Buffer32Bytes).value(h, 2);

        if (id == 1) {
          // save for derived observations
          sht1_temp = t;
          sht1_humid = h; 
        }

        break;
      }

      case SENSOR_SHT45 : {
        Adafruit_SHT4x &sht4 = i2c_44_47_sensors[idx].sht4;  // Create a Alias
        int id = i2c_44_47_sensors[idx].id;
        sensors_event_t humidity, temp;
        float t=0.0;
        float h=0.0;

        sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
        t = temp.temperature;
        h = humidity.relative_humidity;
        t = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
        h = (isnan(h) || (h < QC_MIN_RH) || (h > QC_MAX_RH)) ? QC_ERR_RH : h;

        // SHT4 Temperature
        sprintf (Buffer32Bytes, "st%d", id);
        writer.name(Buffer32Bytes).value(t, 2);
    
        // SHT4 Humidity   
        sprintf (Buffer32Bytes, "sh%d", id);
        writer.name(Buffer32Bytes).value(h, 2);

        if (id == 1) {
          // save for derived observations
          sht1_temp = t;
          sht1_humid = h; 
        }
        break;
      }

      case SENSOR_BMP581 : {
        Adafruit_BMP5xx &bmp5 = i2c_44_47_sensors[idx].bmp5;
        int id = i2c_44_47_sensors[idx].id;
        float t=0.0;
        float p=0.0;

        t = bmp5.readTemperature();
        p = bmp5.readPressure();
        t = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
        p = (isnan(p) || (p < QC_MIN_P)  || (p > QC_MAX_P))  ? QC_ERR_P  : p;

        // BMP Temperature
        sprintf (Buffer32Bytes, "bt%d", id);
        writer.name(Buffer32Bytes).value(t, 2);

        // BMP Pressure 
        sprintf (Buffer32Bytes, "bp%d", id);
        writer.name(Buffer32Bytes).value(p, 2);
        
        if (id == 1) {
          bmx_1_pressure = p; // Used later for mslp calc
        }
        break;
      }

      case SENSOR_UNKNOWN :
      default : {
        break;
      }
    }
  }
}


/*
 * ======================================================================================================================
 * sensor_initialize_i2c_44_47() - 
 * =======================================================================================================================
 */
void sensor_initialize_i2c_44_47() {
  Output ("INIT I2C 44-47");

  int s_count=0;    // obs tags st# sh#
  int bmp_count=0;  // obs tags bt# bp# bh#

  for (uint8_t addr = 0x44; addr <= 0x47; addr++) {
    int idx = addr - 0x44;

    i2c_44_47_sensors[idx].type = i2c_scan_sensor_type(addr);
    i2c_44_47_sensors[idx].i2c_address = addr;

    switch (i2c_44_47_sensors[idx].type) {
      case SENSOR_SHT31 : {
        s_count++;
        i2c_44_47_sensors[idx].id = s_count;
        Adafruit_SHT31 &sht3 = i2c_44_47_sensors[idx].sht3;  // Create a Alias 
        if (!sht3.begin(addr)) {
          sprintf (Buffer32Bytes, " Init SHT(%d) ERR", s_count);
        }
        else {
          sprintf (Buffer32Bytes, " Init SHT(%d) OK", s_count);
        }
        Output (Buffer32Bytes);
        snprintf (i2c_44_47_sensors[idx].sn, I2C_44_77_SN_LEN, "%lX", readSHT31SerialNumber(addr));
        sht3_detail(idx);
        if (s_count==1) {
          SHT_1_exists = true;
        }
        break;
      }
      case SENSOR_SHT45 : {
        s_count++;
        i2c_44_47_sensors[idx].id = s_count;
        Adafruit_SHT4x &sht4 = i2c_44_47_sensors[idx].sht4;  // Create a Alias 
        if (!sht4.begin()) {
          sprintf (Buffer32Bytes, " Init SHT(%d) ERR", s_count);
          Output (Buffer32Bytes);
        }
        else {
          sprintf (Buffer32Bytes, " Init SHT(%d) OK", s_count);
          Output (Buffer32Bytes);

          // You can have 3 different precisions, higher precision takes longer
          sht4.setPrecision(SHT4X_HIGH_PRECISION);

          // You can have 6 different heater settings
          // higher heat and longer times uses more power
          // and reads will take longer too!
          sht4.setHeater(SHT4X_NO_HEATER);
          snprintf (i2c_44_47_sensors[idx].sn, I2C_44_77_SN_LEN, "%lX", sht4.readSerial());
          sht4_detail(idx);
          if (s_count==1) {
            SHT_1_exists = true;
          }
        }
        break;
      }
      case SENSOR_BMP581 : {
        // Allow the bmp581 to be bmp1 or bmp2 tags. Otherwise it becomes 3,4
        if (!BMX_1_exists && (bmp_count==0)) {
          i2c_44_47_sensors[idx].id=1;
          bmp_count=1;
        }
        else if (!BMX_2_exists && (bmp_count<2)) {
          i2c_44_47_sensors[idx].id=2;
          bmp_count=2;
        }
        else {
          if (!bmp_count) {
            bmp_count=2;  // we got here because BMX_1&2_exists, so we need to be 3 or more for the tag id
          }
          bmp_count++;
          i2c_44_47_sensors[idx].id=bmp_count;
        }
        Adafruit_BMP5xx &bmp5 = i2c_44_47_sensors[idx].bmp5;
        if (!bmp5.begin(addr, &Wire)) { 
          sprintf (Buffer32Bytes, " Init BMP(%d) ERR", bmp_count);
          Output (Buffer32Bytes);
        }
        else {
          sprintf (Buffer32Bytes, " Init BMP(%d) OK", bmp_count);
          Output (Buffer32Bytes);
          // Configure sensor for optimal performance
          bmp5.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
          bmp5.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
          bmp5.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
          bmp5.setOutputDataRate(BMP5XX_ODR_50_HZ);
          bmp5.setPowerMode(BMP5XX_POWERMODE_NORMAL);

          bmp5.readPressure();
        }
        break;
      }
      case SENSOR_UNKNOWN :
      default : {
        i2c_44_47_sensors[idx].id = 0;
        break;
      }
    }
  }
}