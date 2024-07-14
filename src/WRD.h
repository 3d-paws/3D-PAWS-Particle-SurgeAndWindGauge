/*
 * ======================================================================================================================
 *  WRD.h - Wind Rain Distance Functions
 * ======================================================================================================================
 */

/*
 * =======================================================================================================================
 *  Distance Gauge
 * =======================================================================================================================
 */
#define DISTANCEGAUGE   A3
#define OD_BUCKETS      91        // Observation Distance Buckets
char SD_5M_DIST_FILE[] = "5MDIST.TXT";  // If file exists use adjustment of 1.25. No file, then 10m Sensor is 2.5
float od_adjustment = 2.5;       // Default sensor is 10m 
unsigned int od_bucket = 0;
unsigned int od_buckets[OD_BUCKETS];
unsigned int od_buckets_median[OD_BUCKETS]; // od_buckets will get copy to this, then we will sort for median

/*
 * ======================================================================================================================
 *  Wind Related Setup
 * 
 *  NOTE: With interrupts tied to the anemometer rotation we are essentually sampling all the time.  
 *        We record the interrupt count, ms duration and wind direction every second.
 *        One revolution of the anemometer results in 2 interrupts. There are 2 magnets on the anemometer.
 * 
 *        Station observations are logged every minute
 *        Wind and Direction are sampled every second producing 60 samples 
 *        The one second wind speed sample are produced from the interrupt count and ms duration.
 *        Wind Observations a 
 *        Reported Observations
 *          Wind Speed = Average of the 60 samples.
 *          Wind Direction = Average of the 60 vectors from Direction and Speed.
 *          Wind Gust = Highest 3 consecutive samples from the 60 samples. The 3 samples are then averaged.
 *          Wind Gust Direction = Average of the 3 Vectors from the Wind Gust samples.
 * ======================================================================================================================
 */
#define ANEMOMETER_IRQ_PIN  A2
bool DoWindOBS = false;          // Is set to true in setup if AS5600 wind direction is found, we thn will do observations      
#define WIND_READINGS   60       // One minute of 1s Samples

typedef struct {
  int direction;
  float speed;
} WIND_BUCKETS_STR;

typedef struct {
  WIND_BUCKETS_STR bucket[WIND_READINGS];
  int bucket_idx;
  float gust;
  int gust_direction;
} WIND_STR;
WIND_STR wind;

/*
 * ======================================================================================================================
 *  Wind Direction - AS5600 Sensor
 * ======================================================================================================================
 */     
bool      AS5600_exists     = true;
int       AS5600_ADR        = 0x36;
const int AS5600_raw_ang_hi = 0x0c;
const int AS5600_raw_ang_lo = 0x0d;

/*
 * ======================================================================================================================
 *  Wind Speed Calibration
 * ======================================================================================================================
 */
float ws_calibration = 2.64;       // From wind tunnel testing
float ws_radius = 0.079;           // In meters

/*
 * ======================================================================================================================
 *  Optipolar Hall Effect Sensor SS451A - Interrupt 1 - Aneometer
 * ======================================================================================================================
 */
volatile unsigned int anemometer_interrupt_count;
uint64_t anemometer_interrupt_stime;

/*
 * ======================================================================================================================
 *  anemometer_interrupt_handler() - This function is called whenever a magnet/interrupt is detected by the arduino
 * ======================================================================================================================
 */
void anemometer_interrupt_handler()
{
  anemometer_interrupt_count++;
}

/* 
 *=======================================================================================================================
 * as5600_initialize() - wind direction sensor
 *=======================================================================================================================
 */
void as5600_initialize() {
  Output("AS5600:INIT");
  Wire.beginTransmission(AS5600_ADR);
  if (Wire.endTransmission()) {
    msgp = (char *) "WD:NF";
    AS5600_exists = false;
    SystemStatusBits |= SSB_AS5600;  // Turn On Bit
  }
  else {
    msgp = (char *) "WD:OK";
  }
  Output (msgp);
}

/* 
 *=======================================================================================================================
 * OBS_Median_Distance()
 *=======================================================================================================================
 */
int OBS_Median_Distance() {
  unsigned int bucket[5];

  bucket[0] = (int) analogRead(DISTANCEGAUGE);
  delay(10);
  bucket[1] = (int) analogRead(DISTANCEGAUGE);
  delay(10);
  bucket[2] = (int) analogRead(DISTANCEGAUGE);
  delay(10);
  bucket[3] = (int) analogRead(DISTANCEGAUGE);
  delay(10);
  bucket[4] = (int) analogRead(DISTANCEGAUGE);

  mysort(bucket, 5);
  return ((int) bucket[2]);
}

/* 
 *=======================================================================================================================
 * OBS_Distance_Do()
 *=======================================================================================================================
 */
void OBS_Distance_Do() {
  od_buckets[od_bucket] = OBS_Median_Distance(); // median value 
  od_bucket = (++od_bucket) % OD_BUCKETS;
}

/* 
 *=======================================================================================================================
 * Wind_SampleDirection() -- Talk i2c to the AS5600 sensor and get direction
 *=======================================================================================================================
 */
int Wind_SampleDirection() {
  
  // Read Raw Angle Low Byte
  Wire.beginTransmission(AS5600_ADR);
  Wire.write(AS5600_raw_ang_lo);
  if (Wire.endTransmission()) {
    if (AS5600_exists) {
      Output ("WD Offline_L");
    }
    AS5600_exists = false;
  }
  else if (Wire.requestFrom(AS5600_ADR, 1)) {
    int AS5600_lo_raw = Wire.read();
  
    // Read Raw Angle High Byte
    Wire.beginTransmission(AS5600_ADR);
    Wire.write(AS5600_raw_ang_hi);
    if (Wire.endTransmission()) {
      if (AS5600_exists) {
        Output ("WD Offline_H");
      }
      AS5600_exists = false;
    }
    else if (Wire.requestFrom(AS5600_ADR, 1)) {
      word AS5600_hi_raw = Wire.read();

      if (!AS5600_exists) {
        Output ("WD Online");
      }
      AS5600_exists = true;           // We made it 
      SystemStatusBits &= ~SSB_AS5600; // Turn Off Bit
      
      AS5600_hi_raw = AS5600_hi_raw << 8; //shift raw angle hi 8 left
      AS5600_hi_raw = AS5600_hi_raw | AS5600_lo_raw; //AND high and low raw angle value
      return ((int) AS5600_hi_raw *0.0879);
    }
  }
  SystemStatusBits |= SSB_AS5600;  // Turn On Bit
  return (-1); // Not the best value to return 
}

/* 
 *=======================================================================================================================
 * Wind_SampleSpeed() - Return a wind speed based on interrupts and duration wind
 * 
 * Optipolar Hall Effect Sensor SS451A - Aneometer
 * speed  = (( (signals/2) * (2 * pi * radius) ) / time) * calibration_factor
 * speed in m/s =  (   ( (interrupts/2) * (2 * 3.14156 * 0.079) )  / (time_period in ms / 1000)  )  * 2.64
 *=======================================================================================================================
 */
float Wind_SampleSpeed() {
  uint64_t delta_ms;
  float wind_speed;
  
  delta_ms = System.millis()-anemometer_interrupt_stime;

  if (anemometer_interrupt_count) {
    // wind_speed = (  ( (anemometer_interrupt_count/2) * (2 * 3.14156 * ws_radius) )  / 
    //  (float)( (float)delta_ms / 1000)  )  * ws_calibration;

    wind_speed = ( ( anemometer_interrupt_count * 3.14156 * ws_radius)  / 
        (float)( (float)delta_ms / 1000) )  * ws_calibration;
  }
  else {
    wind_speed = 0.0;
  }

  anemometer_interrupt_count = 0;
  anemometer_interrupt_stime = System.millis();
  
  return (wind_speed);
} 

/*
 * ======================================================================================================================
 * Wind_TakeReading() - Wind direction and speed, measure every second             
 * ======================================================================================================================
 */
void Wind_TakeReading() {
  if (DoWindOBS) {
    wind.bucket[wind.bucket_idx].direction = (int) Wind_SampleDirection();
    wind.bucket[wind.bucket_idx].speed = Wind_SampleSpeed();
    wind.bucket_idx = (++wind.bucket_idx) % WIND_READINGS; // Advance bucket index for next reading
  }
}

/* 
 *=======================================================================================================================
 * OBS_WindAndDistance_Fill()
 *=======================================================================================================================
 */
void OBS_WindAndDistance_Fill() {
  float distance;

  Output("Wind&Distance Fill");

  // Clear windspeed counter  
  anemometer_interrupt_count = 0;
  anemometer_interrupt_stime = System.millis();
  
  // Init default values.
  wind.gust = 0.0;
  wind.gust_direction = -1;
  wind.bucket_idx = 0;

  for (int i=0; i<OD_BUCKETS; i++) {
    od_buckets[od_bucket] = OBS_Median_Distance();
    distance = od_buckets[od_bucket] * od_adjustment; // Pins are 12bit resolution 

    sprintf (Buffer32Bytes, "D:%d %d.%02d", i, (int)distance, (int)(distance*100)%100);
    Output(Buffer32Bytes);

    od_bucket = (++od_bucket) % OD_BUCKETS;

    Wind_TakeReading();
    HeartBeat();  // Provides a 250ms delay
    delay (750);
  }
}

/* 
 *=======================================================================================================================
 * OBS_Distance_Median() - return median from the last 91 observations
 *=======================================================================================================================
 */
float OBS_Distance_Median() {

  // Make a copy so we can do a sort
  for (int b=0; b<OD_BUCKETS; b++) {
    od_buckets_median[b] = od_buckets[b];
  }
  
  mysort(od_buckets_median, OD_BUCKETS);

  // Return the Center Observation
  return ((float) od_buckets_median[45]*od_adjustment);
}

/* 
 *=======================================================================================================================
 * OBS_Distance_Calc()
 * 
 *   91 one-second water level samples centered on each tenth of an hour are averaged, a three standard deviation 
 *   outlier rejection test applied, the mean and standard deviation are recalculated and reported along with 
 *   the number of outliers. (3 minute water level average).
 * 
 * We can use the observations to capture the mean and standard deviation and then multiply the value by 3. 
 * 
 * To calculate the standard deviation of those numbers:
 *   Work out the Mean (the simple average of the numbers)
 *   Then for each number: subtract the Mean and square the result.
 *   Then work out the mean of those squared differences.
 *   Take the square root of that and we are done!
 * 
 * A sigma value is a statistical term otherwise known as a standard deviation
 * 
 * The three-sigma process:
 *   1 Calculate the mean
 *   2 Calculate the standard deviation
 *   3 Multiply the standard deviation by 3
 *   4 Subtract the product in step 3 from the mean
 *=======================================================================================================================
 */
void OBS_Distance_Calc(float *m, float *d, int *o) {  // (mean, stdev, outliers)
  float mean=0.0;
  float mean_minus_outliers=0.0;
  float sqDevSum=0.0;
  float stDev=0.0;
  float cutoff=0.0;
  int outliers=0;

  // Calculate the Mean
  for (int i=0; i<OD_BUCKETS; i++) {
    mean += od_buckets[i];
  }
  mean /= (float) OD_BUCKETS;

  // Calculate the standard deviation
  for (int i=0; i<OD_BUCKETS; i++) {
    sqDevSum += sq(od_buckets[i] - mean);
  }
  stDev = sqrt(sqDevSum / (float) OD_BUCKETS);  // Take square root of variance

  // Calculate the 3 standard deviation for our cutoff
  cutoff = stDev*3;

  // Calculate the Mean with Outliers Removed
  for (int i=0; i<OD_BUCKETS; i++) {
    if ( (od_buckets[i] >= (mean - cutoff)) && (od_buckets[i] <= (mean + cutoff)) ) {
      mean_minus_outliers += od_buckets[i];
    }
    else {
      outliers++;
    }
  }
  mean_minus_outliers /= (OD_BUCKETS-outliers);

  // Calculate the standard deviation with Outliers Removed
  sqDevSum = 0.0;
  for (int i=0; i<OD_BUCKETS; i++) {
    if ( (od_buckets[i] >= (mean - cutoff)) && (od_buckets[i] <= (mean + cutoff)) ) {
      sqDevSum += sq(od_buckets[i] - mean_minus_outliers);
    }
  }
  stDev = sqrt(sqDevSum / (float) (OD_BUCKETS-outliers));

  // Pins are 12bit resolution (0-4095)
  *m = mean_minus_outliers * (float) od_adjustment;
  *d = stDev * od_adjustment;
  *o = outliers;

  // sprintf (Buffer32Bytes, "sD:%d.%02d",  (int)sqDevSum, (int)(sqDevSum*100)%100);
  // Output(Buffer32Bytes);
  // sprintf (Buffer32Bytes, "D:%d.%02d|%d.%02d|",  (int)stDev, (int)(stDev*100)%100, (int) *d, (int)(*d *100)%100);
  // Output(Buffer32Bytes);
}

/* 
 *=======================================================================================================================
 * Wind_DirectionVector()
 *=======================================================================================================================
 */
int Wind_DirectionVector() {
  double NS_vector_sum = 0.0;
  double EW_vector_sum = 0.0;
  double r;
  float s;
  int d, i, rtod;
  bool ws_zero = true;

  for (i=0; i<WIND_READINGS; i++) {
    d = wind.bucket[i].direction;

    // if at any time 1 of the 60 wind direction readings is -1
    // then the sensor was offline and we need to invalidate or data
    // until it is clean with out any -1's
    if (d == -1) {
      return (-1);
    }
    
    s = wind.bucket[i].speed;

    // Flag we have wind speed
    if (s > 0) {
      ws_zero = false;  
    }
    r = (d * 71) / 4068.0;
    
    // North South Direction 
    NS_vector_sum += cos(r) * s;
    EW_vector_sum += sin(r) * s;
  }
  rtod = (atan2(EW_vector_sum, NS_vector_sum)*4068.0)/71.0;
  if (rtod<0) {
    rtod = 360 + rtod;
  }

  // If all the winds speeds are 0 then we return current wind direction or 0 on failure of that.
  if (ws_zero) {
    return (Wind_SampleDirection()); // Can return -1
  }
  else {
    return (rtod);
  }
}

/* 
 *=======================================================================================================================
 * Wind_SpeedAverage()
 *=======================================================================================================================
 */
float Wind_SpeedAverage() {
  float wind_speed = 0.0;
  for (int i=0; i<WIND_READINGS; i++) {
    // sum wind speeds for later average
    wind_speed += wind.bucket[i].speed;
  }
  return( wind_speed / (float) WIND_READINGS);
}

/* 
 *=======================================================================================================================
 * Wind_Gust()
 *=======================================================================================================================
 */
float Wind_Gust() {
  return(wind.gust);
}

/* 
 *=======================================================================================================================
 * Wind_GustDirection()
 *=======================================================================================================================
 */
int Wind_GustDirection() {
  return(wind.gust_direction);
}

/* 
 *=======================================================================================================================
 * Wind_GustUpdate()
 *   Wind Gust = Highest 3 consecutive samples from the 60 samples. The 3 samples are then averaged.
 *   Wind Gust Direction = Average of the 3 Vectors from the Wind Gust samples.
 * 
 *   Note: To handle the case of 2 or more gusts at the same speed but different directions
 *          Sstart with oldest reading and work forward to report most recent.
 * 
 *   Algorithm: 
 *     Start with oldest reading.
 *     Sum this reading with next 2.
 *     If greater than last, update last 
 * 
 *=======================================================================================================================
 */
void Wind_GustUpdate() {
  int bucket = wind.bucket_idx; // Start at next bucket to fill (aka oldest reading)
  float ws_sum = 0.0;
  int ws_bucket = bucket;
  float sum;

  for (int i=0; i<(WIND_READINGS-2); i++) {  // subtract 2 because we are looking ahead at the next 2 buckets
    // sum wind speeds 
    sum = wind.bucket[bucket].speed +
          wind.bucket[(bucket+1) % WIND_READINGS].speed +
          wind.bucket[(bucket+2) % WIND_READINGS].speed;
    if (sum >= ws_sum) {
      ws_sum = sum;
      ws_bucket = bucket;
    }
    bucket = (++bucket) % WIND_READINGS;
  }
  wind.gust = ws_sum/3;
  
  // Determine Gust Direction 
  double NS_vector_sum = 0.0;
  double EW_vector_sum = 0.0;
  double r;
  float s;
  int d, i, rtod;
  bool ws_zero = true;

  bucket = ws_bucket;
  for (i=0; i<3; i++) {
    d = wind.bucket[bucket].direction;

    // if at any time any wind direction readings is -1
    // then the sensor was offline and we need to invalidate or data
    // until it is clean with out any -1's
    if (d == -1) {
      ws_zero = true;
      break;
    }
    
    s = wind.bucket[bucket].speed;

    // Flag we have wind speed
    if (s > 0) {
      ws_zero = false;  
    }
    r = (d * 71) / 4068.0;
    
    // North South Direction 
    NS_vector_sum += cos(r) * s;
    EW_vector_sum += sin(r) * s;

    bucket = (++bucket) % WIND_READINGS;
  }

  rtod = (atan2(EW_vector_sum, NS_vector_sum)*4068.0)/71.0;
  if (rtod<0) {
    rtod = 360 + rtod;
  }

  // If all the winds speeds are 0 or we has a -1 direction then set -1 dor direction.
  if (ws_zero) {
    wind.gust_direction = -1;
  }
  else {
    wind.gust_direction = rtod;
  }
}

/*
 * ======================================================================================================================
 * I2C_Check_Sensors() - See if each I2C sensor responds on the bus and take action accordingly             
 * ======================================================================================================================
 */
void I2C_Check_Sensors() {

  // BMX_1 Barometric Pressure 
  if (I2C_Device_Exist (BMX_ADDRESS_1)) {
    // Sensor online but our state had it offline
    if (BMX_1_exists == false) {
      if (BMX_1_chip_id == BMP280_CHIP_ID) {
        if (bmp1.begin(BMX_ADDRESS_1)) { 
          BMX_1_exists = true;
          Output ("BMP1 ONLINE");
          SystemStatusBits &= ~SSB_BMX_1; // Turn Off Bit
        } 
      }
      else if (BMX_1_chip_id == BME280_BMP390_CHIP_ID) {
        if (BMX_1_type == BMX_TYPE_BME280) {
          if (bme1.begin(BMX_ADDRESS_1)) { 
            BMX_1_exists = true;
            Output ("BME1 ONLINE");
            SystemStatusBits &= ~SSB_BMX_1; // Turn Off Bit
          } 
        }
        if (BMX_1_type == BMX_TYPE_BMP390) {
          if (bm31.begin_I2C(BMX_ADDRESS_1)) {
            BMX_1_exists = true;
            Output ("BMP390_1 ONLINE");
            SystemStatusBits &= ~SSB_BMX_1; // Turn Off Bit
          }
        }        
      }
      else {
        if (bm31.begin_I2C(BMX_ADDRESS_1)) { 
          BMX_1_exists = true;
          Output ("BM31 ONLINE");
          SystemStatusBits &= ~SSB_BMX_1; // Turn Off Bit
        }                  
      }      
    }
  }
  else {
    // Sensor offline but our state has it online
    if (BMX_1_exists == true) {
      BMX_1_exists = false;
      Output ("BMX1 OFFLINE");
      SystemStatusBits |= SSB_BMX_1;  // Turn On Bit 
    }    
  }

  // BMX_2 Barometric Pressure 
  if (I2C_Device_Exist (BMX_ADDRESS_2)) {
    // Sensor online but our state had it offline
    if (BMX_2_exists == false) {
      if (BMX_2_chip_id == BMP280_CHIP_ID) {
        if (bmp2.begin(BMX_ADDRESS_2)) { 
          BMX_2_exists = true;
          Output ("BMP2 ONLINE");
          SystemStatusBits &= ~SSB_BMX_2; // Turn Off Bit
        } 
      }
      else if (BMX_2_chip_id == BME280_BMP390_CHIP_ID) {
        if (BMX_2_type == BMX_TYPE_BME280) {
          if (bme1.begin(BMX_ADDRESS_2)) { 
            BMX_2_exists = true;
            Output ("BME2 ONLINE");
            SystemStatusBits &= ~SSB_BMX_2; // Turn Off Bit
          } 
        }
        if (BMX_2_type == BMX_TYPE_BMP390) {
          if (bm31.begin_I2C(BMX_ADDRESS_2)) {
            BMX_1_exists = true;
            Output ("BMP390_1 ONLINE");
            SystemStatusBits &= ~SSB_BMX_2; // Turn Off Bit
          }
        }        
      }
      else {
         if (bm32.begin_I2C(BMX_ADDRESS_2)) { 
          BMX_2_exists = true;
          Output ("BM32 ONLINE");
          SystemStatusBits &= ~SSB_BMX_2; // Turn Off Bit
        }                         
      }     
    }
  }
  else {
    // Sensor offline but we our state has it online
    if (BMX_2_exists == true) {
      BMX_2_exists = false;
      Output ("BMX2 OFFLINE");
      SystemStatusBits |= SSB_BMX_2;  // Turn On Bit 
    }    
  }

  // HTU21DF Humidity & Temp Sensor
  if (I2C_Device_Exist (HTU21DF_I2CADDR)) {
    // Sensor online but our state had it offline
    if (HTU21DF_exists == false) {
      // See if we can bring sensor online
      if (htu.begin()) {
        HTU21DF_exists = true;
        Output ("HTU ONLINE");
        SystemStatusBits &= ~SSB_HTU21DF; // Turn Off Bit
      }
    }
  }
  else {
    // Sensor offline but we our state has it online
    if (HTU21DF_exists == true) {
      HTU21DF_exists = false;
      Output ("HTU OFFLINE");
      SystemStatusBits |= SSB_HTU21DF;  // Turn On Bit
    }   
  }
    // SI1145 UV index & IR & Visible Sensor
  if (I2C_Device_Exist (SI1145_ADDR)) {
    // Sensor online but our state had it offline
    if (SI1145_exists == false) {
      // See if we can bring sensore online
      if (uv.begin()) {
        SI1145_exists = true;
        Output ("SI ONLINE");
        SystemStatusBits &= ~SSB_SI1145; // Turn Off Bit
      }
    }
  }
  else {
    // Sensor offline but we our state has it online
    if (SI1145_exists == true) {
      SI1145_exists = false;
      Output ("SI OFFLINE");
      SystemStatusBits |= SSB_SI1145;  // Turn On Bit
    }   
  }

  // VEML7700 Lux 
  if (I2C_Device_Exist (VEML7700_ADDRESS)) {
    // Sensor online but our state had it offline
    if (VEML7700_exists == false) {
      // See if we can bring sensor online
      if (veml.begin()) {
        VEML7700_exists = true;
        Output ("LUX ONLINE");
        SystemStatusBits &= ~SSB_LUX; // Turn Off Bit
      }
    }
  }
  else {
    // Sensor offline but we our state has it online
    if (VEML7700_exists == true) {
      VEML7700_exists = false;
      Output ("LUX OFFLINE");
      SystemStatusBits |= SSB_LUX;  // Turn On Bit
    }   
  }
}

