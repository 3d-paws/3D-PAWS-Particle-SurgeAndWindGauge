/*
 * ======================================================================================================================
 *  wrda.cpp - Wind Rain Distance Air Functions
 * ======================================================================================================================
 */
#include "include/ssbits.h"
#include "include/cf.h"
#include "include/output.h"
#include "include/support.h"
#include "include/sdcard.h"
#include "include/sensors.h"
#include "include/main.h"
#include "include/wrda.h"

/*
 * ======================================================================================================================
 * Variables and Data Structures
 * =======================================================================================================================
 */

 /*
 * ======================================================================================================================
 *  Wind Direction - AS5600 Sensor
 * ======================================================================================================================
 */    
WIND_STR wind; 
bool      AS5600_exists     = true; // if AS5600 wind direction is found, we then will do observations
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
  }
  else {
    msgp = (char *) "WD:OK";
  }
  Output (msgp);
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
      AS5600_hi_raw = AS5600_hi_raw << 8; //shift raw angle hi 8 left
      AS5600_hi_raw = AS5600_hi_raw | AS5600_lo_raw; //AND high and low raw angle value
      return ((int) AS5600_hi_raw *0.0879);
    }
  }
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
  if (AS5600_exists) {
    wind.bucket[wind.bucket_idx].direction = (int) Wind_SampleDirection();
    wind.bucket[wind.bucket_idx].speed = Wind_SampleSpeed();
    wind.bucket_idx = (++wind.bucket_idx) % WIND_READINGS; // Advance bucket index for next reading
  }
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
    bucket = (bucket+1) % WIND_READINGS;
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

    bucket = (bucket+1) % WIND_READINGS;
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
 * =======================================================================================================================
 *  Distance Gauge - Can be Distance or Stream
 * =======================================================================================================================
 */
float dg_adjustment = 2.5;       // Default sensor is 10m 
unsigned int dg_bucket = 0;
unsigned int dg_buckets[DG_BUCKETS];
unsigned int dg_buckets_median[DG_BUCKETS]; // dg_buckets will get copy to this, then we will sort for median


/* 
 *=======================================================================================================================
 * OBS_Distance_SimpleMedian()
 *=======================================================================================================================
 */
int OBS_Distance_SimpleMedian() {
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
  dg_buckets[dg_bucket] = OBS_Distance_SimpleMedian(); // median value 
  dg_bucket = (dg_bucket+1) % DG_BUCKETS;
}


/* 
 *=======================================================================================================================
 * OBS_Distance_Median() - return median from the last 91 observations
 *=======================================================================================================================
 */
float OBS_Distance_Median() {

  // Make a copy so we can do a sort
  for (int b=0; b<DG_BUCKETS; b++) {
    dg_buckets_median[b] = dg_buckets[b];
  }
  
  mysort(dg_buckets_median, DG_BUCKETS);

  // Return the Center Observation
  return ((float) dg_buckets_median[45]*dg_adjustment);
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
  for (int i=0; i<DG_BUCKETS; i++) {
    mean += dg_buckets[i];
  }
  mean /= (float) DG_BUCKETS;

  // Calculate the standard deviation
  for (int i=0; i<DG_BUCKETS; i++) {
    sqDevSum += sq(dg_buckets[i] - mean);
  }
  stDev = sqrt(sqDevSum / (float) DG_BUCKETS);  // Take square root of variance

  // Calculate the 3 standard deviation for our cutoff
  cutoff = stDev*3;

  // Calculate the Mean with Outliers Removed
  for (int i=0; i<DG_BUCKETS; i++) {
    if ( (dg_buckets[i] >= (mean - cutoff)) && (dg_buckets[i] <= (mean + cutoff)) ) {
      mean_minus_outliers += dg_buckets[i];
    }
    else {
      outliers++;
    }
  }
  mean_minus_outliers /= (DG_BUCKETS-outliers);

  // Calculate the standard deviation with Outliers Removed
  sqDevSum = 0.0;
  for (int i=0; i<DG_BUCKETS; i++) {
    if ( (dg_buckets[i] >= (mean - cutoff)) && (dg_buckets[i] <= (mean + cutoff)) ) {
      sqDevSum += sq(dg_buckets[i] - mean_minus_outliers);
    }
  }
  stDev = sqrt(sqDevSum / (float) (DG_BUCKETS-outliers));

  // Pins are 12bit resolution (0-4095)
  *m = mean_minus_outliers * (float) dg_adjustment;
  *d = stDev * dg_adjustment;
  *o = outliers;

  // sprintf (Buffer32Bytes, "sD:%d.%02d",  (int)sqDevSum, (int)(sqDevSum*100)%100);
  // Output(Buffer32Bytes);
  // sprintf (Buffer32Bytes, "D:%d.%02d|%d.%02d|",  (int)stDev, (int)(stDev*100)%100, (int) *d, (int)(*d *100)%100);
  // Output(Buffer32Bytes);
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
  
  for (int i=0; i<WIND_READINGS; i++) {
    wind.bucket[i].direction = (int) -999;
    wind.bucket[i].speed = 0.0;
  }

  for (int i=0; i<DG_BUCKETS; i++) {
    dg_buckets[dg_bucket] = OBS_Distance_SimpleMedian();
    distance = dg_buckets[dg_bucket] * dg_adjustment; // Pins are 12bit resolution 

    sprintf (Buffer32Bytes, "%d D:%d.%02d", i, (int)distance, (int)(distance*100)%100);
    if (AS5600_exists) {
      Wind_TakeReading();
      float ws = Wind_SpeedAverage();
      sprintf (Buffer32Bytes+strlen(Buffer32Bytes), " WD:%3d WS:%d.%02d", 
        Wind_SampleDirection(), (int)ws, (int)(ws*100)%100);
    }
    Output(Buffer32Bytes);
    dg_bucket = (++dg_bucket) % DG_BUCKETS;

    HeartBeat();  // Provides a 250ms delay
    delay (740);  // Substract a little time from 750 for loop execution (This is a guess)
  }
}