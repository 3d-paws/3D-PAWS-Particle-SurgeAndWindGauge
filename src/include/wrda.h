/*
 * ======================================================================================================================
 *  wrda.h - Wind Rain Distance Air Definations
 * ======================================================================================================================
 */

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
 * 
 * Distance Sensors
 * The 5-meter sensors (MB7360, MB7369, MB7380, and MB7389) use a scale factor of (Vcc/5120) per 1-mm.
 * Particle 12bit resolution (0-4095),  Sensor has a resolution of 0 - 5119mm,  Each unit of the 0-4095 resolution is 1.25mm
 * Feather has 10bit resolution (0-1023), Sensor has a resolution of 0 - 5119mm, Each unit of the 0-1023 resolution is 5mm
 * 
 * The 10-meter sensors (MB7363, MB7366, MB7383, and MB7386) use a scale factor of (Vcc/10240) per 1-mm.
 * Particle 12bit resolution (0-4095), Sensor has a resolution of 0 - 10239mm, Each unit of the 0-4095 resolution is 2.5mm
 * Feather has 10bit resolution (0-1023), Sensor has a resolution of 0 - 10239mm, Each unit of the 0-1023 resolution is 10mm
 * ======================================================================================================================
 */
#define ANEMOMETER_IRQ_PIN  A2
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

/*
 * =======================================================================================================================
 * Distance Gauge
 * =======================================================================================================================
 */
#define DISTANCEGAUGE   A4
#define DG_BUCKETS      91        // Observation Distance Buckets

// Extern variables
extern float dg_adjustment;
extern volatile unsigned int anemometer_interrupt_count;
extern uint64_t anemometer_interrupt_stime;

extern bool AS5600_exists;
extern int AS5600_ADR;

// Function prototype
void anemometer_interrupt_handler();
void as5600_initialize();
float Wind_SampleSpeed();
int Wind_SampleDirection();
int Wind_DirectionVector();
float Wind_SpeedAverage();
float Wind_Gust();
int Wind_GustDirection();
void Wind_GustUpdate();
void Wind_TakeReading();
void Wind_Distance_Air_Initialize();

int OBS_Distance_SimpleMedian();
void OBS_Distance_Do();
float OBS_Distance_Median();
void OBS_Distance_Calc(float *m, float *d, int *o);
void OBS_WindAndDistance_Fill();

