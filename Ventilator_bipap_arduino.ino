#include <Wire.h>

// Absolute pressure sensor BMP280 library
// 3.3 V !!! BMP280 
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C
int pressure_sensor_works = 1;
// Nastavitve hitrosti zajemanja meritev
float readAbsolutePressure() {
  //Read BMP-280 sensor abs pressure value [Pa]
  if (pressure_sensor_works) {
    return bmp.readPressure();
  }
}

// Declaring functions, but they're deprecated
void runCPAPCycle(float cycleLength_s=6, float maxThrottle=30 );
void runBipapCycle(float cycleLength_s=6, float maxThrottle=30, float minThrottle=25 );
void runHfovCycle(float frequency=1, float maxThrottle=35, float minThrottle=30 );

// ESC pin settings
int pos = 0; //Throttle position
float pos_float = 0;
int ESC_pin = 9; // ESC pin
// For arming ESC - first analogWrite(0) then analogWrite(128) 

int maxAllowableThrottle = 170; // 128 is zero throttle, 255 is full throttle
void setThrottleSafe(int desiredThrottle){
  // Safer way to set throttle, in case the PID coefficients are bad and want to set turbine to 100% throttle.
   if (desiredThrottle<maxAllowableThrottle) {
    analogWrite(ESC_pin, desiredThrottle);
    }
   else {
    analogWrite(ESC_pin,maxAllowableThrottle);}
  }

//MPX5010DP pressure difference sensor on pin A0. The sensor runs on 5V. Check datasheet because some capacitors are required to reduce noise.
int sensorValue;
int readPressureDifference() {
  //Equation from MPX5010DP datasheet
  // Vsupply = 5V
  // Vout = Vsuppy * (0.09*P + 0.04) +- 5% V_FSS
  // P = differential_pressure (kPa) . P1>P2
  // Vout = 0V - > analogRead output 0 . Vout=5V, analogRead output 1023
  sensorValue = analogRead(A0);
  //Serial.println(sensorValue);
  return sensorValue;
}
int flow_sensor_works = 0;

//PID settings. Be careful; coefficients should be different depending on if you use pressure-control or flow-control
//double kp = 0.25;
double kp = 0.4;
double ki = 0;
double kd = 0.000001;
 
unsigned long currentTime,previousTime;
double error;
double lastError;
double input, output;
double volatile setPoint;
double cumError, rateError;
double out;

// Comment : the 100 next to cumError and the 1000 in rateError are values dependent on PID-calling frequency (so dependent on TIMER). I just made smthn up.
// The right thing would be to deterime relationship between TIMER and pid frequency and set the values according to that.
double PID(double inp,float Setpoint){             
        error = Setpoint - inp;    // determine error
        cumError += error * 100;                // compute integral
        rateError = (error - lastError)/1000;   // compute derivative
        if (cumError>10) {cumError=10;}
        if (cumError<-10) {cumError=-10;}
        out = kp*error + ki*cumError + kd*rateError;  //PID output               
 
        lastError = error;    //remember current error
        // analogWrite(128) means ESC throttle is set to 0. AnalogWrite(255) means its 100%. So, add 120 to make it neutral, if "out" is 0. 
        // The max() is so output can't be lower than 120 (that's invalid signal to ESC.
        // The min() is so PID output can't be greater than 255 (or else integer overflows and serial conn dies)
        return min(252,120+max(out,0));    //have function return the PID output
}

volatile byte led_state = LOW;
volatile float control;
volatile int control_int;

long abs_pressure;

int remapped_pressure; // For sending 0-255 over serial. Accuracy is lost here, because sensor outputs 0-1024. If desired, code can be added to allow python program to read
// 2 bytes in a row for increased accuracy.
long absolute_pressure_at_start = 0;


 // maximum pressure in human lungs is (i think) around 1.08 bar, which is 108000 Pa. Here we are writing RELATIVE pressure, so max 8000 Pa (if we assume atmo is 100000 Pa pressure) 
  ////////////////////////////////////////////////////////////////// SET THIS TO DESIRED RELATIVE PRESSURE (overpressure) in [Pa]. By changing this setting in the loop() you can achieve BiPAP


int BIPAP_overpressure_high = 1000; // [Pa] (relative pressure)    /// SET THIS
int BIPAP_overpressure_low = 100; // [Pa]                          /// SET THIS
float breath_frequency_per_minute = 12;
float cycle_time = 60 / breath_frequency_per_minute;
float PCT_INHALE_TIME = 0.2;  /// How many percent of TOTAL CYCLE TIME is INHALATION.  (so PCT_INHALE_TIME + PCT_EXHALE_TIME == 1)
float millis_length_inhale = cycle_time*PCT_INHALE_TIME;
float millis_length_exhale = cycle_time* (1-PCT_INHALE_TIME);


long target_relative_pressure=BIPAP_overpressure_high; // First "desired value" is BIPAP pressure high
long relative_pressure;


// You want your PID to run in regular intervals. To do that, instead of using millis() (slow), attach an interrupt to a timer. 
// So, when certain amount of time passes, the PID will be called automatically. Check on the net for exact timings of timer.
volatile int run_pid_state = 0;
ISR(TIMER2_OVF_vect){
   run_pid_state += 1;
   // We run PID on every 4th timer overflow. (I made the number 4 up. To improve, check how often timer overflows). 
   // Reason : no sense in running PID much faster than the sensors update (since you'd be running it on old invalid data)
   if (run_pid_state>4) {
      //control = PID(sensorValue,target_flow) + 30;
      control = PID(relative_pressure,target_relative_pressure);
      // The actual throttle in set in the main loop()
      //control_int=int(control);
      //setThrottleSafe(control_int);
      run_pid_state = 0;
      }
    }

void setup() {
   // Check on the internet. Based on what you write here is how often your timer interrupt will be activated.
   TIMSK2 = (TIMSK2 & B11111110) | 0x01;
   TCCR2B = (TCCR2B & B11111000) | 0x07;
   
    previousTime=millis();
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    //Arming the ESC
   analogWrite(ESC_pin,0);
   delay(1000);
   analogWrite(ESC_pin,128);
   delay(2000);
   //analogWrite(ESC_pin,160);
   //delay(2000);
   digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

  pos_float=0;
  // Starting the absolute pressure sensor (BMP280)
  if (!bmp.begin(0x76)) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
      pressure_sensor_works = 0;}
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_125); /* Standby time. */
   delay(100);
   absolute_pressure_at_start=readAbsolutePressure();
} 
  
unsigned long last_sensor_reading_time = millis();
int read_sensors_this_loop = 0;
int bipap_time = millis();

void loop() {

  currentTime = millis();
  // IF more than 0.15 seconds passed, read sensor data
  //Serial.println(currentTime);
  //Serial.println(last_sensor_reading_time);

  if (currentTime-last_sensor_reading_time>150) {
   
      // Read sensor info
      sensorValue = readPressureDifference();
      relative_pressure = readAbsolutePressure()-absolute_pressure_at_start; // In pascals
      
      read_sensors_this_loop = 1;
      //Serial.println(last_sensor_reading_time);
      }


    if ((currentTime-bipap_time) < millis_length_inhale*1000) {target_relative_pressure = BIPAP_overpressure_high;}
    if ((currentTime-bipap_time) > millis_length_inhale*1000) {target_relative_pressure=BIPAP_overpressure_low;}
    if ((currentTime-bipap_time) > cycle_time*1000) {bipap_time = millis();}



    // The PID runs continuously, so you only need to take the last generated throttle signal
    // apply throttle
    control_int=int(control);
    setThrottleSafe(control_int);

  // Send sensor and throttle data over sensors
    if (read_sensors_this_loop) {
      Serial.println("253");
      //For sending over serial, the relative pressure must be remapped. So, based on this, you have to set the conversion 0-255 to pressure in [Pa] in python script.
      remapped_pressure = map(relative_pressure,-1000,8000,0,255);
      if (remapped_pressure > 252) {remapped_pressure = 252;}
      // values 253-255 are reserved for signal type data(pressure-253 , airflow -254 , control -255)
      Serial.println(remapped_pressure);
      // You can't send "long" directly over serial without adding some extra code to allow python script to read it.
      //Serial.println(int(relative_pressure));

      //Serial.print("Air flow: "); 
      Serial.println("254");  
      Serial.println(sensorValue);

      //Send control data over serial
      Serial.println("255");
      Serial.println(control_int);
      read_sensors_this_loop=0;
      last_sensor_reading_time=currentTime;}

    
}


// ALL CODE DOWN HERE IS DEPRECATED. Gotta program the CPAP and BIPAP in a non-blocking way.
int control_program_state = 0;
float control_program_millis = 0;
int target_flow_rate=0;
long timer_n  = 0;

int flow_sensor_neutral_signal = 42; // In my experience - if MPX5010 flow sensor output is 42, then P1 == P2 (and airflow is 0)
int runFlowCycle(float frequency, float max_flow, float inhale_pct) {
 //if (control_program_state == 0)
  // Runs flow control cycle . V prvem delu volumski pretok pada od vmax do 0. Na izdihu je volumski pretok 0 (oziroma sensor daje signal 41)
  // frequency how many breaths per minute
  // max flow - starting flow rate in inhale cycle
  // inhale_pct =100* t_inhale / t_cycle     (t_cycle = t_inhale + t_exhale)
  timer_n +=1; 

  if (timer_n < 1000) {control = PID(sensorValue,max_flow) + 30; // Za standgas 20
  return int(control);
    }
  
  if (timer_n > 1000) {timer_n=0;return int(control);}
  
  }


// old code, instead of setting throttle directly, desired upper and lower pressure should be set, then PID controlled.
float delayTime;
void runHfovCycle(float frequency=1, float maxThrottle=35, float minThrottle=30 ){
  // delta pressure ( med najnizjim in najvisjim pritiskom - nastavimo z maxThrottle in minThrottle
  // Frekvenca HFOV je priblizno 30 na minuto (ali vec !) - torej 0.5 Hz
  // Stevilo "Tranzicij" med throttle nivoji = int(maxThrottle-minThrottle)*2
  // Cas trajanja cikla to = 1 / frekvenca
  // delayTime preklaplanja med throttle nivoji je torej (cas_trajanja_cikla/stevilo_tranzicij)*1000
  delayTime = 1000*(1/frequency)/(2*int(maxThrottle-minThrottle));
  for (pos = minThrottle; pos <= maxThrottle; pos += 1) { 
    setThrottleSafe(pos);             
     delay(delayTime);
  }
  for (pos = maxThrottle; pos >= minThrottle; pos -= 1) { 
    setThrottleSafe(pos);             
     delay(delayTime);
  }
  
  }


void runCPAPCycle(float cycleLength_s=6, float target_relative_pressure_hi=30 ) {
  target_relative_pressure = target_relative_pressure_hi;
}

// Old code, controls throttle. Should be replaced with PID control of pressure
void runBipapCycle(float cycleLength_s=6, float maxThrottle=30, float minThrottle=25 ) {
  float motor_rise_time_s = 0.5;
  int motor_spinup_delay = int(motor_rise_time_s/maxThrottle) * 1000;
  
  for (pos = 0; pos <= maxThrottle; pos += 1) { 
    setThrottleSafe(pos);             
    delay(motor_spinup_delay);
  }
  delay(1000* (cycleLength_s/2 -motor_rise_time_s));

  float motor_fall_time_s = 0.5;
  int motor_spindown_delay = int(motor_fall_time_s/(maxThrottle-minThrottle)) * 1000;

  for (pos = maxThrottle; pos >= minThrottle; pos -= 1) { 
    setThrottleSafe(pos);             
     delay(motor_spindown_delay);
  }
  delay(1000*(cycleLength_s/2 - motor_fall_time_s));
  }
