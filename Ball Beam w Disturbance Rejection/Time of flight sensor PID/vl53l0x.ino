#include "Adafruit_VL53L0X.h"
#include <Servo.h>
#define SERVO_PIN 10
Servo servo;
float TF = 0.4 ;
int CurrentDistance;
float PreviousDistance;

float elapsedTime, time, timePrev;        //Variables for time control
float distance_previous_error, distance_error;
int period = 12;  //Refresh rate period of the loop is 50ms
double filteredDerivative = 0.0;
double alpha = 3.95244837854835;

float kp=5; //Mine was 8 0.0903580249872015
float ki=0.25; //Mine was 0.2 and 0.0903580249872015
float kd=800;//Mine was 180.46529  and 750.182308618545



float distance_setpoint = 130;           //Should be the distance from sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d, PID_total;
int Previousdistance;


Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
servo.attach(SERVO_PIN);
  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}


void loop() {
  if(millis() > time + period) {
  time = millis();

  VL53L0X_RangingMeasurementData_t measure;

  CurrentDistance = PreviousDistance*(1-TF) + measure.RangeMilliMeter*TF;
  PreviousDistance = CurrentDistance + .012;
  
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(CurrentDistance); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
   
  }
    
//  delay(100);

 
// Controller (PID) calculations
distance_error = distance_setpoint - CurrentDistance;
PID_p = kp * distance_error;
  float dist_diference = distance_error - distance_previous_error;     
    float distance_previous;
    float tau = 25;
    // Filtered derivative calculation with low pass filter
    PID_d = 2*kd*(distance_error - distance_previous_error)/(2*tau +period)+(2*tau-period)*(PID_d)/(2*tau+period);
    
  if(-40 < distance_error && distance_error < 40) {
    PID_i = PID_i + ki*distance_error;
    PID_i = constrain(PID_i, -100,100);
  }    else
    PID_i = 0 ;

  int top_ang = 1750;
  int low_ang = 1200;
  
  PID_total = PID_p+PID_i+PID_d;
  PID_total = map(PID_total,-260,250, top_ang, low_ang);
////  Controller saturation
      if(PID_total < low_ang){PID_total = low_ang;}
      if(PID_total > top_ang) {PID_total = top_ang; } 
  
    servo.writeMicroseconds(PID_total);  
    distance_previous_error = distance_error;
    distance_previous = measure.RangeMilliMeter;
}

//    Serial.println(PID_p+PID_i+PID_d);
    Serial.print(",");
       // Serial.println(distance_setpoint);
        Serial.println(PID_total);
}
