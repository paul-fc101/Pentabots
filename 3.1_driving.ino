#include "Encoder.hpp"
#include "Motor.hpp"
#include "MovingAverageFilter.hpp"

#include <Wire.h>
#include <VL6180X.h>

#define MOT1PWM 9 // PIN 9 is a PWM pin
#define MOT1DIR 10
#define MOT2PWM 11
#define MOT2DIR 12

// Rotation Tolerances in degrees
#define ROTTOL1 0.5
#define ROTTOL2 0.75
#define ROTTOL3 3
// Rotation Speeds
#define ROTSPEED1 5
#define ROTSPEED2 20
#define ROTSPEED3 75

// Initalise
mtrn3100::Motor motor(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);

#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder
#define FINAL_DIST 200
#define WHEEL_DIAM 32
#define MATH_PI 3.1415

VL6180X sensor;

mtrn3100::MovingAverageFilter filter;

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  
  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);
}

void loop() 
{ 
  uint16_t sensorValue = sensor.readRangeSingleMillimeters() - 3;
  uint16_t value = filter.average(sensorValue);

  int pwm = 0;

  Serial.print("Filter: ");
  Serial.println(value);
  Serial.print("Sensor: ");
  Serial.println(sensorValue);

  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  // // Linear Equation: Magnitude PWM from 27 to 180.
  // if (value > 102) {
  //   pwm = value - 75;
  // } else if (value < 98) {
  //   pwm = (153/98) * value - 180;
  // }

  // // Linear Equation: Magnitude PWM from 27 to 180.
  // if (value > 103) {
  //   pwm = value - 83;
  // } else if (value < 97) {
  //   pwm = (33/20) * value - 180;
  // }

  if (value > 103) {
    pwm = value - 83;
  } else if (value < 97) {
    pwm = value - 117;
  }

  motor.setPWM(pwm);
  motor2.setPWM(-pwm);
}

