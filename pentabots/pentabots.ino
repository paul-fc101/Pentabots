#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "BangBangController.hpp"
#include "DualEncoder.hpp"

#include "Wire.h"
#include <MPU6050_light.h>

#define MOT1PWM 9 // PIN 9 is a PWM pin
#define MOT1DIR 10
#define MOT2PWM 11
#define MOT2DIR 12
// Rotation Tollerances in degrees
#define ROTTOL1 0.1
#define ROTTOL2 0.5
#define ROTTOL3 3
// Rotation Speeds
#define ROTSPEED1 5
#define ROTSPEED2 20
#define ROTSPEED3 75

// Initalise
mtrn3100::Motor motor(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);
MPU6050 mpu(Wire);


#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder
#define FINAL_DIST 200
#define WHEEL_DIAM 32
#define MATH_PI 3.1415

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::BangBangController controller(120,0);

float initRot;
float currRot;
float prevRot;
float prev2Rot;

void setup() {
  
  Serial.begin(9600);

  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  mpu.calcOffsets(); 
  initRot = mpu.getAngleZ();
  currRot = initRot;
  prevRot = initRot;
  prev2Rot = initRot;
}

void loop() {
  mpu.update();
  currRot = mpu.getAngleZ();
  float accRot = (currRot + prevRot + prev2Rot) / 3;

  int diff = accRot - initRot;
  int pwm = 0;

  if (diff < -ROTTOL3) {
    pwm = ROTSPEED3;
  } else if (diff > ROTTOL3) {
    pwm = -ROTSPEED3;
  } else if (diff < -ROTTOL2) {
    pwm = ROTSPEED2;
  } else if (diff > ROTTOL2) {
    pwm = -ROTSPEED2;
  } else if (diff < -ROTTOL1) {
    pwm = ROTSPEED1;
  } else if (diff > ROTTOL1) {
    pwm = -ROTSPEED1;
  }

  motor.setPWM(pwm);
  motor2.setPWM(pwm);

  prev2Rot = prevRot;
  prevRot = currRot;
}
