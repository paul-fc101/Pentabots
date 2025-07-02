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
#define ROTTOL 0.5 // radians
#define ROTSPEED 20

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
//mtrn3100::Encoder encoder(EN_A, EN_B);
mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::BangBangController controller(120,0);
float initRot;

void setup() {
  
  Serial.begin(9600);

  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  mpu.calcOffsets(); 
  initRot = mpu.getAngleZ();
  
  // float numRotations = WHEEL_DIAM * MATH_PI / FINAL_DIST;
//  controller.zeroAndSetTarget(encoder.getRotation(), numRotations * 2 * MATH_PI);
//  controller.tune(1,0);
//  
//  float b = controller.compute(encoder.getRotation());
////
//  
//  motor.setPWM(0);
//  motor2.setPWM(0);
//  motor.setPWM(b);
//  motor2.setPWM(b);
//  delay(1200);
//  motor.setPWM(0);
//  motor2.setPWM(0);
}

void loop() {
  mpu.update();
  // float b = controller.compute(encoder.getRotation());
  float currRot = mpu.getAngleZ();
  Serial.println(initRot);
  Serial.println(currRot);
  if (currRot < initRot - ROTTOL ) {
    motor.setPWM(ROTSPEED);
    motor2.setPWM(ROTSPEED);
  } else if (currRot > initRot + ROTTOL) {
    motor.setPWM(-ROTSPEED);
    motor2.setPWM(-ROTSPEED);
  } else if ((currRot >= initRot - ROTTOL) || (currRot <= initRot + ROTTOL)) {
    motor.setPWM(0);
    motor2.setPWM(0);
  }
  
  // motor.setPWM(b);
  // motor2.setPWM(-b);
  //Serial.println(controller.error);
  //Serial.println(b);
  // motor.setPWM(125);
  // delay(1000);
  // motor.setPWM(0);
  // delay(3000);
}
