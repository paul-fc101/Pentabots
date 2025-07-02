#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "BangBangController.hpp"

#define MOT1PWM 9 // PIN 9 is a PWM pin
#define MOT1DIR 10
#define MOT2PWM 11
#define MOT2DIR 12
mtrn3100::Motor motor(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);

#define EN_A 2 // PIN 2 is an interupt
#define EN_B 4
#define FINAL_DIST 200
#define WHEEL_DIAM 32
#define MATH_PI 3.1415
mtrn3100::Encoder encoder(EN_A, EN_B);

mtrn3100::BangBangController controller(120,0);


void setup() {
  Serial.begin(9600);
  float numRotations = WHEEL_DIAM * MATH_PI / FINAL_DIST;
  controller.zeroAndSetTarget(encoder.getRotation(), numRotations * 2 * MATH_PI);
  controller.tune(1,0);
  float b = controller.compute(encoder.getRotation());
  motor.setPWM(0);
  motor2.setPWM(0);
  motor.setPWM(-b);
  motor2.setPWM(b);
  delay(1200);
  motor.setPWM(0);
  motor2.setPWM(0);
}

void loop() {
  // float b = controller.compute(encoder.getRotation());
  // motor.setPWM(b);
  // motor2.setPWM(-b);
  //Serial.println(controller.error);
  //Serial.println(b);
  // motor.setPWM(125);
  // delay(1000);
  // motor.setPWM(0);
  // delay(3000);
}
