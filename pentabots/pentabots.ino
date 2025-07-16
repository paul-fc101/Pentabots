#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "BangBangController.hpp"
#include "DualEncoder.hpp"
#include "Turning.hpp"
#include "Driving.hpp"

#include "Wire.h"
#include <MPU6050_light.h>
#include <VL6180X.h>

#define MOT1PWM 9 // PIN 9 is a PWM pin
#define MOT1DIR 10
#define MOT2PWM 11
#define MOT2DIR 12

// Initalise
mtrn3100::Motor motor(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
MPU6050 mpu(Wire);

#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder
#define FINAL_DIST 200
#define WHEEL_DIAM 32
#define MATH_PI 3.1415
//mtrn3100::Encoder encoder(EN_A, EN_B);
VL6180X FrontSensor;

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::BangBangController controller(120,0);
mtrn3100::Turning turnController(motor, motor2, mpu);
mtrn3100::Driving driveController(motor, motor2, FrontSensor);


void setup() {
  Serial.begin(9600);

  Wire.begin();
  byte status = mpu.begin();
  delay(300);
  turnController.attachMPU();
  
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  mpu.calcOffsets();
  turnController.setMPUInitRot();


  driveController.init();
}


bool isDriving = false;
bool isTurning = true;
void loop() {
  // isDriving = driveController.drivingCorrection(isTurning);
  // isTurning = turnController.turningCorrection(isDriving);

  //driveController.drivingCorrection(false);
  turnController.turningCorrection(false);
  
}





