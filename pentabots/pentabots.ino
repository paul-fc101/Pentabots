#include "Motor.hpp"
#include "DualEncoder.hpp"
#include "Turning.hpp"
#include "Driving.hpp"
#include "EncoderOdometry.hpp"
#include "Mapping.hpp"

#include "Wire.h"
#include <MPU6050_light.h>
#include <VL6180X.h>
#include "string.h"

#define MOT1PWM 9
#define MOT1DIR 10
#define MOT2PWM 11
#define MOT2DIR 12

// These are the pins for the PCB encoder
#define EN_1_A 2
#define EN_1_B 7
#define EN_2_A 3
#define EN_2_B 8

// Constants
#define FINAL_DIST 200
#define WHEEL_DIAM 32
#define WHEEL_BASE 91

#define RANGE 1


// Initalise
mtrn3100::Motor motor(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
MPU6050 mpu(Wire);

VL6180X FrontSensor, LeftSensor, RightSensor;

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoderOdometer(WHEEL_DIAM / 2, WHEEL_BASE);
mtrn3100::Driving driveController(motor, motor2, FrontSensor, LeftSensor, RightSensor, encoder, encoderOdometer);
mtrn3100::Turning turnController(motor, motor2, mpu, FrontSensor, LeftSensor, RightSensor);
mtrn3100::Mapping mapper(driveController, turnController);


void MovementString(String command) {
  for (int i = 0; i < command.length(); ++i) {
    char c = command.charAt(i);
    Serial.println(c);
    if (c == 'f') {
      encoder.l_count = 0;
      encoder.r_count = 0;
      encoderOdometer.reset();
      driveController.drive(1);
    } else if (c == 'l') {
      // anticlockwise
      turnController.centerRobot();
      delay(100);
      turnController.turn(90);
    } else if (c == 'r') {
      // clockwise
      turnController.centerRobot();
      delay(100);
      turnController.turn(-90);
    }
    delay(150);
  }
}

void setup() {
  Serial.begin(9600);

  Wire.begin();
  delay(300);
  turnController.attachMPU();

  turnController.setMPUInitRot();

  driveController.initalise_lidar();

  turnController.getLidar(FrontSensor, LeftSensor, RightSensor);
  

  //Uncomment for Task 3.2
  //turnController.turn(90);

  // Task 3.3
  MovementString(F("ffrflflfffflflfrfflffl"));

  // Task 4.3

}

bool isDriving = false;
bool isTurning = true;
void loop() {

  // Task 3.1 - Driving
  //driveController.drivingCorrection(false);

  // Task 3.2 - Turning
  //turnController.turningCorrection(false);

}
