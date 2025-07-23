#pragma once

#include <Arduino.h>

#include "Wire.h"
#include <MPU6050_light.h>
#include <VL6180X.h>

#include "Motor.hpp"
#include "MovingAverageFilter.hpp"

#include "EncoderOdometry.hpp"
#include "DualEncoder.hpp"
#include "PIDController.hpp"

#define WHEEL_DIAM 31.5
#define WHEEL_BASE 91
#define RIGHT_CORRECT 1.031
#define LEFT_CORRECT 1
#define DIST_TOLLERANCE 1
#define CELL_DIST 180


mtrn3100::MovingAverageFilter filter;

namespace mtrn3100 {

class Driving {
public:
    Driving(Motor mot1, Motor mot2, VL6180X& senRef, DualEncoder& enc, EncoderOdometry& odom)
        : motor1(mot1), motor2(mot2), FrontSensor(senRef), encoder(enc), encoderOdometer(odom), pidL(PIDController(1.5, 0.5, 0.6)), pidR(PIDController(1.5, 0.5, 0.6)) {
        motor1.setPWM(0);
        motor2.setPWM(0);
    }

    void init() {
      FrontSensor.init();
      FrontSensor.configureDefault();
      FrontSensor.setTimeout(300);
    }


    void drive(int numCells) {
      // 1 Cell is 180 mm

      encoderOdometer.update(
          encoder.getLeftRotation(),
          encoder.getRightRotation()
      );

      float leftDist = encoder.getLeftDistance(WHEEL_DIAM / 2.0);
      float rightDist = encoder.getRightDistance(WHEEL_DIAM / 2.0);
      float startLeft = leftDist;
      float startRight = rightDist;
      pidL.zeroAndSetTarget(leftDist, leftDist + CELL_DIST * numCells);
      pidR.zeroAndSetTarget(rightDist, rightDist - CELL_DIST * numCells);

      float MaxEndL = DIST_TOLLERANCE + startLeft + CELL_DIST * numCells;
      float MinEndL = - DIST_TOLLERANCE + startLeft + CELL_DIST * numCells;
      float MaxEndR = DIST_TOLLERANCE + startRight - CELL_DIST * numCells;
      float MinEndR = - DIST_TOLLERANCE + startRight - CELL_DIST * numCells;

      while (
        leftDist < MinEndL
        || rightDist < MinEndR
        || leftDist > MaxEndL
        || rightDist > MaxEndR
      ) {
        
        encoderOdometer.update(
            encoder.getLeftRotation(),
            encoder.getRightRotation()
        );

        leftDist = encoder.getLeftDistance(WHEEL_DIAM / 2.0);
        rightDist = encoder.getRightDistance(WHEEL_DIAM / 2.0);

        float correctionL = pidL.compute(leftDist);

        float correctionR = pidR.compute(rightDist);
        
        if (leftDist < MinEndL || leftDist > MaxEndL) {
          if (correctionL < 0 && correctionL > -15) {
            motor2.setPWM(15);
          } else if (correctionL > 0 && correctionL < 15) {
            motor2.setPWM(-15);
          } else {
            motor2.setPWM(-correctionL);
          }
        } else {
          motor2.setPWM(0);
        }
        if (rightDist < MinEndR || rightDist > MaxEndR) {
          if (correctionR < 0 && correctionR > -15) {
            motor1.setPWM(15);
          } else if (correctionR > 0 && correctionR < 15) {
            motor1.setPWM(-15);
          } else {
            motor1.setPWM(-correctionR);
          }
        } else {
          motor1.setPWM(0);
        }
      }
    }

    bool drivingCorrection(bool isTurning) {
      if (isTurning) {
        return false;
      }
      uint16_t sensorValue = FrontSensor.readRangeSingleMillimeters() - 3;
      uint16_t value = filter.average(sensorValue);

      int pwm = 0;

      if (FrontSensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

      if (value > 103) {
        pwm = value - 83;
      } else if (value < 97) {
        pwm = value - 117;
      }

      motor1.setPWM(pwm * RIGHT_CORRECT);
      motor2.setPWM(-pwm * LEFT_CORRECT);
      if (pwm == 0) {
        return false;
      }
      return true;
    }

private:
    VL6180X FrontSensor;
    DualEncoder& encoder;
    EncoderOdometry& encoderOdometer;
    Motor motor1; // Right Motor
    Motor motor2; // Left Motor
    PIDController pidR;
    PIDController pidL;
};
}

