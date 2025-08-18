#pragma once

#include <Arduino.h>

#include "Wire.h"
#include <MPU6050_light.h>
#include <VL6180X.h>

#include "Motor.hpp"
#include "MovingAverageFilter.hpp"
#include "PIDController.hpp"

#define ROTTOL 1 // degrees
#define ROTTOL2 10 // degrees
#define ROTSPEED 30
#define ROTSPEED2 70

#define RIGHT_CORRECT 1.031
#define LEFT_CORRECT 1

mtrn3100::MovingAverageFilter filterT;

namespace mtrn3100 {

class Turning {
public:
    Turning(Motor& mot1, Motor& mot2, MPU6050& mpuRef, VL6180X& frontSen, VL6180X& leftSen, VL6180X& rightSen)
        : motor1(mot1), motor2(mot2), mpu(mpuRef), 
          pid(PIDController(1.8, 0.7, 0.3)),
          pidCenterH(PIDController(0.8, 0.0, 0.2)), // PID for horizontal (pivot) centering
          pidCenterV(PIDController(1.2, 0.1, 0.3)), // PID for vertical (fwd/back) centering
          FrontSensor(frontSen), LeftSensor(leftSen), RightSensor(rightSen) 
    {
        motor1.setPWM(0);
        motor2.setPWM(0);
    }

    void centerRobot() {
        centerHorizontally();
        motor1.stop();
        motor2.stop();
    }

    void setMPUInitRot() {
        mpu.update();
        initRot = mpu.getAngleZ();
    }

    void attachMPU() {
        byte status = mpu.begin();
        Serial.print(F("MPU6050 status: "));
        Serial.println(status);
        delay(200);
        mpu.calcOffsets();
        delay(200);
        float startAngle = mpu.getAngleZ();
        float imuStart = millis();
        delay(1000);
        float endAngle = mpu.getAngleZ();
        float imuEnd = millis();
        
        drift = (endAngle - startAngle) / (imuEnd - imuStart);
    }

    bool turningCorrection(bool isDriving) {
      if (isDriving) {
        return false;
      }
      
      mpu.update();
      float elapsedTime = millis() - prevTime;
      bool isChanging = true;
      float ang = mpu.getAngleZ();// + drift * elapsedTime;
      Serial.println(ang);
      Serial.println(desiredRot);
      prevTime = millis();
      Serial.println(desiredRot);
      Serial.println(ang);
      currRot = ang;
      if (currRot < desiredRot - ROTTOL2) {
        motor1.setPWM(ROTSPEED2);
        motor2.setPWM(ROTSPEED2);
      } else if (currRot > desiredRot + ROTTOL2) {
        motor1.setPWM(-ROTSPEED2);
        motor2.setPWM(-ROTSPEED2);
      } else if (currRot < desiredRot - ROTTOL) {
        motor1.setPWM(ROTSPEED);
        motor2.setPWM(ROTSPEED);
      } else if (currRot > desiredRot + ROTTOL) {
        motor1.setPWM(-ROTSPEED);
        motor2.setPWM(-ROTSPEED);
      } else if ((currRot >= desiredRot - ROTTOL) && (currRot <= desiredRot + ROTTOL)) {
        if (!isDriving) {
          motor1.setPWM(0);
          motor2.setPWM(0);
        }
        isChanging = false;
      }

      return isChanging;
    }

    void turn(float angle) {

      mpu.update();
      float elapsedTime = millis() - prevTime;
      bool isChanging = true;
      currRot = mpu.getAngleZ() + drift * elapsedTime;
      prevTime = millis();

      desiredRot = floor(10 * currRot + 0.5f) / 10 + angle;
      pid.zeroAndSetTarget(currRot, desiredRot);
      bool isTurning = true;
      while (isTurning) {
        mpu.update();
        currRot = mpu.getAngleZ();

        float pwm = pid.compute(currRot);
        if (pwm < 0 && pwm > -15) {
          motor2.setPWM(-15);
        } else if (pwm > 0 && pwm < 15) {
          motor2.setPWM(15);
        } else {
          motor2.setPWM(pwm);
        }

        motor1.setPWM(pwm);
        motor2.setPWM(pwm);

        if (fabs(pid.getError()) < ROTTOL) {
            isTurning = false;
            motor1.stop();
            motor2.stop();
        }

      }
      motor1.stop();
      motor2.stop();
    }

    void getLidar(VL6180X front, VL6180X left, VL6180X right) {
        FrontSensor = front;
        LeftSensor = left;
        RightSensor = right;
    }

    void saveRot() {
      mpu.update();
      float elapsedTime = millis() - prevTime;
      desiredRot = mpu.getAngleZ(); //+ drift * elapsedTime;
      prevTime = millis();
    }

private:

    void centerHorizontally() {
        const uint8_t CENTER_TOLERANCE = 1;
        const uint8_t WALL_MAX_DIST = 140;
        pidCenterH.zeroAndSetTarget(0, 0);

        float li_L = LeftSensor.readRangeSingleMillimeters();
        float li_R = RightSensor.readRangeSingleMillimeters();
        float li_F = FrontSensor.readRangeSingleMillimeters();
        if (li_F < 55) {
          while (li_F < 55) {
            li_F = FrontSensor.readRangeSingleMillimeters();
            motor2.setPWM(30);
            motor1.setPWM(-30);
          }
          motor2.stop();
          motor1.stop();
        }
        if (li_R < WALL_MAX_DIST || li_L < WALL_MAX_DIST ) {
          if (li_R <= 35) {
            moveLeftForwardBack();
          } else if (li_L <= 35) {
            moveRightForwardBack();
          }
        }

        motor1.stop();
        motor2.stop();
    }

    void moveLeftForwardBack() {
      motor2.setPWM(-50);
      motor1.setPWM(50);
      delay(300);
      motor1.setPWM(50);
      motor2.setPWM(50);
      delay(400);
      motor2.setPWM(-50);
      motor1.setPWM(0);
      delay(720);
      motor2.setPWM(50);
      motor1.setPWM(-50);
      delay(600);
      motor1.setPWM(0);
      motor2.setPWM(0);
    }

    void moveRightForwardBack() {
      motor2.setPWM(-50);
      motor1.setPWM(50);
      delay(300);
      motor1.setPWM(-50);
      motor2.setPWM(-50);
      delay(400);
      motor1.setPWM(50);
      motor2.setPWM(0);
      delay(720);
      motor1.setPWM(-50);
      motor2.setPWM(50);
      delay(600);
      motor1.setPWM(0);
      motor2.setPWM(0);
    }


    MPU6050 mpu;
    Motor motor1; // Right Motor
    Motor motor2; // Left Motor
    float initRot;
    float desiredRot;
    float currRot;
    long lastUpdate = 0;
    int numOffset;
    PIDController pid, pidCenterH, pidCenterV;
    float drift;
    float prevTime;
    VL6180X FrontSensor;
    VL6180X LeftSensor;
    VL6180X RightSensor;
    float savedRot = 0;
};
}

