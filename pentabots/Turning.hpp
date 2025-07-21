#pragma once

#include <Arduino.h>

#include "Wire.h"
#include <MPU6050_light.h>

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
    Turning(Motor mot1, Motor mot2, MPU6050& mpuRef)
        : motor1(mot1), motor2(mot2), mpu(mpuRef), pid(PIDController(1.8, 0.7, 0.3)) {
        motor1.setPWM(0);
        motor2.setPWM(0);
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
        delay(500);
        float endAngle = mpu.getAngleZ();
        float imuEnd = millis();
        //Serial.print("Start Angle: "); Serial.println(startAngle); Serial.println(imuStart);
        
        //Serial.print("End Angle: "); Serial.println(endAngle); Serial.println(imuEnd); 
        drift = (endAngle - startAngle) / (imuEnd - imuStart);
        //Serial.print("IMU Drift: "); Serial.println(drift);
    }

    bool turningCorrection(bool isDriving) {
      if (isDriving) {
        return false;
      }
      mpu.update();
      float elapsedTime = millis() - prevTime;
      bool isChanging = true;
      float ang = mpu.getAngleZ() + drift * elapsedTime;
      //Serial.print("ang: "); Serial.println(ang);
      prevTime = millis();

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

    // void turn(double angle) {
    //   desiredRot = currRot + angle;
    //   Serial.print("Desired: ");
    //   Serial.println(desiredRot);
    //   Serial.print("currRot: ");
    //   Serial.println(currRot);
    //   bool isChanging = true;
    //   int i = 0;
    //   while ((currRot > desiredRot + ROTTOL || currRot < desiredRot - ROTTOL) || isChanging) {
    //     isChanging = turningCorrection(false);
    //     if (i % 20 == 0) {
    //       Serial.print("currRotIn: ");
    //       Serial.println(currRot);
    //     }
    //     i++;
    //   }

    // }

    void turn(float angle) {

      mpu.update();
      float elapsedTime = millis() - prevTime;
      bool isChanging = true;
      currRot = mpu.getAngleZ() + drift * elapsedTime;
      prevTime = millis();

      desiredRot = floor(10 * currRot + 0.5f) / 10 + angle;
      pid.zeroAndSetTarget(currRot, desiredRot);
      bool isTurning = true;
      // Serial.print("desired Rot: ");
      // Serial.println(desiredRot);
      while (isTurning) {
        mpu.update();
        currRot = mpu.getAngleZ();

        //if (currRot >= 6553) currRot -= 13106;

        float pwm = pid.compute(currRot);
        if (pwm < 0 && pwm > -15) {
          motor2.setPWM(-15);
        } else if (pwm > 0 && pwm < 15) {
          motor2.setPWM(15);
        } else {
          motor2.setPWM(pwm);
        }
        // Serial.print("Curr Rot: ");
        // Serial.println(currRot);
        // Serial.print("pwm/output: ");
        // Serial.println(pwm);
        

        motor1.setPWM(pwm);
        motor2.setPWM(pwm);

        Serial.print("Error: "); Serial.println(pid.getError());
        Serial.println("");
        if (fabs(pid.getError()) < ROTTOL) {
            isTurning = false;
            motor1.setPWM(0);
            motor2.setPWM(0);
        }

        //delay(10); // Small delay for smoother loop
      }
}


private:
    MPU6050 mpu;
    Motor motor1;
    Motor motor2;
    float initRot;
    float desiredRot;
    float currRot;
    long lastUpdate = 0;
    int numOffset;
    PIDController pid;
    float drift;
    float prevTime;
};
}

