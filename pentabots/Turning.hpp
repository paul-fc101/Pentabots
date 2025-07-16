#pragma once

#include <Arduino.h>

#include "Wire.h"
#include <MPU6050_light.h>
#include "Motor.hpp"

#define ROTTOL 1 // degrees
#define ROTTOL2 10 // degrees
#define ROTSPEED 30
#define ROTSPEED2 70

namespace mtrn3100 {

// The Turning class is designed to control the turning of a robot using encoders and motors.
class Turning {
public:
    Turning(Motor mot1, Motor mot2, MPU6050& mpuRef)
        : motor1(mot1), motor2(mot2), mpu(mpuRef) {
        motor1.setPWM(0);
        motor2.setPWM(0);
    }

    void setMPUInitRot() {
        mpu.update();
        initRot = mpu.getAngleZ() - 90;
    }

    void attachMPU() {
        mpu.begin();
    }

    // void clockwiseTurn(double angle) {
      
    // }


    void turningCorrection() {
      mpu.update();
      this->currRot = mpu.getAngleZ();
      Serial.println(initRot);
      Serial.println(currRot);
      if (currRot < initRot - ROTTOL2) {
        motor1.setPWM(ROTSPEED2);
        motor2.setPWM(ROTSPEED2);
      } else if (currRot > initRot + ROTTOL2) {
        motor1.setPWM(-ROTSPEED2);
        motor2.setPWM(-ROTSPEED2);
      } else if (currRot < initRot - ROTTOL) {
        motor1.setPWM(ROTSPEED);
        motor2.setPWM(ROTSPEED);
      } else if (currRot > initRot + ROTTOL) {
        motor1.setPWM(-ROTSPEED);
        motor2.setPWM(-ROTSPEED);
      } else if ((currRot >= initRot - ROTTOL) || (currRot <= initRot + ROTTOL)) {
        motor1.setPWM(0);
        motor2.setPWM(0);
      }
    }

    // void anticlockwiseTurn(double angle) {
      
    // }

private:
    MPU6050 mpu;
    Motor motor1;
    Motor motor2;
    float initRot;
    float currRot;

};
}

