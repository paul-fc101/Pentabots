#pragma once

#include <Arduino.h>

#include "Wire.h"
#include <MPU6050_light.h>

#include "Motor.hpp"
#include "MovingAverageFilter.hpp"

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
        : motor1(mot1), motor2(mot2), mpu(mpuRef) {
        motor1.setPWM(0);
        motor2.setPWM(0);
    }

    void setMPUInitRot() {
        mpu.update();
        initRot = mpu.getAngleZ();
    }

    void attachMPU() {
        mpu.begin();
    }

    bool turningCorrection(bool isDriving) {
      if (isDriving) {
        return false;
      }
      mpu.update();
      bool isChanging = true;
      float ang = mpu.getAngleZ();
      if (ang >= 6553) {
        ang = ang - 13106;
      }
    //   if (millis() - lastUpdate > 25) {
    //     lastUpdate = millis();
    //     ang += 0.1 * numOffset;
    //     numOffset++;
    //     Serial.println("adding offset");
    //   }
      currRot = ang;

      //this->currRot = filterT.average(ang);
      //Serial.println(currRot);
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

    void turn(double angle) {
      desiredRot = currRot + angle;
      Serial.print("Desired: ");
      Serial.println(desiredRot);
      Serial.print("currRot: ");
      Serial.println(currRot);
      bool isChanging = true;
      int i = 0;
      while ((currRot > desiredRot + ROTTOL || currRot < desiredRot - ROTTOL) || isChanging) {
        isChanging = turningCorrection(false);
        if (i % 20 == 0) {
          Serial.print("currRotIn: ");
          Serial.println(currRot);
        }
        i++;
        //delay(100);
      }
      // lastUpdate = millis();
      // while (millis() - lastUpdate < TURNTIME) {
      //   isChanging = turningCorrection(false);
      //   if (!isChanging) {
      //     break;
      //   }
      // }
    }

//     float filterAngle(float newAngle) {
//         static float filteredAngle = 0;
//         const float alpha = 0.8; // Low-pass filter coefficient
//         filteredAngle = alpha * filteredAngle + (1 - alpha) * newAngle;
//         return filteredAngle;
//     }

// public:
//     void turn(double angle) {
//         mpu.update();
//         this->currRot = filterAngle(mpu.getAngleZ());
//         this->initRot = this->currRot + angle;
        
//         Serial.print("Starting turn: Current="); Serial.print(currRot);
//         Serial.print(" Target="); Serial.println(initRot);
        
//         bool changing = true;
//         unsigned long lastUpdate = 0;
//         const unsigned long UPDATE_INTERVAL = 20; // 20ms updates
        
//         while (changing && (millis() - lastUpdate < 5000)) { // 5 second timeout
//             if (millis() - lastUpdate >= UPDATE_INTERVAL) {
//                 changing = turningCorrection(false);
//                 lastUpdate = millis();
//             }
//         }
        
//         // Final position check
//         mpu.update();
//         float finalAngle = filterAngle(mpu.getAngleZ());
//         Serial.print("Turn complete: Final="); Serial.print(finalAngle);
//         Serial.print(" Error="); Serial.println(finalAngle - initRot);
//     }
    
//     bool turningCorrection(bool isDriving) {
//         if (isDriving) return false;
        
//         mpu.update();
//         this->currRot = filterAngle(mpu.getAngleZ());
        
//         float error = initRot - currRot;
        
//         // Normalize error to [-180, 180]
//         while (error > 180) error -= 360;
//         while (error < -180) error += 360;
        
//         // Proportional control with deadband
//         const float DEADBAND = 2.0; // degrees
//         const float MAX_SPEED = 80;
//         const float MIN_SPEED = 30;
        
//         if (abs(error) < DEADBAND) {
//             motor1.setPWM(0);
//             motor2.setPWM(0);
//             return false; // Stop turning
//         }
        
//         // Calculate PWM with proportional control
//         float proportionalGain = 2.0;
//         float pwm = proportionalGain * abs(error);
//         pwm = constrain(pwm, MIN_SPEED, MAX_SPEED);
        
//         if (error > 0) {
//             // Turn counterclockwise
//             motor1.setPWM(pwm * RIGHT_CORRECT);
//             motor2.setPWM(pwm * LEFT_CORRECT);
//         } else {
//             // Turn clockwise
//             motor1.setPWM(-pwm * RIGHT_CORRECT);
//             motor2.setPWM(-pwm * LEFT_CORRECT);
//         }
        
//         return true; // Still turning
//     }

private:
    MPU6050 mpu;
    Motor motor1;
    Motor motor2;
    float initRot;
    float desiredRot;
    float currRot;
    long lastUpdate = 0;
    int numOffset;
};
}

