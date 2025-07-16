#pragma once

#include <Arduino.h>

#include "Wire.h"
#include <MPU6050_light.h>
#include <VL6180X.h>

#include "Motor.hpp"
#include "MovingAverageFilter.hpp"

#include "EncoderOdometry.hpp"
#include "DualEncoder.hpp"

#define WHEEL_DIAM 32
#define WHEEL_BASE 91
#define RIGHT_CORRECT 1.031
#define LEFT_CORRECT 1
#define DIST_TOLLERANCE 2

mtrn3100::MovingAverageFilter filter;

namespace mtrn3100 {

class Driving {
public:
    Driving(Motor mot1, Motor mot2, VL6180X& senRef, DualEncoder& enc, EncoderOdometry& odom)
        : motor1(mot1), motor2(mot2), FrontSensor(senRef), encoder(enc), encoderOdometer(odom) {
        motor1.setPWM(0);
        motor2.setPWM(0);
    }

    void init() {
      FrontSensor.init();
      FrontSensor.configureDefault();
      FrontSensor.setTimeout(300);
    }


    void drive(int numCells) {
      motor1.setPWM(150 * RIGHT_CORRECT);
      motor2.setPWM(-150 * LEFT_CORRECT);
      delay(880 * numCells);
      motor1.setPWM(0);
      motor2.setPWM(0);
    }

    // void drive(int numCells) {
    //   float target = numCells * 180;

    //   // Reset encoder odometry
    //   encoderOdometer.reset();

    //   Serial.println(encoderOdometer.getDistance());
    //   Serial.println(target);

    //   while (encoderOdometer.getDistance() < target - DIST_TOLLERANCE) {
    //       encoderOdometer.update(
    //           encoder.getLeftDistance(WHEEL_DIAM / 2),
    //           encoder.getRightDistance(WHEEL_DIAM / 2)
    //       );

    //       float remaining = target - encoderOdometer.getDistance();
    //       Serial.println(remaining);
    //       int pwm = 150;
          
    //       if (remaining < 50) {
    //           pwm = max(20, (int)(150 * remaining / 50));
    //       }

    //       motor1.setPWM(pwm);
    //       motor2.setPWM(-pwm);
    //   }

    //   motor1.setPWM(0);
    //   motor2.setPWM(0);
    // }

    // void drive(int numCells) {
    //     float target = numCells * 180.0;
        
    //     encoderOdometer.reset();
        
    //     Serial.print("Target: "); Serial.println(target);
        
    //     unsigned long lastUpdate = 0;
    //     const unsigned long UPDATE_INTERVAL = 5;
        
    //     while (encoderOdometer.getDistance() < target) {
    //         encoder.readLeftEncoder();
    //         encoder.readRightEncoder();
    //         if (millis() - lastUpdate >= UPDATE_INTERVAL) {
                
    //             Serial.println(encoder.getLeftDistance(WHEEL_DIAM / 2));
    //             Serial.println(encoder.getRightDistance(WHEEL_DIAM / 2));

    //             encoderOdometer.update(
    //                 encoder.getLeftDistance(WHEEL_DIAM / 2),
    //                 encoder.getRightDistance(WHEEL_DIAM / 2)
    //             );
                
    //             float remaining = target - encoderOdometer.getDistance();
    //             Serial.println(remaining);
    //             int pwm = 150;
                
    //             if (remaining < 50) {
    //                 pwm = max(20, (int)(150 * remaining / 50));
    //             }
                
    //             motor1.setPWM(pwm * RIGHT_CORRECT);
    //             motor2.setPWM(-pwm * LEFT_CORRECT);
                
    //             lastUpdate = millis();
    //         }
    //     }
        
    //     motor1.setPWM(0);
    //     motor2.setPWM(0);
        
    // }

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
    Motor motor1;
    Motor motor2;

};
}

