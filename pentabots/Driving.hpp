#pragma once

#include <Arduino.h>

#include "Wire.h"
#include <MPU6050_light.h>
#include <VL6180X.h>

#include "Motor.hpp"
#include "MovingAverageFilter.hpp"

mtrn3100::MovingAverageFilter filter;

namespace mtrn3100 {

class Driving {
public:
    Driving(Motor mot1, Motor mot2, VL6180X& senRef)
        : motor1(mot1), motor2(mot2), FrontSensor(senRef) {
        motor1.setPWM(0);
        motor2.setPWM(0);
    }

    void init() {
      FrontSensor.init();
      FrontSensor.configureDefault();
      FrontSensor.setTimeout(500);
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

      motor1.setPWM(pwm);
      motor2.setPWM(-pwm);
      if (pwm == 0) {
        return false;
      }
      return true;
    }


private:
    VL6180X FrontSensor;
    Motor motor1;
    Motor motor2;

};
}

