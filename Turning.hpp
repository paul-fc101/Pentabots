#pragma once

#include <Arduino.h>

namespace mtrn3100 {

// The Turning class is designed to control the turning of a robot using encoders and motors.
class Turning {
public:
    Turning(Encoder& enc, Motor& mot1, Motor& mot2)
        : encoder(enc), motor1(mot1), motor2(mot2) {
        // Initialize the motors to a stopped state
        motor1.setPWM(0);
        motor2.setPWM(0);
    }


    void clockwiseTurn() {
      
    }

    void anticlockwiseTurn() {
      
    }

  }
}
