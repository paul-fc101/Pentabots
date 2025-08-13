#pragma once

#include <Arduino.h>

#include "math.h"

namespace mtrn3100 {

// The motor class is a simple interface designed to assist in motor control
// You may choose to impliment additional functionality in the future such as dual motor or speed control 
class Motor {
public:
    Motor(uint8_t pwm_pin, uint8_t in2) :  pwm_pin(pwm_pin), dir_pin(in2) {
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT); 
    }

    // This function outputs the desired motor direction and the PWM signal. 
    // NOTE: a pwm signal > 255 could cause troubles as such ensure that pwm is clamped between 0 - 255.

    void setPWM(int16_t pwm) {
        if (pwm >  0) {
            digitalWrite(dir_pin, HIGH);
        } else {
            digitalWrite(dir_pin, LOW);
            pwm = -pwm;
        }
        if (pwm > 255) {
            pwm = 255;
        }

        analogWrite(pwm_pin, abs(pwm));
    }

    void setTargetPWM(int16_t pwm) {
        target_pwm = pwm;
        // Clamp the target to a max of 255
        if (target_pwm > 255) target_pwm = 255;
        if (target_pwm < -255) target_pwm = -255;
        update();
    }

    void update() {
        if (current_pwm < target_pwm) {
            current_pwm += MAX_PWM_CHANGE_PER_UPDATE;
            if (current_pwm > target_pwm) {
                current_pwm = target_pwm;
            }
        } else if (current_pwm > target_pwm) {
            current_pwm -= MAX_PWM_CHANGE_PER_UPDATE;
            if (current_pwm < target_pwm) {
                current_pwm = target_pwm;
            }
        }
        setPWM(current_pwm);
    }

    void stop() {
      target_pwm = 0;
      current_pwm = 0;
      setPWM(0);
    }
    
private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;

    static const uint8_t MAX_PWM_CHANGE_PER_UPDATE = 30;
    int16_t target_pwm = 0;
    int16_t current_pwm = 0;
};

}  // namespace mtrn3100
