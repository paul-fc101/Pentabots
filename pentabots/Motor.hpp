#pragma once

#include <Arduino.h>

#include "math.h"

namespace mtrn3100 {

class Motor {
public:
    Motor(uint8_t pwm_pin, uint8_t in2, int motor) :  pwm_pin(pwm_pin), dir_pin(in2), motor(motor) {
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT); 
    }

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

        // Clamp the target to a max of 255
        if (pwm > 255) pwm = 255;
        if (pwm < -255) pwm = -255;
        if ((pwm > 0 && pwm < target_pwm) || (pwm < 0 && pwm > target_pwm)) {
            target_pwm = pwm;
            setPWM(pwm);
        } else {
            target_pwm = pwm;
            update();
        }
    }

    void update() {
        if (current_pwm < target_pwm) {
            current_pwm += MAX_PWM_CHANGE_PER_UPDATE;
        } else if (current_pwm > target_pwm) {
            current_pwm -= MAX_PWM_CHANGE_PER_UPDATE;
        }
        if (current_pwm > 0 && current_pwm < 15) {
            current_pwm = 15;
        } else if (current_pwm < 0 && current_pwm > -15) {
            current_pwm = 15;
        }
        setPWM(current_pwm);
    }

    void instantUpdate() {
        analogWrite(pwm_pin, abs(current_pwm));
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
    const uint8_t motor;
};

}  // namespace mtrn3100
