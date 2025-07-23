#pragma once

#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd)
        : kp(kp), ki(ki), kd(kd), prev_error(0), integral(0), output(0),
        setpoint(0), zero_ref(0), prev_time(0) {}

    // Compute the output signal required from the current/actual value.
    float compute(float input) {
        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        if (dt <= 0.001) {
            dt = 0.001;
        }
        float current_value = input;
        //float current_value = input - zero_ref;
        error = setpoint - current_value;

        integral += error * dt;
        derivative = (error - prev_error) / dt;
        output = kp * error + ki * integral + kd * derivative;

        prev_error = error;
        return output;
    }

    // Function used to return the last calculated error. 
    // The error is the difference between the desired position and current position. 
    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    float getError() {
      return error;
    }

    // This must be called before trying to achieve a setpoint.
    // The first argument becomes the new zero reference point.
    // Target is the setpoint value.
    void zeroAndSetTarget(float zero, float target) {
        zero_ref = zero;
        setpoint = target;
        integral = 0;
        prev_error = 0;
        prev_time = micros();
    }

public:
    uint32_t prev_time, curr_time = micros();
    float dt = 0.0;

private:
    float kp, ki, kd;
    float error = 0, derivative = 0, integral = 0, output = 0;
    float prev_error = 0;
    float setpoint = 0;
    float zero_ref = 0;
};

}  // namespace mtrn3100
