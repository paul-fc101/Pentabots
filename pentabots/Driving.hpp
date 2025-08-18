#pragma once

#include <Arduino.h>

#include "Wire.h"
#include <MPU6050_light.h>
#include <VL6180X.h>

#include "Motor.hpp"
#include "MovingAverageFilter.hpp"

#include "EncoderOdometry.hpp"
#include "DualEncoder.hpp"
#include "PIDController.hpp"

#define WHEEL_DIAM 32
#define WHEEL_BASE 91
#define RIGHT_CORRECT 1.031
#define LEFT_CORRECT 1
#define DIST_TOLLERANCE 2
#define FINAL_DIST_TOLLERANCE 2
#define CELL_DIST 180 // Distance for one cell
#define STOP_DIST 45
#define SHUTDOWN_DISTANCE 40.0 // Distance from target to start fading out wall correction
#define PID_DEADBAND 1
#define MAX_CORRECTION 150

// --- Wall Avoidance Tuning ---
#define WALL_AVOID_THRESHOLD 45           // How close (in mm) to a wall to start reacting.
#define WALL_CORRECTION_SCALE_FACTOR 0.02 // **TUNABLE**: Higher value = more aggressive turn away from wall. Start small.

// Unique I2C addresses for each sensor after reset
const uint8_t addressFront = 0x54;
const uint8_t addressRight = 0x56;
const uint8_t addressLeft  = 0x58;

// XSHUT (enable) pins for each sensor
const int XSHUT_Front = A1;
const int XSHUT_Right = A2;
const int XSHUT_Left  = A0;

mtrn3100::MovingAverageFilter filter;
mtrn3100::MovingAverageFilter rightFilter;
mtrn3100::MovingAverageFilter leftFilter;
mtrn3100::MovingAverageFilter frontFilter;

namespace mtrn3100 {

class Driving {
public:
    Driving(Motor mot1, Motor mot2, VL6180X& frontSen, VL6180X& leftSen, VL6180X& RightSens, DualEncoder& enc, EncoderOdometry& odom)
        : 
          motor1(mot1), 
          motor2(mot2), 
          FrontSensor(frontSen),
          LeftSensor(leftSen), 
          RightSensor(RightSens), 
          encoder(enc), 
          encoderOdometer(odom), 
          pidL(PIDController(1.7, 0.08, 0.4)), 
          pidR(PIDController(1.7, 0.08, 0.4))
    {
        motor1.setPWM(0);
        motor2.setPWM(0);
    }

    void initalise_lidar() {
      // Set XSHUT pins as outputs and keep sensors off initially
      pinMode(XSHUT_Front, OUTPUT);
      pinMode(XSHUT_Right, OUTPUT);
      pinMode(XSHUT_Left, OUTPUT);
      digitalWrite(XSHUT_Front, LOW);
      digitalWrite(XSHUT_Right, LOW);
      digitalWrite(XSHUT_Left, LOW);
      delay(300);       

      // Front
      digitalWrite(XSHUT_Front, HIGH);
      delay(50);
      FrontSensor.init();
      FrontSensor.configureDefault();
      FrontSensor.setTimeout(500);
      FrontSensor.setAddress(addressFront);
      delay(300);

      // Right
      digitalWrite(XSHUT_Right, HIGH);
      delay(50);
      RightSensor.init();
      RightSensor.configureDefault();
      RightSensor.setTimeout(500);
      RightSensor.setAddress(addressRight);
      delay(300);

      // Left
      digitalWrite(XSHUT_Left, HIGH);
      delay(50);
      LeftSensor.init();
      LeftSensor.configureDefault();
      LeftSensor.setTimeout(500);
      LeftSensor.setAddress(addressLeft);
      delay(300);
      Serial.println(F("Sensors ready!"));
    }

    void updateLidar() {
      // li_F = frontFilter.average(FrontSensor.readRangeSingleMillimeters());      
      // li_R = rightFilter.average(RightSensor.readRangeSingleMillimeters());
      // li_L = leftFilter.average(LeftSensor.readRangeSingleMillimeters());
      li_F = FrontSensor.readRangeSingleMillimeters();      
      li_R = RightSensor.readRangeSingleMillimeters();
      li_L = LeftSensor.readRangeSingleMillimeters();
    }

    int getFrontDist() { return li_F; }
    int getRightDist() { return li_R; }
    int getLeftDist() { return li_L; }


    void drive(int numCells) {
        // SETUP
        encoderOdometer.update(encoder.getLeftRotation(), encoder.getRightRotation());
        float leftDist = encoder.getLeftDistance(WHEEL_DIAM / 2.0);
        float rightDist = encoder.getRightDistance(WHEEL_DIAM / 2.0);
        float targetDistance = CELL_DIST * numCells;
        
        // Set distance targets for each wheel
        float targetEndL = leftDist + targetDistance;
        float targetEndR = rightDist + targetDistance;
        
        pidL.zeroAndSetTarget(leftDist, targetEndL);
        pidR.zeroAndSetTarget(rightDist, targetEndR);
        
        bool leftDone = false;
        bool rightDone = false;

        float minR = 255;
        float minL = 255;

        while (!leftDone || !rightDone) {
            // SENSOR & PID UPDATES
            encoderOdometer.update(encoder.getLeftRotation(), encoder.getRightRotation());
            updateLidar();
            leftDist = encoder.getLeftDistance(WHEEL_DIAM / 2.0);
            rightDist = encoder.getRightDistance(WHEEL_DIAM / 2.0);
            
            pidL.setpoint = targetEndL;
            pidR.setpoint = targetEndR;

            float correctionL = pidL.compute(leftDist);
            if (correctionL > MAX_CORRECTION) correctionL = MAX_CORRECTION;
            
            float correctionR =  pidR.compute(rightDist);
            if (correctionR > MAX_CORRECTION) correctionR = MAX_CORRECTION;

            // PID DEADBAND
            if (fabs(pidL.getError()) < PID_DEADBAND) { correctionL = 0; }
            if (fabs(pidR.getError()) < PID_DEADBAND) { correctionR = 0; }
            
            // Start with the base PWM values to go straight
            float pwm_R = correctionR;
            float pwm_L = -correctionL;

            // --- NEW WALL AVOIDANCE (Proportional Speed Reduction) ---
            
            if ((li_R < WALL_AVOID_THRESHOLD && li_R <= minR) || (li_L < WALL_AVOID_THRESHOLD && li_L < minL)) {
                float reduction_factor = 0.0; // The percentage to slow a wheel by (0.0 to 1.0)
                bool apply_to_left_wheel = false; 

                // Prioritize the wall we are closer to
                if (li_R < li_L) {
                    // Closer to right wall -> plan to slow the LEFT wheel to turn away.
                    float error = WALL_AVOID_THRESHOLD - li_R;
                    reduction_factor = error * WALL_CORRECTION_SCALE_FACTOR;
                    apply_to_left_wheel = true;
                } else {
                    // Closer to left wall -> plan to slow the RIGHT wheel to turn away.
                    float error = WALL_AVOID_THRESHOLD - li_L;
                    reduction_factor = error * WALL_CORRECTION_SCALE_FACTOR;
                    apply_to_left_wheel = false;
                }

                // Clamp the reduction factor to a safe range [0, 1] to prevent reversing the motor
                if (reduction_factor > 1.0) reduction_factor = 1.0;
                if (reduction_factor < 0.0) reduction_factor = 0.0;
                
                // --- SHUTDOWN ZONE ---
                // Fade out the wall avoidance effect near the destination for a straight stop
                float remaining_dist = fmin(fabs(targetEndL - leftDist), fabs(targetEndR - rightDist));
                if (remaining_dist < SHUTDOWN_DISTANCE) {
                    reduction_factor *= (remaining_dist / SHUTDOWN_DISTANCE);
                }
                
                // Apply the calculated speed reduction to the correct wheel's PWM
                if (apply_to_left_wheel) {
                    pwm_L *= (1.0 - reduction_factor);
                } else {
                    pwm_R *= (1.0 - reduction_factor);
                }
            }

            if (pwm_R > 0 && pwm_R < 15) pwm_R = 15;
            if (pwm_R < 0 && pwm_R > -15) pwm_R = -15;
            motor1.setPWM(pwm_R);

            if (pwm_L < 0 && pwm_L > -15) pwm_L = -15;
            if (pwm_L > 0 && pwm_L < 15) pwm_L = 15;
            motor2.setPWM(pwm_L);

            // --- EMERGENCY STOP ---
            if (li_F <= STOP_DIST) {
              motor1.setPWM(-30);
              motor2.setPWM(30);
              delay(300);
              motor1.stop();
              motor2.stop();
              break; 
            }

            // --- EXIT CONDITIONS ---
            leftDone = (fabs(leftDist - targetEndL) < FINAL_DIST_TOLLERANCE);
            rightDone = (fabs(rightDist - targetEndR) < FINAL_DIST_TOLLERANCE);
            
            // If one wheel finishes, stop it and let the other catch up briefly
            if (leftDone) {
              rightDone = true;
              motor2.stop();
            } else if (rightDone) {
              leftDone = true;
              motor1.stop();
            }
            if (li_R < minR) {
              minR = li_R;
            }
            if (li_L < minL) {
              minL = li_L;
            }
        }
        motor1.stop();
        motor2.stop();
    }

    // This function remains unchanged.
    bool drivingCorrection(bool isTurning) {
      if (isTurning) {
        return false;
      }
      uint16_t sensorValue = FrontSensor.readRangeSingleMillimeters() - 3;
      uint16_t value = filter.average(sensorValue);

      int pwm = 0;

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
    VL6180X& FrontSensor;
    VL6180X& LeftSensor;
    VL6180X& RightSensor;
    DualEncoder& encoder;
    EncoderOdometry& encoderOdometer;
    Motor motor1; // Right Motor
    Motor motor2; // Left Motor
    PIDController pidR, pidL;
    int li_F;  
    int li_R;
    int li_L;
};
}