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
#define DIST_TOLLERANCE 1
#define FINAL_DIST_TOLLERANCE 2
#define CELL_DIST 180 // * 1.095 // Distance times scaling factor for calibration
#define STOP_DIST 45
#define TARGET_WALL_DISTANCE 50 
#define SHUTDOWN_DISTANCE 30.0
#define PID_DEADBAND 1

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
          pidL(PIDController(1.7, 0.1, 0.4)), 
          pidR(PIDController(1.7, 0.1, 0.4)), 
          // pidL(PIDController(0.5, 0.1, 0.4)), 
          // pidR(PIDController(0.5, 0.1, 0.4)), 
          pidWall(PIDController(1.5, 0.0, 0.0)
    ) {
        motor1.setPWM(0);
        motor2.setPWM(0);
        pidWall.zeroAndSetTarget(0, 0);
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
      Serial.println(F("Initializing Right Sensor..."));
      digitalWrite(XSHUT_Right, HIGH);
      delay(50);
      RightSensor.init();
      RightSensor.configureDefault();
      RightSensor.setTimeout(500);
      RightSensor.setAddress(addressRight);
      delay(300);

      // Left
      Serial.println(F("Initializing Left Sensor..."));
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
      li_F = frontFilter.average(FrontSensor.readRangeSingleMillimeters());      
      li_R = rightFilter.average(RightSensor.readRangeSingleMillimeters());
      li_L = leftFilter.average(LeftSensor.readRangeSingleMillimeters());
    }

    int getFrontDist() {
      return li_F;
    }

    int getRightDist() {
      return li_R;
    }

    int getLeftDist() {
      return li_L;
    }

    // In Driving.hpp, add this new tuning constant at the top
#define AVOIDANCE_PENALTY_SCALER 0.05 // Tune this! Start small.

void drive(int numCells) {
    // SETUP
    encoderOdometer.update(encoder.getLeftRotation(), encoder.getRightRotation());
    float leftDist = encoder.getLeftDistance(WHEEL_DIAM / 2.0);
    float rightDist = encoder.getRightDistance(WHEEL_DIAM / 2.0);
    float targetDistance = CELL_DIST * numCells;

    // Set Dynamic targets
    float targetEndL = leftDist + targetDistance;
    float targetEndR = rightDist + targetDistance;
    
    float maxTargetL = targetEndL * 1.15;
    float maxTargetR = targetEndL * 1.15;
    
    pidL.zeroAndSetTarget(leftDist, targetEndL);
    pidR.zeroAndSetTarget(rightDist, targetEndR);
    
    bool leftDone = false;
    bool rightDone = false;

    // Flag to track wall avoidance
    bool isAvoidingWall = false;
    int minLidarR = 255;
    int minLidarL = 255;

    while (!leftDone || !rightDone) {
        // SENSOR & PID UPDATES
        encoderOdometer.update(encoder.getLeftRotation(), encoder.getRightRotation());
        updateLidar();
        leftDist = encoder.getLeftDistance(WHEEL_DIAM / 2.0);
        rightDist = encoder.getRightDistance(WHEEL_DIAM / 2.0);
        
        // Update the PID controllers with the NEW dynamic targets
        pidL.setpoint = targetEndL;
        pidR.setpoint = targetEndR;
        
        float correctionL = pidL.compute(leftDist);
        float correctionR = pidR.compute(rightDist);

        // PID DEADBAND
        if (fabs(pidL.getError()) < PID_DEADBAND) { correctionL = 0; }
        if (fabs(pidR.getError()) < PID_DEADBAND) { correctionR = 0; }
        
        // WALL AVOIDANCE LOGIC
        float wall_error = 0;
        const float WALL_AVOID_THRESHOLD = 45; 
        if (li_R < WALL_AVOID_THRESHOLD && li_R <= minLidarR) {
            wall_error = WALL_AVOID_THRESHOLD - li_R;
            if (li_R < minLidarR) {
              minLidarR = li_R;
            }
        } else if (li_L < WALL_AVOID_THRESHOLD && li_L <= minLidarL) {
            wall_error = -(WALL_AVOID_THRESHOLD - li_L);
            if (li_L < minLidarL) {
              minLidarL = li_L;
            }
        }
        float wall_correction = pidWall.compute(wall_error);
        if (fabs(wall_error) > 0) { // Check if we are actively avoiding
            if (!isAvoidingWall) {
                // We have just STARTED an avoidance maneuver
                isAvoidingWall = true;
            }

            // Calculate the path length penalty.
            // It's proportional to how hard we are turning.
            // This compensates for the longer, curved path the robot is taking.
            float distance_penalty = fabs(wall_correction) * AVOIDANCE_PENALTY_SCALER;

            // Add the penalty to the final destination

            if (wall_error < 0) {
              targetEndL += distance_penalty;
            } else if (wall_error > 0) {
              targetEndR += distance_penalty;
            } else {
              isAvoidingWall = false;
            }

            // if (wall_error < 0 && targetEndR < maxTargetR) {
            //   targetEndL += distance_penalty;
            // } else if (wall_error > 0 && targetEndL < maxTargetL) {
            //   targetEndR += distance_penalty;
            // }

            // if (targetEndR < maxTargetR) {
            //   targetEndR += distance_penalty;
            // } else {
            //   isAvoidingWall = false;
            // }
            // if (targetEndL < maxTargetL) {
            //   targetEndL += distance_penalty;
            // } else {
            //   isAvoidingWall = false;
            // }
        } else {
            if (isAvoidingWall) {
                // We have just FINISHED an avoidance maneuver.
                isAvoidingWall = false;
            }
        }
        
        // --- SHUTDOWN ZONE (Modified) ---
        float remaining_dist = fmin(fabs(targetEndL - leftDist), fabs(targetEndR - rightDist));
        if (remaining_dist < SHUTDOWN_DISTANCE) {
            wall_correction *= (remaining_dist / SHUTDOWN_DISTANCE);
        }

        // COMBINE AND SET MOTOR PWM
        float pwm_R = correctionR;
        float pwm_L = -correctionL;
        if (isAvoidingWall) {
          pwm_R = correctionR - wall_correction;
          pwm_L = -correctionL - wall_correction;
        }
      
        // Apply low PWM threshold
        // if (pwm_R > 1 && pwm_R < 15) pwm_R = 15;
        // if (pwm_R < -1 && pwm_R > -15) pwm_R = -15;
        // if (pwm_R >= -1 && pwm_R <= 0) pwm_R = 0;
        // if (pwm_R <= 1 && pwm_R >= 0) pwm_R = 0;
        if (pwm_R > 0 && pwm_R < 16) pwm_R = 16;
        if (pwm_R < 0 && pwm_R > -15) pwm_R = -15;
        motor1.setTargetPWM(pwm_R);

        // if (pwm_L < -1 && pwm_L > -15) pwm_L = -15;
        // if (pwm_L > 1 && pwm_L < 15) pwm_L = 15;
        // if (pwm_L >= -1 && pwm_L <= 0) pwm_L = 0;
        // if (pwm_L <= 1 && pwm_L >= 0) pwm_L = 0;
        if (pwm_L < 0 && pwm_L > -16) pwm_L = -16;
        if (pwm_L > 0 && pwm_L < 15) pwm_L = 15;
        motor2.setTargetPWM(pwm_L);

        if (li_F <= STOP_DIST) {
          // Moves backwards slowly
          motor1.setPWM(-30);
          motor2.setPWM(30);
          delay(300);
          motor1.stop();
          motor2.stop();
          break; // move to next command as we are too close to the wall
        }
        motor1.update();
        motor2.update();

        leftDone = (fabs(leftDist - targetEndL) < DIST_TOLLERANCE);
        rightDone = (fabs(rightDist - targetEndR) < DIST_TOLLERANCE);

        float finalTimeL = -1;
        float finalTimeR = -1;
        float currentTime = millis();
        if (fabs(leftDist - targetEndL) < FINAL_DIST_TOLLERANCE) {
          if (finalTimeL == -1) {
            finalTimeL = currentTime;
          } else if (currentTime - finalTimeL > 500) {
            leftDone = true;
          }
        }
        if (fabs(rightDist - targetEndR) < FINAL_DIST_TOLLERANCE) {
          if (finalTimeR == -1) {
            finalTimeR = currentTime;
          } else if (currentTime - finalTimeR > 500) {
            rightDone = true;
          }
        }

        if (leftDone) {
          motor2.stop();
        }
        if (rightDone) {
          motor1.stop();
        }
    }
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
    PIDController pidR, pidL, pidWall;
    int li_F;  
    int li_R;
    int li_L;
};
}

