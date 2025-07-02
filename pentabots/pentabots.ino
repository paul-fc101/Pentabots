#include "Encoder.hpp"
#include "Motor.hpp"
#include "EncoderOdometry.hpp"
#include "PIDController.hpp"
#include "IMUOdometry.hpp"
#include "BangBangController.hpp"
#include "Wire.h"
#include "Turning.hpp"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder
#define MOT1PWM 9
#define MOT1DIR 10
#define MOT2PWM 11
#define MOT2DIR 12

mtrn3100::Encoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);
mtrn3100::Motor motor(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
mtrn3100::IMUOdometry IMU_odometry;
mtrn3100::EncoderOdometry encoder_odometry(16, 94); //WHEEL RADIUS AND AXLE LENGTH

#define WHEEL_DIAM 32
unsigned long timer = 0;


mtrn3100::BangBangController controller(120,0);


void setup() {
  Serial.begin(115200);
  Wire.begin();

  //Set up the IMU
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true);

  con.zeroAndSetTarget(encoder.getLeftRotation(), 6.28);
  Serial.println("Done!\n");
}

void loop() {
  mpu.update();
  
  if((millis()-timer)>10){ // print data every 10ms
	// Serial.print("X : ");
	// Serial.print(mpu.getAngleX());
	// Serial.print("\tY : ");
	// Serial.print(mpu.getAngleY());
	Serial.print("Yaw : ");
	Serial.println(mpu.getAngleZ());
	timer = millis();  
  }
}
