#include "mpu6050.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef TESTING
#include <Arduino.h>
#include <Wire.h>
#else
#include <stdio.h>
#endif  // TESTING

static const int PWR_MGMT_1 = 0x6B;
static const int ACCEL_XOUT_H = 0x3B;

static const int GYRO_CONFIG = 0x1B;
static const int FS_SEL_500 = 1;
static const float kGyroscopeSensitivity = 65.536;  // Full range is +/- 500 degrees/s

static const int ACCEL_CONFIG = 0x1C;
static const int AFS_SEL_4G = 1;
static const int kAccelerometerSensitivity = 8192;  // Full range is +/- 4G

void MPU6050::Initialize() {
#ifndef TESTING
  Wire.begin();
  Wire.beginTransmission(addr_);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);  // Wake up
  Wire.endTransmission(true);

  Wire.beginTransmission(addr_);
  Wire.write(GYRO_CONFIG);
  Wire.write(FS_SEL_500 << 3);
  Wire.endTransmission(true);

  Wire.beginTransmission(addr_);
  Wire.write(ACCEL_CONFIG);
  Wire.write(AFS_SEL_4G << 3);
  Wire.endTransmission(true);

#if 0
  delay(100);

  Wire.beginTransmission(addr_);
  Wire.write(0x1b);
  Wire.endTransmission(true);
  Wire.requestFrom(addr_, 2, true);
  Serial.print(F("gyro_config:  ")); Serial.println(Wire.read());
  Serial.print(F("accel_config: ")); Serial.println(Wire.read());
  delay(3000);
#endif
#endif  // TESTING
}

void MPU6050::SetGyroCorrection(const int* gyro_corrections) {
  memcpy(gyro_corrections_, gyro_corrections, sizeof(gyro_corrections_));
}

void MPU6050::ReadBoth(int16_t* accel, int16_t* gyro) {
#ifndef TESTING
  Wire.beginTransmission(addr_);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(addr_, 14, true);
  for (int i = 0; i < 3; ++i)
    accel[i] = (Wire.read() << 8) | Wire.read();
  Wire.read();  // Throw out temperature reading
  Wire.read();
  for (int i = 0; i < 3; ++i)
    gyro[i] = (Wire.read() << 8) | Wire.read();
#if 0
  for (int i = 0; i < 3; ++i) {
    Serial.print(accel[i]);
    Serial.print("\t");
  }
  for (int i = 0; i < 3; ++i) {
    Serial.print(gyro[i]);
    Serial.print("\t");
  }
  Serial.println();
#endif
#endif  // TESTING
}

// Complementary filter implementation.
void MPU6050::ComputeFilteredPitchRoll(const int16_t* accel, const int16_t* gyro,
																			 float* pitch, float* roll) {
	// Gyro axes (angular velocity measured around these axes):
	// 
	//   2
	//
	//   |_   1
	//  /
	//
	// 0 (head of cat)
  int corrected_gyro[2] = {
    gyro[0] + gyro_corrections_[0],
    gyro[1] + gyro_corrections_[1]
  };

  last_roll_ += ((float)corrected_gyro[0] / kGyroscopeSensitivity) * sampling_;
  last_pitch_ -= ((float)corrected_gyro[1] / kGyroscopeSensitivity) * sampling_;
  
  // Acceleromoter axes (acceleration measured along these axes):
  //
  //   2
  //
  //   |_   1
  //  /
  // 
  // 0 (head of cat)

  float acceleration_pitch = atan2f(float(accel[0]), float(accel[2])) * 180 / M_PI;
  float acceleration_roll = atan2f(float(accel[1]), float(accel[2])) * 180 / M_PI;
  //printf("last_pitch=%f, alpha=%f, acc_pitch=%f\n", double(last_pitch_), double(alpha_), double(acceleration_pitch));
  last_pitch_ = last_pitch_ * alpha_ + acceleration_pitch * (1 - alpha_);
  last_roll_ = last_roll_ * alpha_ + acceleration_roll * (1 - alpha_);

  *pitch = last_pitch_ + pitch_correction_;
  *roll = last_roll_ + roll_correction_;
}

