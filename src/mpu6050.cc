#include "mpu6050.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef TESTING
#include <Wire.h>
#endif  // TESTING

static const int PWR_MGMT_1 = 0x6B;
static const int ACCEL_XOUT_H = 0x3B;
static const float kGyroscopeSensitivity = 65.536;
static const int kAccelerometerSensitivity = 8192;  // .5G is this, 2G is max (4x this)

void MPU6050::Initialize() {
#ifndef TESTING
  Wire.begin();
  Wire.beginTransmission(addr_);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);  // Wake up
  Wire.endTransmission(true);
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
    Serial.print(" ");
  }
  for (int i = 0; i < 3; ++i) {
    Serial.print(gyro[i]);
    Serial.print(" ");
  }
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
  last_pitch_ -= ((float)corrected_gyro[1] / kGyroscopeSensitivity) * sampling_;
  last_roll_ += ((float)corrected_gyro[0] / kGyroscopeSensitivity) * sampling_;

  float acceleration_magnitude = abs(accel[0]) + abs(accel[1]) + abs(accel[2]);
  if (acceleration_magnitude < kAccelerometerSensitivity ||
      acceleration_magnitude > float(4) * kAccelerometerSensitivity) {
    return;
  }
  
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
  last_pitch_ = last_pitch_ * alpha_ + acceleration_pitch * (1 - alpha_);
  last_roll_ = last_roll_ * alpha_ + acceleration_roll * (1 - alpha_);

  *pitch = last_pitch_ + pitch_correction_;
  *roll = last_roll_ + roll_correction_;
}

