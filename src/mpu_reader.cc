#include <Arduino.h>

#include "mpu6050.h"

static const int kMpuI2CAddr = 0x68;
static const float kGyroWeight = .98;
static const float kDt = 10;
int main() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  MPU6050 mpu(kMpuI2CAddr, kDt / 1000, 1);
  mpu.Initialize();

  for (int count = 0;; ++count) {
    float estimated_pitch = 0;
    float estimated_roll = 0;

    int16_t accel[3];
    int16_t gyro[3];
    mpu.ReadBoth(accel, gyro);

    mpu.ComputeFilteredPitchRoll(accel, gyro, &estimated_pitch, &estimated_roll);
    if (count % 10 == 0) {
      Serial.print(estimated_pitch);
      Serial.print(" ");
      Serial.print(estimated_roll);
      Serial.println("");
    }
    delay(kDt);
  }
}
