#ifndef _MPU6050_H
#define _MPU6050_H

#include <stdint.h>

class MPU6050 {
 public:
  // addr is i2c addre sof MPU6050. tau is how quickly to make pitch/roll respond
  // to quick movements (more quickly means more potential for drift), and
  // sampling is the sampling rate at which you call ComputeFilteredPitchRoll.
 	MPU6050(int addr, float tau, float sampling)
      : addr_(addr), sampling_(sampling), alpha_(tau / (tau + sampling)) {}
  void Initialize();
 	void ReadBoth(int16_t* accel, int16_t* gyro);
	void ComputeFilteredPitchRoll(const int16_t* accel, const int16_t* gyro,
				        								float* pitch, float* roll);

  int addr_;
  float sampling_;
  float alpha_;
};

#endif  // _MPU6050_H