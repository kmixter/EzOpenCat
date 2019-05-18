#include "mpu6050.h"
#include <gtest/gtest.h>

const float k1G = 1 << 14;
const float kEpsilon = .01;

class MpuTest : public testing::Test {
 protected:
  MpuTest() : mpu_(0, .5, 1) {}

  void SetUp() override {
  }

  void RunSameManyTimes() {
    for (int i = 0; i < 100; ++i) {
      mpu_.ComputeFilteredPitchRoll(accel_, gyro_, &pitch_, &roll_);
      //printf("%d: %f, %f\n", i, pitch_, roll_);
    }
  }

  MPU6050 mpu_;
  int16_t accel_[3] = {0, 0, 0};
	int16_t gyro_[3] = {0, 0, 0};
	float roll_ = 0;
	float pitch_ = 0;
};

TEST_F(MpuTest, ComputeFilteredPitchRollDown) {
	// Provide most usual inputs - gravity acceleration only along Z, no rotation.
  accel_[2] = k1G;
  RunSameManyTimes();
	EXPECT_EQ(0, roll_);
	EXPECT_EQ(0, pitch_);
}

TEST_F(MpuTest, ComputeFilteredPitchRollPitched) {
	accel_[0] = accel_[2] = k1G;
  RunSameManyTimes();
	EXPECT_NEAR(0, roll_, kEpsilon);
	EXPECT_NEAR(45, pitch_, kEpsilon);
}

TEST_F(MpuTest, ComputeFilteredPitchRollRolled) {
	accel_[1] = accel_[2] = k1G;
  RunSameManyTimes();
	EXPECT_NEAR(45, roll_, kEpsilon);
	EXPECT_NEAR(0, pitch_, kEpsilon);
}