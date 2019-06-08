#include "mpu6050.h"
#include <gtest/gtest.h>

const float k1G = 1 << 14;
const float kEpsilon = .01;
static const float kDt = 10;

class MpuTest : public testing::Test {
 protected:
  MpuTest() {}

  void SetUp() override {
    mpu_.reset(new MPU6050(0, kDt / 1000, 1));
  }

  void RunSameManyTimes() {
    for (int i = 0; i < 100; ++i) {
      mpu_->ComputeFilteredPitchRoll(accel_, gyro_, &pitch_, &roll_);
      //printf("%d: %f, %f\n", i, pitch_, roll_);
    }
  }

  std::unique_ptr<MPU6050> mpu_;
  int16_t accel_[3] = {0, 0, 0};
  int16_t gyro_[3] = {0, 0, 0};
  float roll_ = 0;
  float pitch_ = 0;
};

TEST_F(MpuTest, SingleCallFiltering) {
  accel_[0] = accel_[2] = k1G;
  mpu_->ComputeFilteredPitchRoll(accel_, gyro_, &pitch_, &roll_);
  // Gyro reading 0, history is 0, 0. So first call result only based
  // on alpha weighting portion of result coming from Gyros, which with
  // will be 45 degrees pitched.
  EXPECT_NEAR(0, roll_, kEpsilon);
  EXPECT_NEAR(.99 * 45, pitch_, kEpsilon);

  accel_[0] = 0;
  mpu_->ComputeFilteredPitchRoll(accel_, gyro_, &pitch_, &roll_);
  EXPECT_NEAR(0, roll_, kEpsilon);
  EXPECT_NEAR(.99 * 45 * .0099, pitch_, kEpsilon);

  // Gyro is a 16b signed number where 32768 is 500/s of rotation.
  gyro_[0] = 1966;  // 30 degrees/s
  gyro_[1] = 3932;  // 60 degrees/s
  mpu_->ComputeFilteredPitchRoll(accel_, gyro_, &pitch_, &roll_);
  // Because sampling rate is 1s, we assume we have rotated a full 30 degrees
  // on roll and 60 degrees on pitch, but this is affected by alpha.
  EXPECT_NEAR(30 * .0099, roll_, kEpsilon);
  EXPECT_NEAR(.99 * 45 * .0099 * .0099 - 60 * .0099, pitch_, kEpsilon);
}

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

TEST_F(MpuTest, Alpha) {
  EXPECT_NEAR(mpu_->alpha_, .0099, kEpsilon);

  mpu_.reset(new MPU6050(0, .5, .1));
  EXPECT_NEAR(mpu_->alpha_, .833, kEpsilon);
}

TEST_F(MpuTest, BiasedGyroAffectsResults) {
  gyro_[0] = 300;
  gyro_[1] = 300;
  accel_[2] = k1G;
  RunSameManyTimes();
  EXPECT_NE(0, roll_);
  EXPECT_NE(0, pitch_);
}

TEST_F(MpuTest, GyroCorrection) {
  int gyro_correction[3] = { -300, -300, 0 };
  gyro_[0] = 300;
  gyro_[1] = 300;
  accel_[2] = k1G;
  mpu_->SetGyroCorrection(gyro_correction);
  RunSameManyTimes();
  EXPECT_EQ(0, roll_);
  EXPECT_EQ(0, pitch_);
}

TEST_F(MpuTest, PitchCorrections) {
  accel_[0] = accel_[2] = k1G;
  mpu_->SetPitchRollCorrection(-45, 0);
  RunSameManyTimes();
  EXPECT_NEAR(0, pitch_, kEpsilon);
  EXPECT_NEAR(0, roll_, kEpsilon);
}

TEST_F(MpuTest, RollCorrections) {
  accel_[1] = accel_[2] = k1G;
  mpu_->SetPitchRollCorrection(0, -45);
  RunSameManyTimes();
  EXPECT_NEAR(0, pitch_, kEpsilon);
  EXPECT_NEAR(0, roll_, kEpsilon);
}