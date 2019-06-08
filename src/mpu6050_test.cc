#include "mpu6050.h"
#include <gtest/gtest.h>

const float k1G = 1 << 14;
const float kEpsilon = .01;
static const float kTau = 500;
static const float kDt = 10;

class MpuTest : public testing::Test {
 protected:
  MpuTest() {}

  void SetUp() override {
    mpu_.reset(new MPU6050(0, kTau/1000, kDt / 1000));
  }

  void RunSameManyTimes() {
    for (int i = 0; i < kTau/kDt * 100; ++i) {
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

TEST_F(MpuTest, Alpha) {
  // Sampling at 1s but we want to be mostly showing accelerometer after
  // 10ms (this was an early incorrect setting for the library). Alpha
  // is weighting so that almost nothing comes from the gyroscope.
  mpu_.reset(new MPU6050(0, .01, 1));
  EXPECT_NEAR(mpu_->alpha_, .0099, kEpsilon);

  // Sampling at 10ms with filter mostly coming from accelerometer after
  // 500ms.
  mpu_.reset(new MPU6050(0, .5, .01));
  EXPECT_NEAR(mpu_->alpha_, .98, kEpsilon);
}

TEST_F(MpuTest, SingleCallFiltering) {
  const float kIntEpsilon = .5;
  accel_[0] = accel_[2] = k1G;
  mpu_->ComputeFilteredPitchRoll(accel_, gyro_, &pitch_, &roll_);
  mpu_->ComputeFilteredPitchRoll(accel_, gyro_, &pitch_, &roll_);
  // Gyro reading 0, history is 0, 0. So first call result only based
  // on alpha weighting portion of result coming from Gyros, which with
  // will be 45 degrees pitched.
  EXPECT_NEAR(0, roll_, kIntEpsilon);
  float expect_pitch = .0196 * 45 * .9803 + .0196 * 45;
  EXPECT_NEAR(2, expect_pitch, kIntEpsilon);
  EXPECT_NEAR(expect_pitch, pitch_, kIntEpsilon);

  accel_[0] = 0;
  mpu_->ComputeFilteredPitchRoll(accel_, gyro_, &pitch_, &roll_);

  expect_pitch = expect_pitch * .9803;
  EXPECT_NEAR(2, expect_pitch, kIntEpsilon);
  EXPECT_EQ(0, roll_);
  EXPECT_NEAR(expect_pitch, pitch_, kIntEpsilon);

  // Gyro is a 16b signed number where 32768 is 500/s of rotation.
  gyro_[0] = 19660;  // 300 degrees/s
  gyro_[1] = 26214;  // 400 degrees/s
  mpu_->ComputeFilteredPitchRoll(accel_, gyro_, &pitch_, &roll_);
  // Because sampling rate is 1s, we assume we have rotated a full 30 degrees
  // on roll and 60 degrees on pitch, but this is affected by alpha.
  float expect_roll = 300 * .01 * .9802;
  EXPECT_NEAR(3, expect_roll, kIntEpsilon);
  EXPECT_NEAR(expect_roll, roll_, kIntEpsilon);
  expect_pitch -= 400 * .01 * .9802;
  EXPECT_NEAR(-2, expect_pitch, kIntEpsilon);
  EXPECT_NEAR(expect_pitch, pitch_, kIntEpsilon);
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