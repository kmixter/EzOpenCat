#include "servo_animator.h"

#include <gtest/gtest.h>

class ServoAnimatorTest : public testing::Test {
 protected:
  ServoAnimatorTest() {}

  void SetUp() override {
    animator_.Initialize();
    animator_.SetServoParams(zeroes_);
  }

  ServoAnimator animator_;
  int8_t zeroes_[kServoCount] = {0};
};

TEST_F(ServoAnimatorTest, GetFrame) {
  const int8_t* frame =
    animator_.GetFrame(kAnimationCalibrationPose, 0);
  for (int i = 0; i < kServoCount; ++i)
    EXPECT_EQ(0, frame[i]);
}

TEST_F(ServoAnimatorTest, SetFrameToCalibrationWithNoZeroOffsets) {
  for (int i = 0; i < kServoCount; ++i)
    EXPECT_FALSE(animator_.servo_[i]->attached);
  const int8_t* frame =
    animator_.GetFrame(kAnimationCalibrationPose, 0);
  animator_.Attach();
  animator_.SetFrame(frame, 1);
  for (int i = 0; i < kServoCount; ++i) {
    EXPECT_TRUE(animator_.servo_[i]->attached);
    EXPECT_EQ(90, animator_.servo_[i]->value);  
  }

  animator_.Detach();
  for (int i = 0; i < kServoCount; ++i)
    EXPECT_FALSE(animator_.servo_[i]->attached);
}

TEST_F(ServoAnimatorTest, SetFrameToCalibrationWithZeroOffsets) {
  zeroes_[kServoHead] = -5;
  zeroes_[kServoLeftFrontShoulder] = 7;
  zeroes_[kServoRightFrontShoulder] = 7;
   const int8_t* frame =
    animator_.GetFrame(kAnimationCalibrationPose, 0); 
  animator_.Attach();
  animator_.SetFrame(frame, 1);
  for (int i = 0; i < kServoCount; ++i) {
    EXPECT_TRUE(animator_.servo_[i]->attached);
    int expected = 90;
    switch (i) {
     case kServoHead:
      expected = 85;
      break;

     case kServoLeftFrontShoulder:
      expected = 97;
      break;

     case kServoRightFrontShoulder:
      expected = 83;
      break;
    }
    EXPECT_EQ(expected, animator_.servo_[i]->value) << "servo " << i;
  }
}
