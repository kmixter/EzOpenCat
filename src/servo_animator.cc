#include "servo_animator.h"

static const int kPinMap[] = {
  3,  // kServoHead,
  13, // kServoNeck,
  9,  // kServoLeftFrontKnee,
  4,  // kServoRightFrontKnee,
  8,  // kServoLeftFrontShoulder,
  6,  // kServoRightFrontShoulder,
  11, // kServoLeftBackShoulder,
  7,  // kServoRightBackShoulder,
  10, // kServoLeftBackKnee,
  5,  // kServoRightBackKnee,
  12, // kServoTail,
  // kServoCount
};

const int* ServoAnimator::GetFrame(AnimationSequence sequence, int number) {
  static const int kSequence[][11] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  };

  if (sequence >= kAnimationCount)
    return nullptr;

  if (number > 0)
    return nullptr;

  return kSequence[sequence];
}

void ServoAnimator::Initialize() {
  current_frame_ = GetFrame(kAnimationRest, 0);
  next_frame_ = GetFrame(kAnimationRest, 0);
  // Set up pins.
  for (int i = 0; i < kServoCount; ++i) {
    servo_[i] = new Servo();
  }
}

void ServoAnimator::Attach() {
  for (int i = 0; i < kServoCount; ++i)
    servo_[i]->attach(kPinMap[i], -90, 90);
}

void ServoAnimator::Detach() {
  for (int i = 0; i < kServoCount; ++i)
    servo_[i]->detach();
}

void ServoAnimator::SetServoParams(const EepromSettings::ServoParam* params) {
  servo_params_ = params;
}

void ServoAnimator::SetFrame(int* servo_values) {
  for (int i = 0; i < kServoCount; ++i)
    servo_[i]->write(servo_values[i]);
}
