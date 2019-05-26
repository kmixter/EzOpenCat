#include "servo_animator.h"

#include <Arduino.h>

static const int kPinMap[] = {
  3,  // kServoHead,
  13, // kServoNeck,
  9,  // kServoLeftFrontKnee,
  8,  // kServoLeftFrontShoulder,
  4,  // kServoRightFrontKnee,
  6,  // kServoRightFrontShoulder,
  11, // kServoLeftBackShoulder,
  10, // kServoLeftBackKnee,
  7,  // kServoRightBackShoulder,
  5,  // kServoRightBackKnee,
  12, // kServoTail,
  // kServoCount
};

static const int kDirectionMap[] = {
  1,
  1,
  1,
  1,
  -1,
  -1,
  1,
  1,
  -1,
  -1,
  1
};

const int* ServoAnimator::GetFrame(AnimationSequence sequence, int number) {
  static const int kSequence[][11] = {
    {-30, -80, -45, 60, -45, 60, -60, 45, -60, 45, -45},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  };

#if 0
  const char rest[] PROGMEM = { 
  1, 0, 0,
  -30,-80,-45,  0, -3, -3,  3,  3, 60, 60,-60,-60,-45,-45, 45, 45,};
#endif

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

void ServoAnimator::SetServoParams(const int8_t* servo_zero_offsets) {
  servo_zero_offsets_ = servo_zero_offsets;
}

void ServoAnimator::SetFrame(const int* servo_values) {
  for (int i = 0; i < kServoCount; ++i) {
    Serial.print("Servo ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(servo_values[i]);
    Serial.print(", ");
    Serial.print(servo_zero_offsets_[i]);
    int angle = 90 + (servo_values[i] + servo_zero_offsets_[i]) * kDirectionMap[i];
    Serial.print("setting to ");
    Serial.println(angle);
    servo_[i]->write(angle);
  }
  //while (!Serial.available()) yield();
  //Serial.read();
}
