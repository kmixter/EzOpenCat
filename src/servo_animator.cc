#include "servo_animator.h"

#ifndef TESTING
#include <Arduino.h>
#define pgm_read_int8(_a) (int8_t)pgm_read_byte(_a)
#else
#define PROGMEM
#define pgm_read_int8(_a) (*(_a))
#endif  // TESTING

#include "Instinct.h"

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
  static int result[11];
  static const char* map[] = {
    rest,
    calib,
    sleep
  };

  if (sequence >= kAnimationCount)
    return nullptr;

  const char* instinct = map[sequence];
  const char* walking_frame = nullptr;
  if (pgm_read_int8(instinct) == 1) {
    const char* full_frame = instinct + 3;
    // Single frame has 16 entries.
    result[kServoHead] = pgm_read_int8(full_frame + 0);
    result[kServoNeck] = pgm_read_int8(full_frame + 1);
    result[kServoTail] = pgm_read_int8(full_frame + 2);
    walking_frame = full_frame + 8;
  } else {
    if (number >= pgm_read_int8(instinct))
      return nullptr;
    walking_frame = instinct + 3 + 8 * number;
    result[kServoHead] = 0;
    result[kServoNeck] = 0;
    result[kServoTail] = 0;
  }
  result[kServoLeftFrontKnee] = pgm_read_int8(walking_frame + 4);
  result[kServoLeftFrontShoulder] = pgm_read_int8(walking_frame + 0);
  result[kServoRightFrontKnee] = pgm_read_int8(walking_frame + 5);
  result[kServoRightFrontShoulder] = pgm_read_int8(walking_frame + 1);
  result[kServoLeftBackShoulder] = pgm_read_int8(walking_frame + 3);
  result[kServoLeftBackKnee] = pgm_read_int8(walking_frame + 7);
  result[kServoRightBackShoulder] = pgm_read_int8(walking_frame + 2);
  result[kServoRightBackKnee] = pgm_read_int8(walking_frame + 6);    

  return result;
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

void ServoAnimator::SetFrame(const int* servo_values, unsigned long millis_now) {
  if (servo_values == 0) {
#ifndef TESTING
    Serial.println("Invalid frame");
#endif
    return;
  }
  for (int i = 0; i < kServoCount; ++i) {
    int angle = 90 + (servo_values[i] + servo_zero_offsets_[i]) * kDirectionMap[i];
#if 0
    Serial.print("Servo ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(servo_values[i]);
    Serial.print(", zeroed ");
    Serial.print(servo_zero_offsets_[i]);
    Serial.print(" so setting to ");
    Serial.println(angle);
#endif
    servo_[i]->write(angle);
  }
  //while (!Serial.available()) yield();
  //Serial.read();
}
