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
  12, // kServoTail,
  8,  // kServoLeftFrontShoulder,
  6,  // kServoRightFrontShoulder,
  7,  // kServoRightBackShoulder,
  11, // kServoLeftBackShoulder,
  9,  // kServoLeftFrontKnee,
  4,  // kServoRightFrontKnee,
  5,  // kServoRightBackKnee,
  10, // kServoLeftBackKnee,
  // kServoCount
};

static const int kDirectionMap[] = {
  1,
  -1,
  -1,
  1,
  -1,
  -1,
  1,
  -1,
  1,
  1,
  -1
};

const int8_t* ServoAnimator::GetFrame(int animation, int number) {
  static int8_t result[11];

  if (animation < 0 || animation >= NUM_SKILLS)
    return nullptr;

  const char* instinct = progmemPointer[animation];
  const char* walking_frame = nullptr;
  if (number >= pgm_read_int8(instinct))
    return nullptr;
  if (pgm_read_int8(instinct) == 1) {
    const char* full_frame = instinct + 3;
    // Single frame has 16 entries.
    result[kServoHead] = pgm_read_int8(full_frame + 0);
    result[kServoNeck] = pgm_read_int8(full_frame + 1);
    result[kServoTail] = pgm_read_int8(full_frame + 2);
    walking_frame = full_frame + 8;
  } else {
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

void ServoAnimator::SetFrame(const int8_t* servo_values, unsigned long millis_now) {
  if (servo_values == 0) {
#ifndef TESTING
    Serial.println("Invalid frame");
#endif
    return;
  }
  for (int i = 0; i < kServoCount; ++i) {
    int angle = 90 + (servo_values[i] + servo_zero_offsets_[i]) * kDirectionMap[i];
#ifndef TESTING
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
#endif
    servo_[i]->write(angle);
  }
  //while (!Serial.available()) yield();
  //Serial.read();
}
