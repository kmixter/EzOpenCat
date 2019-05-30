#ifndef _SERVO_ANIMATOR_H
#define _SERVO_ANIMATOR_H

#ifndef TESTING
#include <Servo.h>
#else

class Servo {
 public:
  Servo() {}

  void attach(int pin) {
    this->pin = pin;
    attached = true;
  }

  void detach() {
    attached = false;
  }

  void write(int value) {
    this->value = value;
  }

  int pin = 0;
  int value = 0;
  bool attached = false;
};

#endif  // TESTING


#include "eeprom_settings.h"

const int kAnimationRest = 37;
const int kAnimationCalibrationPose = 18;
const int kAnimationSleep = 39;
const int kAnimationBalance = 16;
const int kAnimationSit = 38;
const int kAnimationWalk = 13;

enum ServoIndex {
  kServoHead,
  kServoNeck,
  kServoTail,
  kServoLeftFrontShoulder,
  kServoRightFrontShoulder,
  kServoRightBackShoulder,
  kServoLeftBackShoulder,
  kServoLeftFrontKnee,
  kServoRightFrontKnee,
  kServoRightBackKnee,
  kServoLeftBackKnee,
  kServoCount
};

const int kDefaultMsPerDegree = 1;

class ServoAnimator {
 public:
  ServoAnimator() {}

  void Initialize();
  void ResetAnimation();
  void WriteServo(int servo, int logical_angle);
  void Attach();
  void Detach();
  void SetServoParams(const int8_t* servo_zero_offsets);
  void SetFrame(const int8_t* servo_values, unsigned long millis_now);
  const int8_t* GetFrame(int animation, int number);
  void Animate(unsigned long millis_now);
  bool animating() const { return animating_; }
  void set_ms_per_degree(int ms) { ms_per_degree_ = ms; }
  int ConvertToRealAngle(int servo, int angle) {
    return 90 + (angle + servo_zero_offsets_[servo]) * kDirectionMap[servo];
  }

 private:
  unsigned long millis_last_ = 0;
  unsigned long millis_start_;
  const int8_t* servo_zero_offsets_ = nullptr;
  int ms_per_degree_ = kDefaultMsPerDegree;
  bool animating_ = false;
  int8_t start_frame_[kServoCount];
  int8_t target_frame_[kServoCount];
  int8_t current_positions_[kServoCount];

#ifdef TESTING
 public:
#endif  // TESTING
  static const int kDirectionMap[kServoCount];
  Servo* servo_[kServoCount];
};

#endif  // _SERVO_ANIMATOR_H