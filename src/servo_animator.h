#ifndef _SERVO_ANIMATOR_H
#define _SERVO_ANIMATOR_H

#ifndef TESTING
#include <Servo.h>
#else

class Servo {
 public:
  Servo() {}

  void attach(int pin, int min, int max) {
    this->pin = pin;
    this->min = min;
    this->max = max;
    attached = true;
  }

  void detach() {
    attached = false;
  }

  void write(int value) {
    this->value = value;
  }

  int pin = 0;
  int min = 0;
  int max = 0;
  int value = 0;
  bool attached = false;
};

#endif  // TESTING


#include "eeprom_settings.h"

const int kAnimationRest = 37;
const int kAnimationCalibrationPose = 18;

enum ServoIndex {
  kServoHead,
  kServoNeck,
  kServoLeftFrontKnee,
  kServoLeftFrontShoulder,
  kServoRightFrontKnee,
  kServoRightFrontShoulder,
  kServoLeftBackShoulder,
  kServoLeftBackKnee,
  kServoRightBackShoulder,
  kServoRightBackKnee,
  kServoTail,
  kServoCount
};

class ServoAnimator {
 public:
  ServoAnimator() {}

  void Initialize();
  void Attach();
  void Detach();
  void SetServoParams(const int8_t* servo_zero_offsets);
  void SetFrame(const int8_t* servo_values, unsigned long millis_now);
  const char* GetAnimationNameByNumber(int animation);
  const int8_t* GetFrame(int animation, int number);
  void Animate(unsigned long millis_now);

 private:
  const int8_t* servo_zero_offsets_ = nullptr;

#ifdef TESTING
 public:
#endif  // TESTING
  Servo* servo_[kServoCount];
};

#endif  // _SERVO_ANIMATOR_H