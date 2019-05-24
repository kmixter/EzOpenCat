#ifndef _SERVO_ANIMATOR_H
#define _SERVO_ANIMATOR_H

#include <Servo.h>

#include "eeprom_settings.h"

enum AnimationSequence {
  kAnimationRest,
  kAnimationCalibrationPose,
  kAnimationCount
};

enum ServoIndex {
  kServoHead,
  kServoNeck,
  kServoLeftFrontKnee,
  kServoRightFrontKnee,
  kServoLeftFrontShoulder,
  kServoRightFrontShoulder,
  kServoLeftBackShoulder,
  kServoRightBackShoulder,
  kServoLeftBackKnee,
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
  void SetServoParams(const EepromSettings::ServoParam* params);
  void SetFrame(int* servo_values);

 private:
  const int* GetFrame(AnimationSequence sequence, int number);
  const EepromSettings::ServoParam* servo_params_ = nullptr;
  const int* current_frame_ = nullptr;
  const int* next_frame_ = nullptr;
  Servo* servo_[kServoCount];
};

#endif  // _SERVO_ANIMATOR_H