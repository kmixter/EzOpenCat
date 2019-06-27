#ifndef _SERVO_ANIMATOR_TESTFAKE_H
#define _SERVO_ANIMATOR_TESTFAKE_H

#include "servo_animator.h"

class ServoAnimatorTestFake : public ServoAnimator {
 public:
  ServoAnimatorTestFake();

  void StartAnimation(int animation, unsigned long millis_now) override {
    animation_sequence_ = animation;
    animating_ = true;
  }

  void set_animating(bool animating) { animating_ = animating; }
};

#endif  // _SERVO_ANIMATOR_TESTFAKE_H
