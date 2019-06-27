#include "auto_mode.h"

#ifdef TESTING
#include <stdio.h>
#else
#include <Arduino.h>
#endif  // TESTING

#include "servo_animator.h"

void AutoMode::SetEnabled(bool enabled) {
  if (servo_animator_ == nullptr) return;
  if (enabled == enabled_) return;

  next_animation_delay_ = 0;
  last_animation_time_ = 0;
  if (!enabled) {
    servo_animator_->set_ms_per_degree(saved_ms_per_degree_);
  } else {
    saved_ms_per_degree_ = servo_animator_->ms_per_degree();
  }

#ifndef TESTING
  Serial.print(F("AutoMode "));
  Serial.println(enabled ? F("enabled") : F("disabled"));
#endif  // TESTING
  enabled_ = enabled;
}

void AutoMode::Update(unsigned long millis_now) {
  if (!enabled_)
    return;

  if (last_animation_time_ &&
        millis_now - last_animation_time_ < next_animation_delay_) {
    return;
  }

  int animation_speed = 4;
  int next_animation = kAnimationSingleFrame;

#ifndef TESTING
  Serial.println(F("AutoMode state transition"));
#endif  // TESTING

  switch (state_) {
   case Init:
    animation_speed = 10;  // Slow wakeup
    next_animation = kAnimationBalance;
    next_animation_delay_ = 1000;
    state_ = Stretch;
    break;

   case Stretch:
    animation_speed = 10;
    next_animation = kAnimationStretch;
    next_animation_delay_ = 2000;
    state_ = Wake;
    break;

   case Wake:
    animation_speed = 4;
    next_animation = kAnimationBalance;
    next_animation_delay_ = 10000;
    state_ = WalkInPlace;
    break;

   case WalkInPlace:
    animation_speed = 2;
    next_animation = kAnimationWalkPlace;
    next_animation_delay_ = 2000;
    state_ = WakeToRest;
    break;

   case WakeToRest:
    animation_speed = 4;
    next_animation = kAnimationBalance;
    next_animation_delay_ = 10000;
    state_ = FallAsleep;
    break;

   case FallAsleep:
    animation_speed = 20;
    next_animation = kAnimationRest;
    next_animation_delay_ = 20000;
    state_ = Stretch;
    break;

   default:
    break;
  }

  if (next_animation != kAnimationSingleFrame) {
#ifndef TESTING
    Serial.print(F("Setting animation "));
    Serial.print(next_animation);
    Serial.print(F(" speed "));
    Serial.println(animation_speed);
#endif  // TESTING
    last_animation_time_ = millis_now;
    servo_animator_->set_ms_per_degree(animation_speed);
    servo_animator_->StartAnimation(next_animation, millis_now);
  }
}
