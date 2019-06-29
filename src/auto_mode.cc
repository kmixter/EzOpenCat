#include "auto_mode.h"

#ifdef TESTING
#include <stdio.h>
#define HDEBUG(_A) _A
#else
#define HDEBUG(_A)
#include <Arduino.h>
#endif  // TESTING

#include "prng.h"
#include "servo_animator.h"

static AutoMode::StateData s_state_data[kStateCount] = {
  {
    { 0, 50, 20, 20, 10, 0 },
    0, 0,
    10,
    kAnimationRest,
  },
  {
    { 0, 0, 40, 40, 20, 0  },  // StateStretch
    2, 1,
    10,
    kAnimationStretch
  },
  {
    { 0, 0, 0, 40, 30, 30 },  // StateBalance
    10, 5,  // StateBalance
    4,
    kAnimationBalance,
  },
  {
    { 0, 0, 60, 0, 10, 30 },  // StateSit
    10, 5, // StateSit
    4,
    kAnimationSit,
  },
  {
    { 0, 0, 30, 40, 0, 30 },  // StateWalkInPlace
    2, 1,  // StateWalkInPlace
    2,
    kAnimationWalkInPlace,
  },
  {
    { 0, 40, 20, 20, 0, 20 },  // StateSleeping
    20, 4,  // StateSleeping
    10,
    kAnimationRest
  }
};

void AutoMode::Initialize(ServoAnimator* animator, PRNG* prng) {
  state_data_ = s_state_data;
  servo_animator_ = animator;
  prng_ = prng;
}

void AutoMode::SetEnabled(bool enabled) {
  if (servo_animator_ == nullptr) return;
  if (enabled == enabled_) return;

  millis_next_state_ = 0;
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
  HDEBUG(printf("%d now %lu state %d next %lu\n", __LINE__, millis_now, state_,
                millis_next_state_));
  if (!enabled_)
    return;

  if (millis_next_state_ && millis_now < millis_next_state_) {
    return;
  }

  StateData* old_state_data = &state_data_[state_];

#ifndef TESTING
  Serial.println(F("AutoMode state transition"));
#endif  // TESTING

  int new_state_roll = prng_->Roll(100);
  HDEBUG(printf("%d random %d\n", __LINE__, new_state_roll));
  int new_state = 0;
  int accum_prob = 0;

  for (int i = 0; i < kStateCount; ++i) {
    int new_state_prob = old_state_data->next_state_prob[i];
    if (accum_prob <= new_state_roll && new_state_prob != 0)
      new_state = i;
    accum_prob += new_state_prob;
  }

  state_ = AutoModeState(new_state);
  HDEBUG(printf("%d state_ updated to %d\n", __LINE__, new_state));

  StateData* new_state_data = &state_data_[state_];
  int seconds_delay = new_state_data->seconds_exit_median;

  if (new_state_data->seconds_exit_variance > 0) {
    int seconds_variance = prng_->Roll(new_state_data->seconds_exit_variance * 2) -
        new_state_data->seconds_exit_variance;
    HDEBUG(printf("%d seconds_variance %d\n", __LINE__, seconds_variance));
    seconds_delay += seconds_variance;
  }
  HDEBUG(printf("%d seconds_delay %d\n", __LINE__, seconds_delay));

  int new_animation = new_state_data->animation_sequence;
  int ms_per_degree = new_state_data->ms_per_degree;
  millis_next_state_ = millis_now + seconds_delay * 1000;
  HDEBUG(printf("%d millis_next_state_ %lu\n", __LINE__, millis_next_state_));

#ifndef TESTING
  Serial.print(F("Setting animation "));
  Serial.print(new_animation);
  Serial.print(F(" ms_per_degree "));
  Serial.println(ms_per_degree);
#endif  // TESTING
  servo_animator_->set_ms_per_degree(ms_per_degree);
  servo_animator_->StartAnimation(new_animation, millis_now);
}
