#include "auto_mode.h"

#ifdef TESTING
#include <stdio.h>
#define HDEBUG(_A) _A
#define DDEBUG(_A)
#else
#define HDEBUG(_A)
#define DDEBUG(_A) _A
#include <Arduino.h>
#endif  // TESTING

#include <string.h>

#include "prng.h"
#include "servo_animator.h"

static AutoMode::StateData s_state_data[kStateCount] = {
  {
    { 0, 50, 50, 0, 0 },  // StateSleeping
    20, 4,  // StateSleeping
    10,
    false,
    kAnimationRest
  },
  {
    { 0, 0, 40, 40, 20 },  // StateStretch
    2, 1,
    10,
    false,
    kAnimationStretch
  },
  {
    { 30, 0, 0, 40, 30 },  // StateBalance
    10, 5,  // StateBalance
    4,
    true,
    kAnimationBalance,
  },
  {
    { 30, 0, 60, 0, 10 },  // StateSit
    10, 5, // StateSit
    4,
    true,
    kAnimationSit,
  },
  {
    { 30, 0, 30, 40, 0 },  // StateWalkInPlace
    2, 1,  // StateWalkInPlace
    2,
    false,
    kAnimationWalkInPlace,
  },
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

void AutoMode::LookAround(unsigned long millis_now) {
  if (millis_next_look_around_) {
    if (millis_now < millis_next_look_around_) {
      return;
    }

    int8_t new_frame[kServoCount];
    const int8_t* frame = servo_animator_->GetFrame(
        state_data_[state_].animation_sequence, 0);
    memcpy(new_frame, frame, sizeof(new_frame));

    new_frame[kServoHead] = 50 - prng_->Roll(100);
    new_frame[kServoNeck] = 70 - prng_->Roll(140);
    servo_animator_->set_ms_per_degree(10);
    servo_animator_->StartFrame(new_frame, millis_now);
  }
  int millis_next = 4000 - prng_->Roll(3500);
  millis_next_look_around_ = millis_now + millis_next;
  return;
}

void AutoMode::Update(unsigned long millis_now) {
  HDEBUG(printf("%d now %lu state %d next %lu\n", __LINE__, millis_now, state_,
                millis_next_state_));
  if (!enabled_)
    return;

  if (millis_next_state_ && millis_now < millis_next_state_) {
    if (!servo_animator_->animating() && look_around_enabled_ && 
          state_data_[state_].look_around)
      LookAround(millis_now);
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
  millis_next_look_around_ = 0;
}
