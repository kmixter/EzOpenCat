#include "servo_animator.h"

#ifndef TESTING
#include <Arduino.h>
#define pgm_read_int8(_a) (int8_t)pgm_read_byte(_a)
#else
#define PROGMEM
#define pgm_read_int8(_a) (*(_a))
#include <cstdlib>
#include <stdio.h>
#endif  // TESTING

#include <math.h>
#include <string.h>

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

const int ServoAnimator::kDirectionMap[kServoCount] = {
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
  int total_frames = pgm_read_int8(instinct);
  int frame_dofs = 16;
  if (total_frames < 0) {
    total_frames = -total_frames;
    frame_dofs = ActualDOF;
  } else if (total_frames > 1) {
    frame_dofs = WalkingDOF;
  }
  if (number >= total_frames)
    return nullptr;

  const char* frame_start = instinct + 3 + frame_dofs * number;
  if (frame_dofs == 16 || frame_dofs == ActualDOF) {
    result[kServoHead] = pgm_read_int8(frame_start + 0);
    result[kServoNeck] = pgm_read_int8(frame_start + 1);
    result[kServoTail] = pgm_read_int8(frame_start + 2);
    if (frame_dofs == 16)
      walking_frame = frame_start + 8;
    else
      walking_frame = frame_start + 3;
  } else if (frame_dofs == WalkingDOF) {
    walking_frame = frame_start;
    result[kServoHead] = 0;
    result[kServoNeck] = 0;
    result[kServoTail] = 0;
  } else {
    return nullptr;
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
  ResetAnimation();
  // we cannot sense initial position from servos, so assume starting
  // at rest position.
  memcpy(current_positions_, GetFrame(kAnimationRest, 0),
         sizeof(current_positions_));
  memcpy(target_unbalanced_frame_, current_positions_, sizeof(target_unbalanced_frame_));
  animation_sequence_ = kAnimationRest;
}

void ServoAnimator::WriteServo(int servo, int logical_angle) {
  if (logical_angle > eeprom_settings_->servo_upper_extents[servo])
    logical_angle = eeprom_settings_->servo_upper_extents[servo];
  if (logical_angle < eeprom_settings_->servo_lower_extents[servo])
    logical_angle = eeprom_settings_->servo_lower_extents[servo];
  int real_angle = ConvertToRealAngle(servo, logical_angle);
  servo_[servo]->write(real_angle);
  //printf("Writing servo %d to logical %d, real %d\n", servo, logical_angle,
  //       real_angle);
  current_positions_[servo] = logical_angle; 
}


int ServoAnimator::ConvertToRealAngle(int servo, int angle) {
  return 90 + (angle + eeprom_settings_->servo_zero_offset[servo]) *
      kDirectionMap[servo];
}

void ServoAnimator::Attach() {
  for (int i = 0; i < kServoCount; ++i) {
    servo_[i]->attach(kPinMap[i]);
    WriteServo(i, current_positions_[i]);
  }
}

void ServoAnimator::Detach() {
  for (int i = 0; i < kServoCount; ++i)
    servo_[i]->detach();
}

void ServoAnimator::SetEepromSettings(const EepromSettings* settings) {
  eeprom_settings_ = settings;
}

// To get taller:
// Shoulders go towards 0, knees go towards 90 (negative 90 for back).

// To get shorter:
// Shoulders go towards 90 (negative in back), knees go towards 0

void ServoAnimator::ComputeBalancedFrame() {
  target_balanced_frame_[0] = AngleAdd(target_unbalanced_frame_[0], pitch_);
  target_balanced_frame_[1] = AngleAdd(target_unbalanced_frame_[1], -roll_);
  target_balanced_frame_[2] = AngleAdd(target_unbalanced_frame_[2], -roll_ * 2);
  bool leaning_left = roll_ > 0;
  for (int i = int(kServoLeftFrontShoulder); i <= int(kServoLeftBackShoulder); ++i) {
    bool is_left_leg = i == kServoLeftBackShoulder || i == kServoLeftFrontShoulder;
    int lean_factor = is_left_leg == leaning_left ? -2 : -3;
    int back_factor = i >= kServoRightBackShoulder ? -1 : 1;
    target_balanced_frame_[i] = AngleAdd(target_unbalanced_frame_[i],
        -9 * pitch_ / 10 + abs(roll_) * back_factor * -lean_factor / 2);
  }
  for (int i = int(kServoLeftFrontKnee); i <= int(kServoLeftBackKnee); ++i) {
    bool is_left_leg = i == kServoLeftBackKnee || i == kServoLeftFrontKnee;
    int lean_factor = is_left_leg == leaning_left ? -2 : -3;
    int back_factor = i >= kServoRightBackKnee ? -1 : 1;
    target_balanced_frame_[i] = AngleAdd(target_unbalanced_frame_[i],
        13 * pitch_ / 10 + abs(roll_) * back_factor * lean_factor * 7 / 10);
  }
}

void ServoAnimator::HandlePitchRoll(int pitch, int roll, unsigned long millis_now) {
  bool any_change = false;
  if (abs(pitch) > 90 || abs(roll) > 90) {
    pitch = 0;
    roll = 0;
  }
  if (abs(pitch_ - pitch) >= 10) {
    pitch_ = pitch;
    any_change = true;
  }
  if (abs(roll_ - roll) >= 10) {
    roll_ = roll;
    any_change = true;
  }
  if (any_change) {
    SetFrame(target_unbalanced_frame_, millis_now);
  }
}

int ServoAnimator::AngleAdd(int a1, int a2) {
  int result = a1 + a2;
  if (result < -127)
    return -127;
  if (result > 128)
    return 128;
  return result;
}

void ServoAnimator::StartFrame(const int8_t* new_frame, unsigned long millis_now) {
  SetFrame(new_frame, millis_now);
  animation_sequence_ = kAnimationSingleFrame;
}

void ServoAnimator::SetFrame(const int8_t* new_frame, unsigned long millis_now) {
  if (new_frame == nullptr) {
#ifndef TESTING
    Serial.println(F("Invalid frame"));
#endif
    return;
  }
  memcpy(start_frame_, current_positions_, sizeof(start_frame_));
  memcpy(target_unbalanced_frame_, new_frame, sizeof(target_unbalanced_frame_));
  ComputeBalancedFrame();
  millis_start_ = millis_now;
  animating_ = true;
}

void ServoAnimator::ResetAnimation() {
  animating_ = false;
  millis_start_ = 0;
  memset(start_frame_, 0, sizeof(start_frame_));
  memset(target_balanced_frame_, 0, sizeof(target_balanced_frame_));
}

void ServoAnimator::StartAnimation(int animation, unsigned long millis_now) {
  animation_sequence_ = animation;
  animation_sequence_frame_number_ = 0;
  Attach();
  SetFrame(GetFrame(animation, 0), millis_now);
}

void ServoAnimator::WaitUntilDone() const {
  while (animating()) {
#ifndef TESTING
    delay(100);
#endif
  }
}

void ServoAnimator::InterpolateToFrame(unsigned long millis_now,
                                       bool* done) {
  *done = false;
  if (millis_last_ == millis_now) {
    // Already ran at this millis clock, no updates possible.
    return;
  }
  millis_last_ = millis_now;

  bool any_not_done = false;
  unsigned long millis_elapsed = millis_now - millis_start_;

  for (int i = 0; i < kServoCount; ++i) {
    // Interpolate with smooth curve an angle transition based on
    // ms_per_degree.
    int total_angle_motion = target_balanced_frame_[i] - start_frame_[i];
    int abs_total_angle_motion = total_angle_motion;
    if (abs_total_angle_motion < 0)
      abs_total_angle_motion = -abs_total_angle_motion;
    int ms_for_angle_motion = abs_total_angle_motion * ms_per_degree_;
    float portion_done;
    if (ms_for_angle_motion == 0) {
      //printf("@%lums, servo %d: no motion\n", millis_now, i);
      portion_done = 1;
    } else {
      portion_done = (float)millis_elapsed / ms_for_angle_motion;
      if (portion_done > 1.0) portion_done = 1;
    }
    float portion_done_smoothed = (1 - cos(portion_done * M_PI)) / 2;
    int rounded_angle_motion;
    if (total_angle_motion > 0)
      rounded_angle_motion = portion_done_smoothed * total_angle_motion + .5;
    else
      rounded_angle_motion = portion_done_smoothed * total_angle_motion - .5;
    int angle_at_portion = start_frame_[i] + rounded_angle_motion;
    //printf("@%lums, %d: %d abs motion, %f (%f smoothed) done (%lu/%d), start %d, progress %d, logical angle %d\n", millis_now, i, abs_total_angle_motion, (double)portion_done, (double)portion_done_smoothed, millis_elapsed, ms_for_angle_motion, start_frame_[i], rounded_angle_motion, angle_at_portion);
#ifndef TESTING
#if 0
    Serial.print("Servo ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(new_frame[i]);
    Serial.print(", zeroed ");
    Serial.print(servo_zero_offsets_[i]);
    Serial.print(" so setting to ");
    Serial.println(angle);
#endif
#endif
    WriteServo(i, angle_at_portion);
    if (portion_done < 1.0)
      any_not_done = true;
  }

  *done = !any_not_done;
  //printf("At end: %d\n", any_not_done);
}

void ServoAnimator::StartNextAnimationFrame(unsigned long millis_now) {
  animation_sequence_frame_number_++;
  const int8_t* next_frame =
      GetFrame(animation_sequence_, animation_sequence_frame_number_);
  if (next_frame == nullptr) {
    if (animation_sequence_frame_number_ == 1) {
      if (animation_sequence_ == kAnimationRest) {
        // TODO: Create an observer interface and use it detach when we
        // observe Rest animation finishing.
        Detach();
      }
      ResetAnimation();
      return;
    }

    animation_sequence_frame_number_ = 0;
    next_frame = GetFrame(animation_sequence_, animation_sequence_frame_number_);
  }

  SetFrame(next_frame, millis_now);
}

void ServoAnimator::Animate(unsigned long millis_now) {
  if (!animating_) {
#ifdef TESTING
    printf("@%lums, not animating\n", millis_now);
#endif
    return;
  }

#ifndef TESTING
  if (millis_last_ && millis_now - millis_last_ > 10) {
    Serial.print(F("Slow animation: "));
    Serial.print(millis_now - millis_last_);
    Serial.print(F("ms @"));
    Serial.println(millis_now);
  }
#endif

  bool done_interpolation;

  InterpolateToFrame(millis_now, &done_interpolation);

  if (done_interpolation) {
    if (animation_sequence_ == kAnimationSingleFrame)
      // A direct SetFrame call finished.
      ResetAnimation();
    else
      StartNextAnimationFrame(millis_now);
  }
}
