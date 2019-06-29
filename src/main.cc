#include <Arduino.h>

#include "auto_mode.h"
#include "eeprom_settings.h"
#include "mpu6050.h"
#include "prng.h"
#include "remote_control.h"
#include "servo_animator.h"

static const int kMpuI2CAddr = 0x68;
static const float kDt = 10;
static const float kTau = 500;

static EepromSettingsManager s_eeprom_settings;
static ServoAnimator s_servo_animator;
#ifdef MPU
static MPU6050 s_mpu(kMpuI2CAddr, kTau / 1000, kDt / 1000);
#endif  // MPU
static RemoteControl s_control(A0);
static AutoMode s_auto;
static SmallPRNG s_prng(0);

class MyControlObserver : public ControlObserver {
 public:
  void OnRemoteKey(RemoteKey key) {
    Serial.print(F("Serial received: ")); Serial.println(key);
    last_key_ = key;
  }

  bool Get(RemoteKey* result) {
    if (last_key_ == kKeyMax)
      return false;
    *result = last_key_;
    last_key_ = kKeyMax;
    return true;
  }

  void Flush() {
    last_key_ = kKeyMax;
  }

  RemoteKey last_key_ = kKeyMax;
};

static MyControlObserver s_control_observer;


static void ResetServos() {
  s_servo_animator.Attach();
  delay(500);
  s_servo_animator.Detach();
}

static void UpdateWalkingAnimation(int walk_mode, const int walk_modes[][3], int walk_modes_max, int* next_animation) {
  Serial.print("New walk mode ");
  Serial.println(walk_mode);

  for (int i = 0; i < walk_modes_max; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (s_servo_animator.animation_sequence() == walk_modes[i][j]) {
        *next_animation = walk_modes[walk_mode][j];
        Serial.print("Updating to ");
        Serial.println(*next_animation);
      }
    }
  }
}

static void HandleKey(RemoteKey key, int* ms_per_degree) {
  int walk_mode = 0;
  const int walk_modes[][3] = {
    { kAnimationWalkLeft, kAnimationWalk, kAnimationWalkRight },
    { kAnimationTrLeft, kAnimationTr, kAnimationTrRight },
    { kAnimationCrawlLeft, kAnimationCrawl, kAnimationCrawlRight }
  };
  const int walk_modes_max = sizeof(walk_modes[0]) / sizeof(walk_modes[0][0]);

  int next_animation = kAnimationSingleFrame;
  switch (key) {
    case kKeyPause: {
      next_animation = kAnimationRest;
      if (s_servo_animator.animation_sequence() == kAnimationRest)
        next_animation = kAnimationBalance;
      break;
    }
    case kKeyPrev:
      walk_mode--;
      if (walk_mode < 0)
        walk_mode = walk_modes_max;
      UpdateWalkingAnimation(walk_mode, walk_modes, walk_modes_max, &next_animation);
      break;
    case kKeyNext:
      walk_mode++;
      if (walk_mode == walk_modes_max)
        walk_mode = 0;
      UpdateWalkingAnimation(walk_mode, walk_modes, walk_modes_max, &next_animation);
      break;
    case kKeyEq:
      next_animation = kAnimationStretch;
      break;
    case kKey1:
      next_animation = walk_modes[walk_mode][0];
      break;
    case kKey2:
      next_animation = walk_modes[walk_mode][1];
      break;
    case kKey3:
      next_animation = walk_modes[walk_mode][2];
      break;
    case kKey4:
      next_animation = kAnimationWalkInPlace;
      break;
    case kKey5:
      next_animation = kAnimationSit;
      break;
    case kKey6:
      next_animation = kAnimationFistBump;
      break;
    case kKey7:
      next_animation = kAnimationBackUpLeft;
      break;
    case kKey8:
      next_animation = kAnimationBackUp;
      break;
    case kKey9:
      next_animation = kAnimationBackUpRight;
      break;
    case kKeyMinus:
      ++*ms_per_degree;
      s_servo_animator.set_ms_per_degree(*ms_per_degree);
      Serial.print(*ms_per_degree);
      Serial.println(F("ms/deg"));
      break;
    case kKeyPlus:
      if (*ms_per_degree > 1) {
        --*ms_per_degree;
        s_servo_animator.set_ms_per_degree(*ms_per_degree);
      }
      Serial.print(*ms_per_degree);
      Serial.println(F("ms/deg"));
      break;
    default:
      Serial.println(F("Unhandled"));
      break;
  }
  if (next_animation != kAnimationSingleFrame) {
    if (s_servo_animator.animation_sequence() == next_animation)
      next_animation = kAnimationBalance;
    s_servo_animator.StartAnimation(next_animation, millis());
  }
}

int main() {
  init();

  // initialize serial communication at 9600 bits per second:
  Serial.begin(57600);

  s_eeprom_settings.Initialize();
  s_servo_animator.Initialize();
  s_servo_animator.SetEepromSettings(&s_eeprom_settings.settings());
#ifdef MPU
  s_mpu.Initialize();
  s_mpu.SetGyroCorrection(s_eeprom_settings.settings().gyro_correction);
  s_mpu.SetPitchRollCorrection(s_eeprom_settings.settings().pitch_correction,
                               s_eeprom_settings.settings().roll_correction);
#endif  // MPU
  s_control.Initialize();
  s_auto.Initialize(&s_servo_animator, &s_prng);

  ResetServos();

  Serial.println(F("Ready..."));

  s_prng.SetSeed(micros());

  unsigned long last_manual_animation_done_time = 0;
  bool first_key = true;
  const int auto_mode_reenter_timeout = 10000;
  int manual_mode_ms_per_degree = 4;
  s_servo_animator.set_ms_per_degree(manual_mode_ms_per_degree);

  s_auto.SetEnabled(true);

  while (true) {
    RemoteKey key;
    unsigned long millis_now = millis();

    if (s_control_observer.Get(&key)) {
      if (first_key) {
        // First key press is our first entropy event. Use it.
        s_prng.SetSeed(micros());
        first_key = false;
      }
      s_auto.SetEnabled(false);
      last_manual_animation_done_time = 0;
      HandleKey(key, &manual_mode_ms_per_degree);
      continue;
    }

    if (!s_auto.enabled() && last_manual_animation_done_time &&
          millis_now - last_manual_animation_done_time > auto_mode_reenter_timeout) {
      s_auto.SetEnabled(true);
    }


    if (!s_auto.enabled()) {
      if (!last_manual_animation_done_time && !s_servo_animator.animating()) {
        last_manual_animation_done_time = millis_now;
      }
      yield();
      continue;
    }

    s_auto.Update(millis_now);
    yield();
  }

}

void yield() {
  unsigned long millis_now = millis();
  s_servo_animator.Animate(millis_now);
  s_control.ReadAndDispatch(&s_control_observer);

#ifdef MPU
  static long millis_last_mpu = 0;
  if (millis_now - millis_last_mpu >= kDt) {
    millis_last_mpu = millis_now;
    int16_t accel[3];
    int16_t gyro[3];
    s_mpu.ReadBoth(accel, gyro);
    float pitch, roll;
    s_mpu.ComputeFilteredPitchRoll(accel, gyro, &pitch, &roll);
    s_servo_animator.HandlePitchRoll(pitch, roll, millis_now);
  }
#endif

  if (serialEventRun) {
    serialEventRun();
  }
}
