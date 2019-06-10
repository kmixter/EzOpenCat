#include <Arduino.h>

#include "eeprom_settings.h"
#include "mpu6050.h"
#include "remote_control.h"
#include "servo_animator.h"

static const int kMpuI2CAddr = 0x68;
static const float kDt = 10;
static const float kTau = 500;

static EepromSettingsManager s_eeprom_settings;
static ServoAnimator s_servo_animator;
static MPU6050 s_mpu(kMpuI2CAddr, kTau / 1000, kDt / 1000);
static RemoteControl s_control(A0);

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

static void RunStartupSequence() {
  s_servo_animator.Attach();
  delay(500);
  s_servo_animator.Detach();
}

int main() {
  init();

  // initialize serial communication at 9600 bits per second:
  Serial.begin(57600);

  s_eeprom_settings.Initialize();
  s_servo_animator.Initialize();
  s_servo_animator.SetServoParams(&s_eeprom_settings.settings().servo_zero_offset[0]);
  s_servo_animator.set_ms_per_degree(2);
  s_mpu.Initialize();
  s_mpu.SetGyroCorrection(s_eeprom_settings.settings().gyro_correction);
  s_mpu.SetPitchRollCorrection(s_eeprom_settings.settings().pitch_correction,
                               s_eeprom_settings.settings().roll_correction);
  s_control.Initialize();

  RunStartupSequence();

  Serial.println(F("Ready..."));

  while (true) {
    RemoteKey key;
    if (s_control_observer.Get(&key)) {
      int next_animation = kAnimationSingleFrame;
      switch (key) {
        case kKeyPause: {
          next_animation = kAnimationBalance;
          if (s_servo_animator.animation_sequence() == kAnimationBalance)
            next_animation = kAnimationRest;
          break;
        }
        case kKey1:
          next_animation = kAnimationWalkLeft;
          break;
        case kKey2:
          next_animation = kAnimationWalk;
          break;
        case kKey3:
          next_animation = kAnimationWalkRight;
          break;
        case kKey5:
          next_animation = kAnimationSit;
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
    yield();
  }
}

void yield() {
  unsigned long millis_now = millis();
  s_servo_animator.Animate(millis_now);
  s_control.ReadAndDispatch(&s_control_observer);

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

  if (serialEventRun) {
    serialEventRun();
  }
}
