#include <Arduino.h>

#include "eeprom_settings.h"
#include "mpu6050.h"
#include "remote_control.h"
#include "servo_animator.h"

static const int kMpuI2CAddr = 0x68;
static const float kGyroWeight = .98;
static const float kDt = 10;

class MyControlObserver : public ControlObserver {
 public:
  void OnRemoteKey(RemoteKey key) {
    Serial.print("Serial received: "); Serial.println(key);
  }
};

void HandleString(const char* str, ServoAnimator* animator) {
  Serial.print("Got command: ");
  Serial.println(str);
  int angles[11] = {0};
  int which = 0;
  int this_value = 0;
  bool this_negative = false;

  for (const char* p = str; *p; ++p) {
    if (*p == ' ') continue;
    if (*p == '-') {
      this_negative = true;
      continue;
    }
    if (*p >= '0' && *p <= '9') {
      this_value *= 10;
      this_value += *p - '0';
      continue;
    }
    if (*p == ',')  {
      angles[which] = this_value;
      if (this_negative)
        angles[which] *= -1;
      which++;
      if (which == 11) break;
      continue;
    }
    Serial.print("ignoring ");
    Serial.println(*p);
  }
  if (which == 8) {
    memmove(&angles[3], &angles[0], sizeof(int[8]));
    memset(&angles[0], 0, sizeof(int[2]));
  }


  Serial.print("Moving to ");
  for (int i = 0; i < 11; ++i) {
    Serial.print(angles[i]);
    Serial.print(", ");
  }
  Serial.println("");
  animator->SetFrame(angles);
}

int main() {
  init();

  // initialize serial communication at 9600 bits per second:
  Serial.begin(57600);

  MPU6050 mpu(kMpuI2CAddr, kDt / 1000, 1);
  mpu.Initialize();
  RemoteControl control(A0);
  MyControlObserver controlObserver;
  control.Initialize();
  ServoAnimator animator;
  EepromSettingsManager settings_manager;
  settings_manager.Initialize();
  animator.Initialize();
  animator.SetServoParams(&settings_manager.settings().servo_zero_offset[0]);
  animator.Attach();

  char cmd[80];
  char* cmd_cursor = cmd;

  for (int count = 0;; ++count) {
    float estimated_pitch = 0;
    float estimated_roll = 0;

    int16_t accel[3];
    int16_t gyro[3];
    mpu.ReadBoth(accel, gyro);

    mpu.ComputeFilteredPitchRoll(accel, gyro, &estimated_pitch, &estimated_roll);
    if (count % 10 == 0) {
      Serial.println("");
      Serial.print("accl: "); Serial.print(accel[0]); Serial.print(" "); Serial.print(accel[1]); Serial.print(" "); Serial.println(accel[2]);
      Serial.print("gyro: "); Serial.print(gyro[1]); Serial.print(":"); Serial.println(gyro[0]);

      Serial.print("outp: ");
      Serial.print(estimated_pitch);
      Serial.print(" ");
      Serial.print(estimated_roll);
      Serial.println("");
    }
    control.ReadAndDispatch(&controlObserver);
    //animator.Animate();
    while (Serial.available()) {
      int ch = Serial.read();
      if (ch == '\r') {
        *cmd_cursor = 0;
        HandleString(cmd, &animator);
        cmd_cursor = cmd;
      } else
        *cmd_cursor++ = ch;
    }
    delay(kDt);
  }
}

void yield() {
  if (serialEventRun) serialEventRun();
}
