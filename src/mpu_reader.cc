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

void HandleString(const char* str) {
  Serial.print("Got command: ");
  Serial.println(str);
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
  animator.SetServoParams(&settings_manager.settings().servo_param[0]);
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
        HandleString(cmd);
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
