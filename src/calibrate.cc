#include <Arduino.h>

#include "eeprom_settings.h"
#include "mpu6050.h"
#include "servo_animator.h"

static const int kMpuI2CAddr = 0x68;
static const float kDt = 10;

// Once gyro is differing by only 2 between successive runs, we're happy.
static const int kSettleGyroDiff = 2;

EepromSettingsManager s_eeprom_settings;

void UpdateMenu(int cursor, const char* message, const char** options)
{
  Serial.print("\e[1;1H\e[0m\e[1m");
  Serial.println(message);
  Serial.print("\e[0m");
  int max_strlen = 0;
  for (int i = 0; options[i] != nullptr; ++i) {
    int sl = strlen(options[i]);
    max_strlen = sl > max_strlen ? sl : max_strlen;
  }

  for (int i = 0; options[i] != nullptr; ++i) {
    if (cursor == i)
      Serial.print("\e[41m");
    else
      Serial.print("\e[0m");
    Serial.print(options[i]);
    for (int pad = max_strlen - strlen(options[i]); pad > 0; --pad)
      Serial.print(" ");
    Serial.println("");
  }
  Serial.print("\e[0m");
}

int GetSelection(const char* message, const char** options) {
  int cursor = 0;
 
  Serial.print("\e[2J");
  UpdateMenu(cursor, message, options);
 
  while (true) {
    yield();
    if (!Serial.available()) continue;

    uint8_t b = Serial.read();
    switch (b) {
      case '\r':
        return cursor;
      case '\e':
        break;
      default:
        continue;
    }

    delay(100);
    b = Serial.read();
    if (b != '[') continue;

    b = Serial.read();
    if (b == 'A') {  // Up
      if (cursor > 0)
        --cursor;
    }
    if (b == 'B') {  // Down
      if (options[cursor + 1] != nullptr)
        ++cursor;
    }
    UpdateMenu(cursor, message, options);
  }
}

void CalibrateServos() {
  const char* kServos[] = {
    "Back <<",
    "Head",
    "Neck",
    "Left Front Knee",
    "Right Front Knee",
    "Left Front Shoulder",
    "Right Front Shoulder",
    "Left Back Shoulder",
    "Right Back Shoulder",
    "Left Back Knee",
    "Right Back Knee",
    "Tail",
    nullptr
  };
  GetSelection("Choose which servo to calibrate:", kServos);
}

void CalibrateMPU() {
  const char* kChoices[] = {
    "Back <<",
    "Continue >>",
    nullptr
  };

  Serial.print("Stored gyro correction: ");
  for (int i = 0; i < 3; ++i) {
    Serial.print(s_eeprom_settings.settings().gyro_correction[i]);
    Serial.print(" ");
  }
  Serial.println("");
  delay(2000);

  if (!GetSelection("Lay cat completely flat.", kChoices)) return;

  MPU6050 mpu(kMpuI2CAddr, kDt / 1000, 1);
  mpu.Initialize();

  Serial.println("\e[2J\e[1m\e[1;1HPlease wait...\e[0m\n");

  int accel_correction[3] = {0};
  int gyro_correction[3] = {0};

  while (true) {
    Serial.print("\e[3;1Haccl correction: ");
    for (int i = 0; i < 3; ++i) {
      Serial.print(accel_correction[i]);
      Serial.print(" ");
    }
    Serial.println("");
    Serial.print("gyro correction: ");
    for (int i = 0; i < 3; ++i) {
      Serial.print(gyro_correction[i]);
      Serial.print(" ");
    }
    Serial.println("");

    long sum_accel[3] = {0};
    long sum_gyro[3] = {0};
    const int kSamples = 100;
    for (int count = 0; count < kSamples; ++count) {
      int16_t accel[3];
      int16_t gyro[3];
      mpu.ReadBoth(accel, gyro);

      if (count % 10 == 0) {
        Serial.print("\e[6;1H");
        Serial.print("accl: "); Serial.print(accel[0]); Serial.print(" "); Serial.print(accel[1]); Serial.print(" "); Serial.println(accel[2]);
        Serial.print("gyro: "); Serial.print(gyro[1]); Serial.print(":"); Serial.println(gyro[0]);

        Serial.print("perc: ");
        Serial.print(count * 100 / kSamples);
        Serial.println("%  ");
      }
      for (int i = 0; i < 3; ++i)
        sum_accel[i] += accel[i];
      for (int i = 0; i < 3; ++i)
        sum_gyro[i] += gyro[i];
      delay(kDt);
    }

    int next_accel_correction[3];
    int next_gyro_correction[3];
    for (int i = 0; i < 3; ++i)
      next_accel_correction[i] = -sum_accel[i] / kSamples;
    for (int i = 0; i < 3; ++i)
      next_gyro_correction[i] = -sum_gyro[i] / kSamples;

    // Compare if close enough.
    int abs_gyro_diff = 0;
    for (int i = 0; i < 3; ++i)
      abs_gyro_diff += abs(next_gyro_correction[i] - gyro_correction[i]);

    memcpy(accel_correction, next_accel_correction, sizeof(next_accel_correction));
    memcpy(gyro_correction, next_gyro_correction, sizeof(next_gyro_correction));

    if (abs_gyro_diff <= kSettleGyroDiff)
      break;
  }

  for (int i = 0; i < 3; ++i)
    s_eeprom_settings.settings().gyro_correction[i] = gyro_correction[i];

  s_eeprom_settings.Store();
}

int main() {
  init();

  // initialize serial communication at 9600 bits per second:
  Serial.begin(57600);
  s_eeprom_settings.Initialize();

  while (true) {
    const char* kTopMenuSelections[] = {
      "Calibrate Servos",
      "Calibrate MPU",
      nullptr
    };

    switch (GetSelection("Pick one:", kTopMenuSelections)) {
      case 0:
        CalibrateServos();
        break;
      case 1:
        CalibrateMPU();
        break;
    }
  }
}

void yield() {
  if (serialEventRun) serialEventRun();
}
