#include <Arduino.h>

#include "eeprom_settings.h"
#include "mpu6050.h"
#include "servo_animator.h"

static const int kMpuI2CAddr = 0x68;
static const float kDt = 10;

// Once gyro is differing by only 2 between successive runs, we're happy.
static const int kSettleGyroDiff = 2;

EepromSettingsManager s_eeprom_settings;
ServoAnimator s_servo_animator;

class MenuObserver {
 public:
  MenuObserver() {}
  virtual void Show() = 0;
  virtual void HandleKey(char key) = 0;
  virtual bool HandleSelection() = 0;
  virtual ~MenuObserver() {};
};

void UpdateMenu(int cursor, const char* message, const char** options, MenuObserver** observers)
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
    if (observers != nullptr) {
      if (observers[i] != nullptr)
        observers[i]->Show();
    }
    Serial.println("");
  }
  Serial.print("\e[0m");
}

int GetSelection(const char* message, const char** options, MenuObserver** observers = nullptr) {
  int cursor = 0;
 
  Serial.print("\e[2J");
  UpdateMenu(cursor, message, options, observers);
 
  while (true) {
    yield();
    if (!Serial.available()) continue;

    uint8_t b = Serial.read();
    switch (b) {
      case '\r':
        if (observers != nullptr && observers[cursor] != nullptr) {
          if (!observers[cursor]->HandleSelection())
            break;
        }
        return cursor;
      case '\e':
        break;
      default:
        observers[cursor]->HandleKey(b);
        continue;
    }

    delay(10);
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
    if (b == 'C') {  // Right
      if (observers[cursor] != nullptr)
        observers[cursor]->HandleKey(2);
    }
    if (b == 'D') {  // Left
      if (observers[cursor] != nullptr)
        observers[cursor]->HandleKey(1);
    }
    UpdateMenu(cursor, message, options, observers);
  }
}

void CalibrateServos() {
  const char* kServos[] = {
    "Back <<",
    "Head",
    "Neck",
    "Left Front Knee",
    "Left Front Shoulder",
    "Right Front Knee",
    "Right Front Shoulder",
    "Left Back Shoulder",
    "Left Back Knee",
    "Right Back Shoulder",
    "Right Back Knee",
    "Tail",
    nullptr
  };
  int8_t* values = &s_eeprom_settings.settings().servo_zero_offset[0];
  class ServoCalibrationMenu : public MenuObserver {
   public:
    ServoCalibrationMenu(int8_t* value) : value_(value) {}

    void Show() override {
      Serial.print("  \e[0m\e[30m\e[47m");
      int pad = *value_ < 0 ? 0 : 1;
      int v = abs(*value_);
      if (v < 10) {
        pad += 2;
      } else if (v < 100) {
        pad += 1;
      }
      while (pad--) Serial.print(" ");
      Serial.print(*value_);
    }

    bool HandleSelection() override {
      return false;
    }

    void HandleKey(char key) override {
      bool any_change = false;
      if (key == 2 || key == '+') {  // up
        ++*value_;
        any_change = true;
      } else if (key == 1 || key == '-') {  // value;
        --*value_;
        any_change = true;
      }
      if (any_change)
        s_servo_animator.SetFrame(s_servo_animator.GetFrame(kAnimationCalibrationPose, 0), millis());
    }

    ~ServoCalibrationMenu() override {}

   protected:
    int8_t* value_;
  };

  MenuObserver** observers = new MenuObserver*[12];
  observers[0] = nullptr;
  for (int i = 1; i < 12; ++i) {
    observers[i] = new ServoCalibrationMenu(&values[i - 1]);
  }

  s_servo_animator.Attach();
  s_servo_animator.SetFrame(s_servo_animator.GetFrame(kAnimationCalibrationPose, 0), millis());

  GetSelection("Choose which servo to calibrate:", kServos, observers);

  for (int i = 1; i < 12; ++i) {
    delete observers[i];
  }

  delete[] observers;
  s_eeprom_settings.Store();
  s_servo_animator.Detach();
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

void SetPose() {
  const char* kPoseSelections[] = {
    "Back <<",
    "Rest Pose",
    "Calibrate Pose",
    nullptr
  };

  class PoseMenu : public MenuObserver {
   public:
    PoseMenu(int animation) : animation_(animation) {}

    void Show() override {}

    void HandleKey(char key) override {
    }

    bool HandleSelection() override {
      s_servo_animator.SetFrame(s_servo_animator.GetFrame(animation_, 0), millis());
      return false;
    }

    ~PoseMenu() override {}

   protected:
    int animation_;
  };

  s_servo_animator.Attach();

  MenuObserver** observers = new MenuObserver*[3];
  observers[0] = nullptr;
  observers[1] = new PoseMenu(kAnimationRest);
  observers[2] = new PoseMenu(kAnimationCalibrationPose);

  // Only selection returned by GetSelection will be back.
  GetSelection("Pick one:", kPoseSelections, observers);

  s_servo_animator.Detach();
}

int main() {
  init();

  // initialize serial communication at 9600 bits per second:
  Serial.begin(57600);

  s_eeprom_settings.Initialize();
  s_servo_animator.Initialize();
  s_servo_animator.SetServoParams(&s_eeprom_settings.settings().servo_zero_offset[0]);

  while (true) {
    const char* kTopMenuSelections[] = {
      "Calibrate Servos",
      "Calibrate MPU",
      "Set Pose",
      nullptr
    };

    switch (GetSelection("Pick one:", kTopMenuSelections)) {
      case 0:
        CalibrateServos();
        break;
      case 1:
        CalibrateMPU();
        break;
      case 2:
        SetPose();
        break;
    }
  }
}
