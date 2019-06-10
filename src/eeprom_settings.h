#ifndef _EEPROM_SETTINGS_H
#define _EEPROM_SETTINGS_H

#include <stdint.h>

static const int kNumServos = 11;

struct EepromSettings {
  uint8_t signature[3];

  int8_t servo_zero_offset[kNumServos];
  int8_t servo_upper_extents[kNumServos];
  int8_t servo_lower_extents[kNumServos];

  int16_t gyro_correction[3];
  int16_t pitch_correction;
  int16_t roll_correction;
};

class EepromSettingsManager {
 public:
  EepromSettingsManager() {}
  void Initialize();
  EepromSettings& settings() { return settings_; }
  void Store();

 private:
  EepromSettings settings_;
};

#endif  // _EEPROM_SETTINGS_H