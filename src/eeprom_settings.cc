#include "eeprom_settings.h"

#include <Arduino.h>
#include <EEPROM.h>

static const char kEepromSignature[] = "eM";

void EepromSettingsManager::Initialize() {
  EEPROM.get(0, settings_);
  if (memcmp(&settings_.signature, kEepromSignature, strlen(kEepromSignature)) == 0) {
  	return;
  }

  Serial.println("Creating zeroed settings");
  memset(&settings_, 0, sizeof(settings_));
  memcpy(&settings_.signature, kEepromSignature, strlen(kEepromSignature));
  EEPROM.put(0, settings_);
}

void EepromSettingsManager::Store() {
  EEPROM.put(0, settings_);
}
