#include <Arduino.h>

// 1ms is not enough because delay() itself can overshoot by 2ms.
static const int kWarningDeltaMicros = 3000;

void yield() {
  static unsigned long last_micros = 0;
  unsigned long this_micros = micros();
  if (last_micros && this_micros - last_micros > kWarningDeltaMicros) {
    Serial.print("Slow yield: ");
    Serial.print(this_micros - last_micros);
    Serial.print(" @");
    Serial.println(this_micros);
  }
  if (serialEventRun) serialEventRun();

  last_micros = micros();
}
