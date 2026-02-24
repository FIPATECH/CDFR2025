#include "globals.h"
#include "odom_logger.h"

void odom_logger_task(void *pv) {
  OdomData_t o;
  for (;;) {
    if (xQueueReceive(odomQueue, &o, portMAX_DELAY) == pdTRUE) {
      Serial.print(F("ODOM vR=")); Serial.print(o.vR, 2);
      Serial.print(F("\tvL="));    Serial.print(o.vL, 2);
      Serial.print(F("\tcountR=")); Serial.print(o.cR);
      Serial.print(F("\tcountL=")); Serial.println(o.cL);
    }
  }
}
