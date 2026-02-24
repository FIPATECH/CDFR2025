// #include "autonomous_sequence.h"
// #include "globals.h"       // cmdVG, cmdVD, matchStopReceived
// #include <Arduino.h>       // pour Serial

// // ---------------------------------------------------------------------------------
// // Constantes de durée (calibrage à adapter)
// static const TickType_t TURN_90_MS   = pdMS_TO_TICKS(800);   // rotation 90 °
// static const TickType_t TURN_180_MS  = pdMS_TO_TICKS(1600);  // rotation 180 °
// static const TickType_t PAUSE_SHORT  = pdMS_TO_TICKS(500);   // pause 0,5 s
// static const TickType_t PAUSE_MEDIUM = pdMS_TO_TICKS(1000);  // pause 1 s
// // ---------------------------------------------------------------------------------

// // Petite fonction interne pour faciliter l’« abort » si STOP_MATCH reçu
// static inline bool exitIfStop()
// {
//   if (matchStopReceived) {
//     cmdVG = 0;                  // assure l’arrêt immédiat
//     cmdVD = 0;
//     Serial.println(F("autonomousSequence: ABORT (STOP_MATCH)"));
//     return true;                // stop request, quit sequence
//   }
//   return false;
// }

// void autonomousSequence()
// {
//   Serial.println(F("autonomousSequence: start"));

//   // 1) Reculer 2 s
//   cmdVG = -15;  cmdVD = -15;
//   vTaskDelay(pdMS_TO_TICKS(2000));
//   if (exitIfStop()) return;

//   // 2) Arrêt 1 s
//   cmdVG = 0;  cmdVD = 0;
//   vTaskDelay(PAUSE_MEDIUM);
//   if (exitIfStop()) return;

//   // 3) Avancer 5 s
//   cmdVG = 15;  cmdVD = 15;
//   vTaskDelay(pdMS_TO_TICKS(5000));
//   if (exitIfStop()) return;

//   // 4) Arrêt 1 s
//   cmdVG = 0;  cmdVD = 0;
//   vTaskDelay(PAUSE_MEDIUM);
//   if (exitIfStop()) return;

//   // 5) Tourner 90 ° à droite
//   cmdVG = 15;  cmdVD = -15;
//   vTaskDelay(TURN_90_MS);
//   cmdVG = 0;    cmdVD = 0;
//   vTaskDelay(PAUSE_SHORT);
//   if (exitIfStop()) return;

//   // 6) Avancer 5 s
//   cmdVG = 15;  cmdVD = 15;
//   vTaskDelay(pdMS_TO_TICKS(5000));
//   cmdVG = 0;    cmdVD = 0;
//   vTaskDelay(PAUSE_SHORT);
//   if (exitIfStop()) return;

//   // 7) Demi-tour 180 °
//   cmdVG = 15;  cmdVD = -15;
//   vTaskDelay(TURN_180_MS);
//   cmdVG = 0;    cmdVD = 0;
//   vTaskDelay(PAUSE_SHORT);
//   if (exitIfStop()) return;

//   // 8) Reculer 2 s
//   cmdVG = -15; cmdVD = -15;
//   vTaskDelay(pdMS_TO_TICKS(2000));

//   // Arrêt final
//   cmdVG = 0;    cmdVD = 0;

//   Serial.println(F("autonomousSequence: done"));
// }

// // -----------------------------------------------------------------------------
// //                     TÂCHE WRAPPER POUR FREE RTOS
// // -----------------------------------------------------------------------------
// void autonomous_task(void *pvParameters)
// {
//   (void)pvParameters;
//   autonomousSequence();      // exécute la séquence
//   vTaskDelete(NULL);         // auto-détruit la tâche
// }
