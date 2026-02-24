#ifndef MATCH_TRIGGER_H
#define MATCH_TRIGGER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "cmsis_os.h"

#define BLUE_BTN_GPIO_PORT USER_BTN_GPIO_Port // USER_BTN (tirette)
#define BLUE_BTN_GPIO_PIN USER_BTN_Pin

  /* Fonctions de gestion de l'état du match */
  uint8_t IsTiretteRemoved(void);
  void MatchTrigger_StopCallback(void);
  void MatchTriggerTask(void *argument);

  void MatchTrigger_EXTI(uint16_t pin);

#ifdef __cplusplus
}
#endif

#endif /* MATCH_TRIGGER_H */
