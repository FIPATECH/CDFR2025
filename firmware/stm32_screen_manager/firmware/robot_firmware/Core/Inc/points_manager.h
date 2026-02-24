
#ifndef POINTS_MANAGER_H
#define POINTS_MANAGER_H

#include "cmsis_os.h"

extern osMessageQueueId_t pointsQueueHandle;

#ifdef __cplusplus
extern "C"
{
#endif
  void PointsManager_Init(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
void PointsManager_SetPresenter(class Screen4Presenter *p);
#endif

#endif /* POINTS_MANAGER_H */
