#include <cstdio>
#include "points_manager.h"
#include "uart_handler.h"
#include <gui/screen4_screen/Screen4Presenter.hpp>
#include "cmsis_os.h"
#include <cstring>

osMessageQueueId_t pointsQueueHandle = nullptr;
static osThreadId_t pointsTaskHandle = nullptr;

static Screen4Presenter *presenterPtr = nullptr;

static osThreadAttr_t pointsTaskAttr;

static void Points_Task(void *argument);

void PointsManager_Init(void)
{
    std::memset(&pointsTaskAttr, 0, sizeof(pointsTaskAttr));

    pointsTaskAttr.name = "Points_Task";
    pointsTaskAttr.stack_size = 256 * 4;
    pointsTaskAttr.priority = osPriorityNormal;

    pointsQueueHandle = osMessageQueueNew(1, sizeof(uint32_t), nullptr);
    pointsTaskHandle = osThreadNew(Points_Task, nullptr, &pointsTaskAttr);
}

void PointsManager_SetPresenter(Screen4Presenter *p)
{
    presenterPtr = p;
}

static void Points_Task(void *argument)
{
    (void)argument;
    uint32_t pts;
    for (;;)
    {
        if (osMessageQueueGet(pointsQueueHandle, &pts, nullptr, osWaitForever) == osOK)
        {
            // Log via UART_Send_Raw
            {
                char buf[64];
                int len = snprintf(buf, sizeof(buf),
                                   "Points_Task: reçu pts=%lu\r\n",
                                   (unsigned long)pts);
                if (len > 0)
                    UART_Send_Raw(buf);
            }

            if (presenterPtr)
            {
                presenterPtr->updatePoints(pts);
            }
        }
    }
}
