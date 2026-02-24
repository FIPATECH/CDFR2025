#include <cstdio>
#include "gui/screen4_screen/Screen4Presenter.hpp"
#include "gui/screen4_screen/Screen4View.hpp"
#include "points_manager.h" // PointsManager_SetPresenter()
#include "uart_handler.h"

Screen4Presenter::Screen4Presenter(Screen4View &v)
    : view(v)
{
}

void Screen4Presenter::activate()
{
    PointsManager_SetPresenter(this);
}

void Screen4Presenter::deactivate()
{
    PointsManager_SetPresenter(nullptr);
}

void Screen4Presenter::updatePoints(uint32_t pts)
{
    char log[48];
    int n = std::snprintf(log, sizeof(log),
                          "Presenter: updatePoints(%lu)\r\n",
                          static_cast<unsigned long>(pts));
    if (n > 0)
        UART_Send_Raw(log);

    view.updatePoints(pts);
}
