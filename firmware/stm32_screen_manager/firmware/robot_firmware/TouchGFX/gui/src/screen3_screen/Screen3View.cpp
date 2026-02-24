#include <gui/screen3_screen/Screen3View.hpp>
#include "uart_handler.h"

Screen3View::Screen3View()
    : Screen3ViewBase()
{
}

void Screen3View::setupScreen()
{
    Screen3ViewBase::setupScreen();
}

void Screen3View::tearDownScreen()
{
    Screen3ViewBase::tearDownScreen();
}

void Screen3View::handleTickEvent()
{
    Screen3ViewBase::handleTickEvent();

    if (uartStartMatchFlag)
    {
        uartStartMatchFlag = false;
        presenter->startMatch();
    }
}
