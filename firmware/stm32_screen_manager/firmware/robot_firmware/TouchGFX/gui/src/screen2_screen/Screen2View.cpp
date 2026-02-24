#include <gui/screen2_screen/Screen2View.hpp>
#include "../../../Core/Inc/strategy_manager.h"

Screen2View::Screen2View()
{
}

void Screen2View::setupScreen()
{
    Screen2ViewBase::setupScreen();
}

void Screen2View::tearDownScreen()
{
    Screen2ViewBase::tearDownScreen();
}

void Screen2View::Send_Strategy_Yellow()
{
    Apply_Strategy(YELLOW);
}
void Screen2View::Send_Strategy_Blue()
{
    Apply_Strategy(BLUE);
}
