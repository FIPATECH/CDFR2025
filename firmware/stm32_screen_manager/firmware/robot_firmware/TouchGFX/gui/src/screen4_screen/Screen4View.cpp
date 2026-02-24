#include <cstdio>
#include <touchgfx/Unicode.hpp>
#include "gui/screen4_screen/Screen4View.hpp"
#include "uart_handler.h"

Screen4View::Screen4View() {}

void Screen4View::setupScreen()
{
    Screen4ViewBase::setupScreen();
    textPoints.setWildcard(textPointsBuffer);

    Unicode::snprintf(textPointsBuffer, TEXTPOINTS_SIZE, "%02u", 0);
    textPoints.resizeToCurrentText();
    textPoints.invalidate();
}

void Screen4View::tearDownScreen()
{
    Screen4ViewBase::tearDownScreen();
}

void Screen4View::updatePoints(uint32_t pts)
{
    pendingPts = pts;
    dirty = true; // on demande un refresh au prochain tick
}

void Screen4View::handleTickEvent()
{
    Screen4ViewBase::handleTickEvent(); // conserve l’animation TouchGFX

    if (dirty)
    {
        dirty = false;

        Unicode::snprintf(textPointsBuffer, TEXTPOINTS_SIZE, "%02u", pendingPts);
        textPoints.resizeToCurrentText();
        textPoints.invalidate(); // exécuté sur le thread UI → sûr
    }
}
