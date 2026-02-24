#ifndef STRATEGY_MANAGER_H
#define STRATEGY_MANAGER_H

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <queue.h>

enum Color {
    COLOR_RED,
    COLOR_BLUE
};

enum Zone {
    ZONE_A,
    ZONE_B
};

void Apply_Strategy(Color teamColor, Zone teamZone, Zone enemyZone);

#endif
