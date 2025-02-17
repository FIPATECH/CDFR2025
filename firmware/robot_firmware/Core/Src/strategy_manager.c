#include "../Inc/strategy_manager.h"
#include "../Inc/uart_handler.h"
#include <stdio.h>

void applyStrategy(Color teamColor, Zone teamZone, Zone enemyZone)
{
    Strategy strat;
    strat.color = teamColor;
    strat.teamzone = teamZone;
    strat.ennemyzone = enemyZone;

    char msg[50];
    sprintf(msg, "STRATEGY:%d:%d:%d\n", strat.color, strat.teamzone, strat.ennemyzone);

    uart_send_raw(msg);
}
