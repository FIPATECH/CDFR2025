#include "../Inc/strategy_manager.h"
// #include "../Inc/uart_comm.h"
#include <stdio.h>

void applyStrategy(Color teamColor, Zone myZone, Zone enemyZone)
{
    Strategie strat;
    strat.couleur = teamColor;
    strat.myzone = myZone;
    strat.ennemyzone = enemyZone;

    char msg[50];
    sprintf(msg, "STRATEGY:%d:%d:%d", strat.couleur, strat.myzone, strat.ennemyzone);

    // uart_send(msg);
}
