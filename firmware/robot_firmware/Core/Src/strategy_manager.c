#include "../Inc/strategy_manager.h"
#include "../Inc/uart_handler.h"
#include "../Inc/uart_commands.h"
#include <stdio.h>

void Apply_Strategy(Color teamColor, Zone teamZone, Zone enemyZone)
{
    Strategy strat;
    // strat.team_color = teamColor;
    // strat.team_zone = teamZone;
    // strat.ennemy_zone = enemyZone;

    // Tests
    strat.team_color = 0;
    strat.team_zone = 4;
    strat.ennemy_zone = 2;

    char msg[50];
    int length = sprintf(msg, "STRATEGY:%d:%d:%d", strat.team_color, strat.team_zone, strat.ennemy_zone);

    UART_Encode_And_Send_Message(UART_CMD_STRATEGY, (uint16_t)length, (const uint8_t *)msg);
}
