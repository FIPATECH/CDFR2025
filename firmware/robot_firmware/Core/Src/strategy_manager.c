#include "../Inc/strategy_manager.h"
#include "../Inc/uart_handler.h"
#include "../Inc/uart_commands.h"
#include <stdio.h>

void Apply_Strategy(Color teamColor, Zone teamZone, Zone enemyZone)
{
    Strategy strat;
    strat.color = teamColor;
    strat.teamzone = teamZone;
    strat.ennemyzone = enemyZone;

    char msg[50];
    int length = sprintf(msg, "STRATEGY:%d:%d:%d", strat.color, strat.teamzone, strat.ennemyzone);

    UART_Encode_And_Send_Message(UART_CMD_STRATEGY, (uint16_t)length, (const uint8_t *)msg);
}
