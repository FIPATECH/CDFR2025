

#include "strategy_manager.h"
#include "uart_handler.h"
#include "uart_commands.h"
#include <stdio.h>
#include <stdint.h>

void Apply_Strategy(Color teamColor)
{
    Strategy strat;
    strat.team_color = teamColor;

    char msg[50];
    int length = snprintf(msg, sizeof(msg), "STRATEGY:%u:", (unsigned)strat.team_color);

    UART_Encode_And_Send_Message(UART_CMD_STRATEGY,
                                 (uint16_t)length,
                                 (const uint8_t *)msg);
}
