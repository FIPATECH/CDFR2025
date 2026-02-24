#ifndef STRATEGY_MANAGER_H
#define STRATEGY_MANAGER_H

#include <stdint.h>
#include "uart_commands.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        BLUE = 0,
        YELLOW = 1
    } Color;

    typedef struct
    {
        Color team_color;
    } Strategy;

    /**
     * @brief Construit et envoie la trame STRATEGY via UART
     * @param teamColor BLUE ou YELLOW
     */
    void Apply_Strategy(Color teamColor);

#ifdef __cplusplus
}
#endif

#endif // STRATEGY_MANAGER_H
