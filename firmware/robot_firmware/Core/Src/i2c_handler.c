// Core/Src/i2c_handler.c
#include "i2c_handler.h"
#include "match_trigger.h"   // pour extern volatile uint8_t startMatchFlag
#include "command_manager.h" // pour CommandManager_Process_Command()
#include "stm32f4xx_hal_i2c.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c3;
#define I2C_RX_BUF_SIZE ACTION_MSG_SIZE
static uint8_t i2cRxBuf[I2C_RX_BUF_SIZE];

/**
 * @brief  Initialise la partie I²C (bus-recovery éventuel) et démarre la réception esclave IT.
 */
void I2C_Handler_Init(void)
{
    // ← si besoin, votre routine de bus-recovery
    HAL_I2C_Slave_Receive_IT(&hi2c3, i2cRxBuf, I2C_RX_BUF_SIZE);
}

/**
 * @brief  Envoie une commande texte terminée par '\n' via I²C.
 */
HAL_StatusTypeDef I2C_SendCommand(const char *cmd)
{
    uint16_t addr = (I2C_SLAVE_ADDR << 1);
    return HAL_I2C_Master_Transmit(
        &hi2c3,
        addr,
        (uint8_t *)cmd,
        strlen(cmd),
        I2C_TIMEOUT);
}

/**
 * @brief Callback HAL appelé en fin de réception I2C esclave.
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C3)
        return;

    // Terminaison de la chaîne
    i2cRxBuf[I2C_RX_BUF_SIZE - 1] = '\0';

    // Si "start\n", lève le flag uniquement s’il est encore à 0
    if (strcmp((char *)i2cRxBuf, "start\n") == 0)
    {
        extern volatile uint8_t startMatchFlag;
        if (startMatchFlag == 0)
            startMatchFlag = 1;
    }
    else
    {
        // Sinon, passe au parser générique
        CommandManager_Process_Command((char *)i2cRxBuf);
    }

    // Réarme la réception IT
    HAL_I2C_Slave_Receive_IT(&hi2c3, i2cRxBuf, I2C_RX_BUF_SIZE);
}

/**
 * @brief Callback HAL sur erreur I2C esclave : on réarme la réception.
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C3)
    {
        HAL_I2C_Slave_Receive_IT(&hi2c3, i2cRxBuf, I2C_RX_BUF_SIZE);
    }
}
