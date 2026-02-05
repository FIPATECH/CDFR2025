#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

    /* Initialisation */
    void UART_Init(void);

    /* Fonctions d'envoi */
    void UART_Send_Raw(const char *str);
    void UART_Send_Text(const char *text);
    void UART_Send_Bytes(const uint8_t *data, uint16_t length);
    void UART_Encode_And_Send_Message(uint16_t msgFunction, uint16_t msgPayloadLength, const uint8_t *msgPayload);

    /* Fonctions de décodage et de traitement des messages reçus */
    void UART_Process_Data(void);
    void UART_Decode_Message(uint8_t c);
    void UART_Process_Decoded_Message(uint16_t function, uint16_t payloadLength, const uint8_t *payload);

    /* Callbacks HAL pour la gestion des interruptions UART */
    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hUART);
    void HAL_UART_TxCpltCallback(UART_HandleTypeDef *hUART);

#ifdef __cplusplus
}
#endif

#endif /* UART_HANDLER_H */
