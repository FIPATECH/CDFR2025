#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Taille du buffer circulaire pour la réception */
#define RX_BUFFER_SIZE 128

    /**
     * @brief Initialise la réception UART en interruption.
     */
    void UART_Init(void);

    /**
     * @brief Envoie une chaîne de caractères (mode bloquant).
     * @param str La chaîne à envoyer (terminée par '\0').
     */
    void uart_send_raw(const char *str);

    /**
     * @brief Envoie un buffer binaire (mode bloquant).
     * @param data Pointeur vers les données.
     * @param length Nombre d'octets à envoyer.
     */
    void uart_send_bytes(const uint8_t *data, uint16_t length);

    /**
     * @brief Traite les données reçues dans le tampon (à appeler périodiquement ou dans la callback).
     */
    void uart_process_data(void);

    /**
     * @brief Callback appelé par le HAL lorsqu'un octet est reçu.
     */
    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* UART_HANDLER_H */
