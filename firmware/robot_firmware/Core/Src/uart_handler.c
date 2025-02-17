#include "uart_handler.h"
#include <string.h>

/* Utilisation de USART3 pour la communication */
extern UART_HandleTypeDef huart3;

/* Tampon circulaire pour stocker les octets reçus */
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_write_index = 0;
/* Variable temporaire pour la réception d'un octet */
static uint8_t rx_byte;

/**
 * @brief Initialise la réception en interruption sur USART3.
 */
void UART_Init(void)
{
    /* Démarre la réception d'un octet en mode interruption */
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

/**
 * @brief Envoie une chaîne de caractères via USART3.
 */
void uart_send_raw(const char *str)
{
    if (str == NULL)
        return;
    HAL_UART_Transmit(&huart3, (uint8_t *)str, (uint16_t)strlen(str), 100);
}

/**
 * @brief Envoie un buffer de données via USART3.
 */
void uart_send_bytes(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0)
        return;
    HAL_UART_Transmit(&huart3, (uint8_t *)data, length, 100);
}

/**
 * @brief Callback de réception (appelé par HAL lorsqu'un octet est reçu).
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        /* Stocke l'octet reçu dans le buffer circulaire */
        rx_buffer[rx_write_index++] = rx_byte;
        if (rx_write_index >= RX_BUFFER_SIZE)
        {
            rx_write_index = 0;
        }
        /* Relance la réception pour l'octet suivant */
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}

/**
 * @brief Exemple de fonction de traitement du buffer.
 * Récupérer les octets jusqu'à détecter un délimiteur (ex: '\n'),
 * puis analyser la commande reçue.
 */
void uart_process_data(void)
{
    // TODO
}
