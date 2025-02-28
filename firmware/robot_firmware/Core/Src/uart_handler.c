#include "uart_handler.h"
#include "circular_buffer.h"
#include "uart_commands.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include "strategy_manager.h"

/* Utilisation de USART3 pour la communication */
extern UART_HandleTypeDef huart3;

/* Taille des buffers RX et TX */
#define RX_BUFFER_SIZE 512
#define TX_BUFFER_SIZE 512

/* Buffers statiques pour RX et TX */
static uint8_t rxArray[RX_BUFFER_SIZE];
static uint8_t txArray[TX_BUFFER_SIZE];

/* Instances de buffers circulaires */
static CircularBuffer rxCB;
static CircularBuffer txCB;

/* Flag indiquant si une transmission est en cours */
static volatile uint8_t isTransmitting = 0;

/* Octet temporaire pour la réception et pour la transmission */
static uint8_t rx_byte;
static uint8_t tx_byte;

/* États de réception */
#define RCV_STATE_WAITING 0
#define RCV_STATE_FUNCTION_MSB 1
#define RCV_STATE_FUNCTION_LSB 2
#define RCV_STATE_LENGTH_MSB 3
#define RCV_STATE_LENGTH_LSB 4
#define RCV_STATE_PAYLOAD 5
#define RCV_STATE_CHECKSUM 6

/* Variables pour le décodage */
static uint16_t msgDecodedFunction = 0;
static uint16_t msgDecodedPayloadLength = 0;
static uint8_t msgDecodedPayload[128];
static uint16_t msgDecodedPayloadIndex = 0;
static uint8_t rcvState = RCV_STATE_WAITING;

/**
 * @brief Calcule le checksum d'une trame.
 * Le calcul démarre avec la valeur 0x4A (en-tête) et effectue un XOR sur :
 * - les 2 octets de msgFunction (MSB puis LSB),
 * - les 2 octets de la longueur du payload (MSB puis LSB),
 * - puis sur chaque octet du payload.
 */
static uint8_t UART_Calculate_Checksum(uint16_t msgFunction, uint16_t msgPayloadLength, const uint8_t *msgPayload)
{
    uint8_t checksum = 0x4A;
    checksum ^= (uint8_t)(msgFunction >> 8);
    checksum ^= (uint8_t)msgFunction;
    checksum ^= (uint8_t)(msgPayloadLength >> 8);
    checksum ^= (uint8_t)msgPayloadLength;
    for (uint16_t i = 0; i < msgPayloadLength; i++)
    {
        checksum ^= msgPayload[i];
    }
    return checksum;
}

/**
 * @brief Encode une trame complète et l'envoie via UART.
 * La trame est composée de :
 *   - un octet d'en-tête (0x4A),
 *   - le msgFunction sur 2 octets (MSB puis LSB),
 *   - la longueur du payload sur 2 octets (MSB puis LSB),
 *   - le payload,
 *   - et le checksum (1 octet).
 */
void UART_Encode_And_Send_Message(uint16_t msgFunction, uint16_t msgPayloadLength, const uint8_t *msgPayload)
{
    uint16_t totalLength = 1 + 2 + 2 + msgPayloadLength + 1;
    uint8_t msg[totalLength];
    uint16_t pos = 0;

    msg[pos++] = 0x4A;
    msg[pos++] = (uint8_t)(msgFunction >> 8);
    msg[pos++] = (uint8_t)(msgFunction & 0xFF);
    msg[pos++] = (uint8_t)(msgPayloadLength >> 8);
    msg[pos++] = (uint8_t)(msgPayloadLength & 0xFF);

    if (msgPayloadLength > 0 && msgPayload != NULL)
    {
        memcpy(&msg[pos], msgPayload, msgPayloadLength);
        pos += msgPayloadLength;
    }
    msg[pos++] = UART_Calculate_Checksum(msgFunction, msgPayloadLength, msgPayload);

    UART_Send_Bytes(msg, totalLength);
}

/**
 * @brief Initialise l'UART et les buffers circulaires.
 */
void UART_Init(void)
{
    CircularBuffer_Init(&rxCB, rxArray, RX_BUFFER_SIZE);
    CircularBuffer_Init(&txCB, txArray, TX_BUFFER_SIZE);

    isTransmitting = 0;

    /* Démarrage de la réception d'un octet en mode interruption */
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

/**
 * @brief Envoie une chaîne de caractères non encodée via UART.
 */
void UART_Send_Raw(const char *str)
{
    if (str == NULL)
        return;
    UART_Send_Bytes((const uint8_t *)str, (uint16_t)strlen(str));
}

/**
 * @brief Envoie une chaîne de caractères encodée via UART.
 */
void UART_Send_Text(const char *text)
{
    if (text == NULL)
        return;
    uint16_t len = (uint16_t)strlen(text);
    UART_Encode_And_Send_Message(UART_CMD_TEXT, len, (const uint8_t *)text);
}

/**
 * @brief Envoie un buffer de données via UART en utilisant le buffer circulaire TX.
 */
void UART_Send_Bytes(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0)
        return;

    HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET); // LED Verte

    for (uint16_t i = 0; i < length; i++)
    {
        while (CircularBuffer_IsFull(&txCB))
        {
            // Timeout ?
        }
        CircularBuffer_Put(&txCB, data[i]);
    }

    if (!isTransmitting)
    {
        isTransmitting = 1;
        if (CircularBuffer_Get(&txCB, &tx_byte))
        {
            HAL_UART_Transmit_IT(&huart3, &tx_byte, 1);
        }
    }
}

/**
 * @brief Callback de réception (appelé par HAL quand un octet est reçu).
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        HAL_GPIO_TogglePin(LED_PORT, LED_RED_PIN); // LED Rouge

        CircularBuffer_Put(&rxCB, rx_byte);
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}

/**
 * @brief Callback de transmission (appelé par HAL quand la transmission d'un octet est terminée).
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (CircularBuffer_Get(&txCB, &tx_byte))
        {
            HAL_UART_Transmit_IT(&huart3, &tx_byte, 1);
        }
        else
        {
            isTransmitting = 0;
            HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
        }
    }
}

/**
 * @brief Traite les octets stockés dans le buffer RX et lance le décodage.
 */
void UART_Process_Data(void)
{
    uint8_t byte;
    while (!CircularBuffer_IsEmpty(&rxCB))
    {
        if (CircularBuffer_Get(&rxCB, &byte))
        {
            UART_Decode_Message(byte);
        }
    }
}

/**
 * @brief Fonction de décodage : reconstitue une trame à partir d'octets reçus.
 */
void UART_Decode_Message(uint8_t c)
{
    switch (rcvState)
    {
    case RCV_STATE_WAITING:
        if (c == 0x4A)
        {
            rcvState = RCV_STATE_FUNCTION_MSB;
            msgDecodedFunction = 0;
            msgDecodedPayloadLength = 0;
            msgDecodedPayloadIndex = 0;
        }
        break;

    case RCV_STATE_FUNCTION_MSB:
        msgDecodedFunction = c << 8;
        rcvState = RCV_STATE_FUNCTION_LSB;
        break;

    case RCV_STATE_FUNCTION_LSB:
        msgDecodedFunction |= c;
        rcvState = RCV_STATE_LENGTH_MSB;
        break;

    case RCV_STATE_LENGTH_MSB:
        msgDecodedPayloadLength = c << 8;
        rcvState = RCV_STATE_LENGTH_LSB;
        break;

    case RCV_STATE_LENGTH_LSB:
        msgDecodedPayloadLength |= c;
        // Si le payload est de longueur zéro, on passe directement à l'état CHECKSUM
        if (msgDecodedPayloadLength == 0)
        {
            rcvState = RCV_STATE_CHECKSUM;
        }
        else
        {
            rcvState = RCV_STATE_PAYLOAD;
        }
        break;

    case RCV_STATE_PAYLOAD:
        if (msgDecodedPayloadIndex < sizeof(msgDecodedPayload))
        {
            msgDecodedPayload[msgDecodedPayloadIndex++] = c;
        }
        if (msgDecodedPayloadIndex >= msgDecodedPayloadLength)
        {
            rcvState = RCV_STATE_CHECKSUM;
        }
        break;

    case RCV_STATE_CHECKSUM:
    {
        uint8_t calcChecksum = UART_Calculate_Checksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
        if (calcChecksum == c)
        {
            UART_Process_Decoded_Message(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
        }
        rcvState = RCV_STATE_WAITING;
    }
    break;

    default:
        rcvState = RCV_STATE_WAITING;
        break;
    }
}

/**
 * @brief Traite la trame décodée en fonction de son identifiant.
 */
void UART_Process_Decoded_Message(uint16_t function, uint16_t payloadLength, const uint8_t *payload)
{
    switch (function)
    {
    case UART_CMD_STRATEGY:
        Apply_Strategy(1, 1, 1);
        break;

    case UART_CMD_PING:
        UART_Encode_And_Send_Message(UART_CMD_PONG, 0, NULL);
        break;

    case UART_CMD_PONG:
        // Test de réception du ping
        // HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
        // HAL_Delay(1000);
        // HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
        break;

    case UART_CMD_TEXT:
        if (payloadLength > 0)
        {
            char text[128] = {0};
            memcpy(text, payload, (payloadLength < sizeof(text) - 1) ? payloadLength : sizeof(text) - 1);
            // Vérifier si le texte est "Hello STM!"
            if (strcmp(text, "Hello STM!") == 0)
            {
                HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
            }
        }
        break;

    default:
        break;
    }
}
