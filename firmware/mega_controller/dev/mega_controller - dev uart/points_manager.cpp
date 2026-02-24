#include <Arduino.h>
#include "points_manager.h"
#include "uart_handler.h"
#include "uart_commands.h"

// Valeur courante des points pendant le match
static uint32_t current_points = 0;

/**
 * @brief Envoie le frame POINTS avec la valeur current_points
 */
static void sendPointsFrame(void)
{
    uint8_t payload[4] = {
        (uint8_t)(current_points >> 24),
        (uint8_t)(current_points >> 16),
        (uint8_t)(current_points >>  8),
        (uint8_t)(current_points)
    };
    // Envoi bas-niveau + log USB-Serial
    extern void sendFrameSerial2(uint16_t function, uint16_t length, const uint8_t *payload);
    sendFrameSerial2(UART_CMD_POINTS, 4, payload);
}

/**
 * Initialise le manager (rien de spécial ici, mais prêt pour l’avenir)
 */
void PointsManager_Init(void)
{
    current_points = 0;
    // Optionnel : envoyer le 0 initial
    PointsManager_Send();
}

/**
 * Remise à zéro en début de match
 */
void PointsManager_Reset(void)
{
    current_points = 0;
    Serial.println("PointsManager: reset to 0");
    sendPointsFrame();
}

/**
 * Ajoute pts au total et envoie immédiatement
 */
void PointsManager_Add(uint32_t pts)
{
    current_points += pts;
    Serial.print("PointsManager: add ");
    Serial.print(pts);
    Serial.print(" → total=");
    Serial.println(current_points);
    sendPointsFrame();
}

/**
 * Simple renvoi du total courant
 */
void PointsManager_Send(void)
{
    Serial.print("PointsManager: send current=");
    Serial.println(current_points);
    sendPointsFrame();
}
