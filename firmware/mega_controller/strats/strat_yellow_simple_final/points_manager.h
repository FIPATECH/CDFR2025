#ifndef POINTS_MANAGER_H
#define POINTS_MANAGER_H

#include <stdint.h>

/** Initialise le manager (à appeler dans setup) */
void PointsManager_Init(void);

/** Remet les points à zéro (début de match) */
void PointsManager_Reset(void);

/** Ajoute pts au total et envoie le nouveau total via UART */
void PointsManager_Add(uint32_t pts);

/** Envoie le total courant via UART sans le modifier */
void PointsManager_Send(void);

#endif // POINTS_MANAGER_H
