#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint8_t *buffer; // Pointeur vers le tableau de stockage
    uint16_t head;   // Indice d'écriture
    uint16_t tail;   // Indice de lecture
    uint16_t size;   // Taille totale du buffer
} CircularBuffer;

/**
 * @brief Initialise un buffer circulaire.
 * @param buffer Tableau alloué pour le stockage.
 * @param size Taille du tableau.
 */
void CircularBuffer_Init(CircularBuffer *cb, uint8_t *buffer, uint16_t size);

/**
 * @brief Vérifie si le buffer est vide.
 * @return true si le buffer est vide, false sinon.
 */
bool CircularBuffer_IsEmpty(CircularBuffer *cb);

/**
 * @brief Vérifie si le buffer est plein.
 * @return true si le buffer est plein, false sinon.
 */
bool CircularBuffer_IsFull(CircularBuffer *cb);

/**
 * @brief Retourne le nombre d'octets actuellement stockés dans le buffer.
 * @return Nombre d'octets dans le buffer.
 */
uint16_t CircularBuffer_GetDataSize(CircularBuffer *cb);

/**
 * @brief Retourne l'espace libre restant dans le buffer.
 * @return Nombre d'octets pouvant encore être ajoutés.
 */
uint16_t CircularBuffer_GetRemainingSize(CircularBuffer *cb);

/**
 * @brief Ajoute un octet dans le buffer.
 * @param data Octet à ajouter.
 * @return true si l'ajout a réussi, false si le buffer est plein.
 */
bool CircularBuffer_Put(CircularBuffer *cb, uint8_t data);

/**
 * @brief Récupère un octet depuis le buffer.
 * @param data Pointeur où stocker l'octet récupéré.
 * @return true si la lecture a réussi, false si le buffer est vide.
 */
bool CircularBuffer_Get(CircularBuffer *cb, uint8_t *data);

#endif /* CIRCULAR_BUFFER_H */
