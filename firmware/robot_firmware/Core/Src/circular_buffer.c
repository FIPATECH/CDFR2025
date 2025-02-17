#include "circular_buffer.h"


void CircularBuffer_Init(CircularBuffer *cb, uint8_t *buffer, uint16_t size)
{
    cb->buffer = buffer;
    cb->size = size;
    cb->head = 0;
    cb->tail = 0;
}

bool CircularBuffer_IsEmpty(CircularBuffer *cb)
{
    return (cb->head == cb->tail);
}

bool CircularBuffer_IsFull(CircularBuffer *cb)
{
    /* On réserve une case pour différencier le buffer plein du buffer vide */
    return (((cb->head + 1) % cb->size) == cb->tail);
}

uint16_t CircularBuffer_GetDataSize(CircularBuffer *cb)
{
    if (cb->head >= cb->tail)
        return cb->head - cb->tail;
    else
        return cb->size - (cb->tail - cb->head);
}

uint16_t CircularBuffer_GetRemainingSize(CircularBuffer *cb)
{
    /* On réserve une case pour la différenciation */
    return (cb->size - 1) - CircularBuffer_GetDataSize(cb);
}

bool CircularBuffer_Put(CircularBuffer *cb, uint8_t data)
{
    if (CircularBuffer_IsFull(cb))
    {
        return false; // Buffer plein, l'octet n'est pas ajouté.
    }
    cb->buffer[cb->head] = data;
    cb->head = (cb->head + 1) % cb->size;
    return true;
}

bool CircularBuffer_Get(CircularBuffer *cb, uint8_t *data)
{
    if (CircularBuffer_IsEmpty(cb))
    {
        return false; // Buffer vide, aucune donnée à récupérer.
    }
    *data = cb->buffer[cb->tail];
    cb->tail = (cb->tail + 1) % cb->size;
    return true;
}
