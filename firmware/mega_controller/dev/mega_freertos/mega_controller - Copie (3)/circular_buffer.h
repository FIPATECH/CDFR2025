#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <Arduino.h>

#define CB_SIZE 128

typedef struct {
  uint8_t buffer[CB_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
} CircularBuffer;

void circular_buffer_init(CircularBuffer *cb);
bool circular_buffer_push(CircularBuffer *cb, uint8_t data);
bool circular_buffer_pop(CircularBuffer *cb, uint8_t *data);
uint16_t circular_buffer_available(CircularBuffer *cb);

#endif  // CIRCULAR_BUFFER_H
