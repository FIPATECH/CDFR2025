#include "circular_buffer.h"

void circular_buffer_init(CircularBuffer *cb) {
  cb->head = 0;
  cb->tail = 0;
}

bool circular_buffer_push(CircularBuffer *cb, uint8_t data) {
  uint16_t next = (cb->head + 1) % CB_SIZE;
  if (next == cb->tail) return false;  // plein
  cb->buffer[cb->head] = data;
  cb->head = next;
  return true;
}

bool circular_buffer_pop(CircularBuffer *cb, uint8_t *data) {
  if (cb->head == cb->tail) return false;  // vide
  *data = cb->buffer[cb->tail];
  cb->tail = (cb->tail + 1) % CB_SIZE;
  return true;
}

uint16_t circular_buffer_available(CircularBuffer *cb) {
  int16_t diff = cb->head - cb->tail;
  if (diff < 0) diff += CB_SIZE;
  return (uint16_t)diff;
}
