
#ifndef _CAN_BUFFER_
#define _CAN_BUFFER_

#include <string.h>
#include <stdlib.h>

#define CB_SUCCESS 0        /* CB operation was successful */
#define CB_MEMORY_ERROR 1   /* Failed to allocate memory */
#define CB_OVERFLOW_ERROR 2 /* CB is full. Cannot push more items. */
#define CB_EMPTY_ERROR 3    /* CB is empty. Cannot pop more items. */

typedef struct circular_buffer {
  void *buffer;
  void *buffer_end;
  size_t sz;
  void *head;
  void *tail;
} circular_buffer;

int cb_init(circular_buffer *cb, size_t capacity, size_t sz) {
  const int incremented_capacity = capacity + 1; // Add extra element to evaluate count
  cb->buffer = malloc(incremented_capacity * sz);
  if (cb->buffer == NULL)
    return CB_MEMORY_ERROR;
  cb->buffer_end = (char *)cb->buffer + incremented_capacity * sz;
  cb->sz = sz;
  cb->head = cb->buffer;
  cb->tail = cb->buffer;
  return CB_SUCCESS;
}

int cb_free(circular_buffer *cb) {
  free(cb->buffer);
  return CB_SUCCESS;
}

const int _cb_length(circular_buffer *cb) {
  return (char *)cb->buffer_end - (char *)cb->buffer;
}

int cb_push_back(circular_buffer *cb, const void *item) {
  const int buffer_length = _cb_length(cb);
  const int capacity_length = buffer_length - cb->sz;

  if ((char *)cb->tail - (char *)cb->head == cb->sz ||
      (char *)cb->head - (char *)cb->tail == capacity_length)
    return CB_OVERFLOW_ERROR;

  memcpy(cb->head, item, cb->sz);

  cb->head = (char*)cb->head + cb->sz;
  if(cb->head == cb->buffer_end)
    cb->head = cb->buffer;

  return CB_SUCCESS;
}

int cb_pop_front(circular_buffer *cb, void *item) {
  if (cb->head == cb->tail)
    return CB_EMPTY_ERROR;

  memcpy(item, cb->tail, cb->sz);

  cb->tail = (char*)cb->tail + cb->sz;
  if(cb->tail == cb->buffer_end)
    cb->tail = cb->buffer;

  return CB_SUCCESS;
}

#endif