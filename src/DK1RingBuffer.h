#ifndef DK1_RING_BUFFER_H
#define DK1_RING_BUFFER_H

#include <stddef.h>
#include "DK1Tracker/DK1Types.h"
#include <stdbool.h>

#define DK1_RING_BUFFER_SIZE 1024

typedef struct {
    DK1Sample buffer[DK1_RING_BUFFER_SIZE];
    size_t head;
    size_t tail;
    size_t count;
} DK1RingBuffer;

void dk1_ring_buffer_init(DK1RingBuffer *rb);
bool dk1_ring_buffer_push(DK1RingBuffer *rb, const DK1Sample *sample);
bool dk1_ring_buffer_pop(DK1RingBuffer *rb, DK1Sample *out_sample);
#endif
