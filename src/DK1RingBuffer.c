#include "DK1RingBuffer.h"
#include <string.h>

void dk1_ring_buffer_init(DK1RingBuffer *rb) {
    memset(rb, 0, sizeof(DK1RingBuffer));
}

bool dk1_ring_buffer_push(DK1RingBuffer *rb, const DK1Sample *sample) {
    if (rb->count == DK1_RING_BUFFER_SIZE) return false;
    rb->buffer[rb->head] = *sample;
    rb->head = (rb->head + 1) % DK1_RING_BUFFER_SIZE;
    rb->count++;
    return true;
}

bool dk1_ring_buffer_pop(DK1RingBuffer *rb, DK1Sample *out_sample) {
    if (rb->count == 0) return false;
    *out_sample = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % DK1_RING_BUFFER_SIZE;
    rb->count--;
    return true;
}
