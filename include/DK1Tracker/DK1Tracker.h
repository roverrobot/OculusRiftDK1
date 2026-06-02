#ifndef DK1_TRACKER_H
#define DK1_TRACKER_H

#include "DK1Types.h"
#include "DK1Error.h"

typedef struct DK1Tracker DK1Tracker;

int dk1_tracker_create(DK1Tracker **out_tracker);
void dk1_tracker_destroy(DK1Tracker *tracker);

int dk1_tracker_open(DK1Tracker *tracker);
void dk1_tracker_close(DK1Tracker *tracker);
int dk1_tracker_is_open(const DK1Tracker *tracker);

int dk1_tracker_start(DK1Tracker *tracker);
void dk1_tracker_stop(DK1Tracker *tracker);

void dk1_tracker_set_sample_callback(
    DK1Tracker *tracker,
    DK1SampleCallback callback,
    void *user_data
);

int dk1_tracker_poll_sample(
    DK1Tracker *tracker,
    DK1Sample *out_sample
);

int dk1_tracker_set_keepalive(
    DK1Tracker *tracker,
    uint16_t interval_ms
);

int dk1_tracker_get_orientation(
    DK1Tracker *tracker,
    DK1Quaternion *out_q
);

#endif // DK1_TRACKER_H
