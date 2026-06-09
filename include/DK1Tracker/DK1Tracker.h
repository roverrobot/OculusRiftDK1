#ifndef DK1_TRACKER_H
#define DK1_TRACKER_H

#include "DK1Types.h"
#include "DK1Error.h"
#include <stddef.h>

/**
 * Optional raw report callback.  The library will invoke this callback with
 * a *normalized* 62‑byte DK1 report (ReportID byte 0 included).  The
 * callback runs on the tracker worker thread, not the HID callback thread.
 * It should still return quickly so inference can keep up with incoming
 * reports.
 */
typedef void (*DK1RawReportCallback)(
    const uint8_t *data,
    size_t length,
    void *user_data
);

typedef struct DK1Tracker DK1Tracker;

int dk1_tracker_create(DK1Tracker **out_tracker);
void dk1_tracker_destroy(DK1Tracker *tracker);

int dk1_tracker_open(DK1Tracker *tracker);
void dk1_tracker_close(DK1Tracker *tracker);
int dk1_tracker_is_open(const DK1Tracker *tracker);

int dk1_tracker_start(DK1Tracker *tracker);
void dk1_tracker_stop(DK1Tracker *tracker);

/**
 * Register an optional parsed sample callback.
 * The callback runs on the tracker worker thread after the estimator has
 * consumed the sample and published the latest tracker state.
 */
void dk1_tracker_set_sample_callback(
    DK1Tracker *tracker,
    DK1SampleCallback callback,
    void *user_data
);

/**
 * Register a raw report callback.
 * The callback is called with the raw 62‑byte report before it is parsed.
 * If NULL is passed, no raw callback will be invoked.
 */
int dk1_tracker_set_raw_report_callback(
    DK1Tracker *tracker,
    DK1RawReportCallback callback,
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

/**
 * Copy the latest inferred tracker state published by the tracker worker.
 * This is a snapshot at the time of the call; it does not block waiting for
 * a new sample.
 */
int dk1_tracker_get_state(
    DK1Tracker *tracker,
    DK1TrackerState *out_state
);

int dk1_tracker_get_config(
    const DK1Tracker *tracker,
    DK1Config *out_config
);

int dk1_tracker_get_distortion_mesh(
    const DK1Tracker *tracker,
    DK1Eye eye,
    const DK1DistortionMesh **out_mesh
);

int dk1_tracker_set_gyro_bias(
    DK1Tracker *tracker,
    DK1Vector3 bias
);

int dk1_tracker_set_mag_calibration(
    DK1Tracker *tracker,
    const DK1MagCalibration *calibration
);

#endif // DK1_TRACKER_H
