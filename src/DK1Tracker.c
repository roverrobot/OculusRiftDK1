#include "DK1Tracker/DK1Tracker.h"
#include "DK1Tracker/DK1Error.h"
#include "DK1HIDBackend.h"
#include "DK1Parser.h"
#include "DK1Estimator.h"
#include "DK1RingBuffer.h"
#include "DK1Config.h"
#include "DK1Distortion.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

struct DK1Tracker {
    DK1HIDBackend backend;
    DK1Estimator estimator;
    DK1RingBuffer ring_buffer;
    DK1Config config;
    DK1DistortionMesh distortion_meshes[2];
    
    DK1SampleCallback user_callback;
    void *user_callback_data;

    DK1RawReportCallback raw_callback;
    void *raw_callback_data;
    
    int is_open;
    int is_started;
    // Keepalive command ID counter for feature reports.
    uint16_t keepalive_cmd_id;
};

static void internal_report_cb(const uint8_t *data, size_t length, void *user_data) {
    DK1Tracker *tracker = (DK1Tracker *)user_data;
    
    /* Forward raw report to user callback if registered */
    if (tracker->raw_callback) {
        tracker->raw_callback(data, length, tracker->raw_callback_data);
    }

    DK1Sample samples[3];
    size_t count = 0;
    if (dk1_parse_input_report(data, length, samples, 3, &count) == DK1_OK) {
        for (size_t i = 0; i < count; ++i) {
            dk1_ring_buffer_push(&tracker->ring_buffer, &samples[i]);
            dk1_estimator_update(&tracker->estimator, &samples[i]);
            if (tracker->user_callback) {
                tracker->user_callback(&samples[i], tracker->user_callback_data);
            }
        }
    }
}

int dk1_tracker_create(DK1Tracker **out_tracker) {
    if (!out_tracker) return DK1_ERROR_INVALID_ARGUMENT;

    DK1Tracker *tracker = calloc(1, sizeof(DK1Tracker));
    if (!tracker) return DK1_ERROR_IO;

    int config_result = dk1_config_load_default(&tracker->config);
    if (config_result != DK1_OK) {
        free(tracker);
        return config_result;
    }

    int left_mesh_result = dk1_distortion_mesh_build(
        &tracker->distortion_meshes[DK1_EYE_LEFT],
        DK1_EYE_LEFT,
        &tracker->config
    );
    if (left_mesh_result != DK1_OK) {
        free(tracker);
        return left_mesh_result;
    }

    int right_mesh_result = dk1_distortion_mesh_build(
        &tracker->distortion_meshes[DK1_EYE_RIGHT],
        DK1_EYE_RIGHT,
        &tracker->config
    );
    if (right_mesh_result != DK1_OK) {
        dk1_distortion_mesh_destroy(&tracker->distortion_meshes[DK1_EYE_LEFT]);
        free(tracker);
        return right_mesh_result;
    }
    
    dk1_hid_backend_create_mac(&tracker->backend);
    dk1_estimator_init(&tracker->estimator);
    dk1_ring_buffer_init(&tracker->ring_buffer);
    tracker->keepalive_cmd_id = 0;
    
    *out_tracker = tracker;
    return DK1_OK;
}

void dk1_tracker_destroy(DK1Tracker *tracker) {
    if (!tracker) return;
    dk1_tracker_close(tracker);
    dk1_distortion_mesh_destroy(&tracker->distortion_meshes[DK1_EYE_LEFT]);
    dk1_distortion_mesh_destroy(&tracker->distortion_meshes[DK1_EYE_RIGHT]);
    free(tracker);
}

int dk1_tracker_open(DK1Tracker *tracker) {
    if (!tracker) return DK1_ERROR_INVALID_ARGUMENT;
    int res = tracker->backend.open(&tracker->backend, 0x2833, 0x0001);
    if (res == DK1_OK) {
        tracker->is_open = 1;
        tracker->backend.set_raw_report_callback(&tracker->backend, internal_report_cb, tracker);
    }
    return res;
}

void dk1_tracker_close(DK1Tracker *tracker) {
    if (!tracker || !tracker->is_open) return;
    tracker->backend.close(&tracker->backend);
    tracker->is_open = 0;
    tracker->is_started = 0;
}

int dk1_tracker_is_open(const DK1Tracker *tracker) {
    return (tracker && tracker->is_open) ? DK1_OK : DK1_ERROR_NOT_OPEN;
}

int dk1_tracker_start(DK1Tracker *tracker) {
    if (!tracker || !tracker->is_open) return DK1_ERROR_NOT_OPEN;
    if (tracker->is_started) return DK1_OK;
    int res = tracker->backend.start(&tracker->backend);
    if (res == DK1_OK) tracker->is_started = 1;
    return res;
}

void dk1_tracker_stop(DK1Tracker *tracker) {
    if (!tracker || !tracker->is_started) return;
    tracker->backend.stop(&tracker->backend);
    tracker->is_started = 0;
}

void dk1_tracker_set_sample_callback(DK1Tracker *tracker, DK1SampleCallback callback, void *user_data) {
    if (!tracker) return;
    tracker->user_callback = callback;
    tracker->user_callback_data = user_data;
}

int dk1_tracker_set_raw_report_callback(DK1Tracker *tracker, DK1RawReportCallback callback, void *user_data) {
    if (!tracker) return DK1_ERROR_INVALID_ARGUMENT;
    tracker->raw_callback = callback;
    tracker->raw_callback_data = user_data;
    return DK1_OK;
}

int dk1_tracker_poll_sample(DK1Tracker *tracker, DK1Sample *out_sample) {
    if (!tracker || !out_sample) return DK1_ERROR_INVALID_ARGUMENT;
    return dk1_ring_buffer_pop(&tracker->ring_buffer, out_sample) ? DK1_OK : DK1_ERROR_IO;
}

int dk1_tracker_set_keepalive(DK1Tracker *tracker, uint16_t interval_ms) {
    if (!tracker || !tracker->is_open) return DK1_ERROR_NOT_OPEN;
    
    // Pack a 5‑byte feature report: ID=8, cmd id, interval.
    uint8_t report[5];
    report[0] = 8; // Report ID
    report[1] = (uint8_t)(tracker->keepalive_cmd_id & 0xFF); // Command ID low
    report[2] = (uint8_t)((tracker->keepalive_cmd_id >> 8) & 0xFF); // high
    report[3] = (uint8_t)(interval_ms & 0xFF); // interval low
    report[4] = (uint8_t)((interval_ms >> 8) & 0xFF); // high
    // Increment command ID for next call.
    tracker->keepalive_cmd_id++;
    
    return tracker->backend.set_feature_report(&tracker->backend, report, sizeof(report));
}

int dk1_tracker_get_orientation(DK1Tracker *tracker, DK1Quaternion *out_q) {
    if (!tracker || !out_q) return DK1_ERROR_INVALID_ARGUMENT;
    *out_q = tracker->estimator.orientation;
    return DK1_OK;
}

int dk1_tracker_get_state(DK1Tracker *tracker, DK1TrackerState *out_state) {
    if (!tracker || !out_state) return DK1_ERROR_INVALID_ARGUMENT;
    dk1_estimator_get_state(&tracker->estimator, out_state);
    return DK1_OK;
}

int dk1_tracker_get_config(const DK1Tracker *tracker, DK1Config *out_config) {
    if (!tracker || !out_config) return DK1_ERROR_INVALID_ARGUMENT;
    *out_config = tracker->config;
    return DK1_OK;
}

int dk1_tracker_get_distortion_mesh(
    const DK1Tracker *tracker,
    DK1Eye eye,
    const DK1DistortionMesh **out_mesh
) {
    if (!tracker || !out_mesh) return DK1_ERROR_INVALID_ARGUMENT;
    if (eye != DK1_EYE_LEFT && eye != DK1_EYE_RIGHT) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }
    *out_mesh = &tracker->distortion_meshes[eye];
    return DK1_OK;
}

int dk1_tracker_set_gyro_bias(DK1Tracker *tracker, DK1Vector3 bias) {
    if (!tracker) return DK1_ERROR_INVALID_ARGUMENT;
    dk1_estimator_set_gyro_bias(&tracker->estimator, bias);
    return DK1_OK;
}

int dk1_tracker_set_mag_calibration(
    DK1Tracker *tracker,
    const DK1MagCalibration *calibration
) {
    if (!tracker) return DK1_ERROR_INVALID_ARGUMENT;
    return dk1_estimator_set_mag_calibration(&tracker->estimator, calibration);
}
