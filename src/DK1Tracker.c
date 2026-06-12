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
#include <pthread.h>
#include <stdatomic.h>

#define DK1_TRACKER_REPORT_QUEUE_CAPACITY 8192
#define DK1_TRACKER_REPORT_MAX_LEN 64

typedef struct DK1QueuedReport {
    uint8_t data[DK1_TRACKER_REPORT_MAX_LEN];
    size_t length;
} DK1QueuedReport;

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

    DK1QueuedReport report_queue[DK1_TRACKER_REPORT_QUEUE_CAPACITY];
    size_t report_queue_head;
    size_t report_queue_tail;
    size_t report_queue_count;
    atomic_uint_fast64_t dropped_report_count;

    pthread_t worker_thread;
    int worker_thread_running;
    int worker_thread_started;
    pthread_mutex_t report_mutex;
    pthread_cond_t report_cond;
    pthread_mutex_t state_mutex;
    pthread_mutex_t callback_mutex;
    int sync_initialized;
    DK1TrackerState latest_state;
    
    int is_open;
    int is_started;
    // Keepalive command ID counter for feature reports.
    uint16_t keepalive_cmd_id;
};

static void tracker_destroy_sync(DK1Tracker *tracker) {
    if (!tracker || !tracker->sync_initialized) return;
    pthread_mutex_destroy(&tracker->callback_mutex);
    pthread_mutex_destroy(&tracker->state_mutex);
    pthread_cond_destroy(&tracker->report_cond);
    pthread_mutex_destroy(&tracker->report_mutex);
    tracker->sync_initialized = 0;
}

static int tracker_init_sync(DK1Tracker *tracker) {
    if (pthread_mutex_init(&tracker->report_mutex, NULL) != 0) {
        return DK1_ERROR_IO;
    }
    if (pthread_cond_init(&tracker->report_cond, NULL) != 0) {
        pthread_mutex_destroy(&tracker->report_mutex);
        return DK1_ERROR_IO;
    }
    if (pthread_mutex_init(&tracker->state_mutex, NULL) != 0) {
        pthread_cond_destroy(&tracker->report_cond);
        pthread_mutex_destroy(&tracker->report_mutex);
        return DK1_ERROR_IO;
    }
    if (pthread_mutex_init(&tracker->callback_mutex, NULL) != 0) {
        pthread_mutex_destroy(&tracker->state_mutex);
        pthread_cond_destroy(&tracker->report_cond);
        pthread_mutex_destroy(&tracker->report_mutex);
        return DK1_ERROR_IO;
    }
    tracker->sync_initialized = 1;
    return DK1_OK;
}

static void tracker_publish_state_locked(DK1Tracker *tracker) {
    dk1_estimator_get_state(&tracker->estimator, &tracker->latest_state);
    tracker->latest_state.report_queue_dropped_count =
        atomic_load_explicit(&tracker->dropped_report_count, memory_order_relaxed);
}

static void tracker_queue_report(DK1Tracker *tracker, const uint8_t *data, size_t length) {
    if (!tracker || !data) return;
    if (length > DK1_TRACKER_REPORT_MAX_LEN) {
        length = DK1_TRACKER_REPORT_MAX_LEN;
    }

    pthread_mutex_lock(&tracker->report_mutex);
    if (!tracker->worker_thread_running) {
        pthread_mutex_unlock(&tracker->report_mutex);
        return;
    }
    if (tracker->report_queue_count == DK1_TRACKER_REPORT_QUEUE_CAPACITY) {
        tracker->report_queue_head =
            (tracker->report_queue_head + 1) % DK1_TRACKER_REPORT_QUEUE_CAPACITY;
        tracker->report_queue_count--;
        atomic_fetch_add_explicit(
            &tracker->dropped_report_count,
            1,
            memory_order_relaxed
        );
    }

    DK1QueuedReport *entry = &tracker->report_queue[tracker->report_queue_tail];
    memcpy(entry->data, data, length);
    entry->length = length;
    tracker->report_queue_tail =
        (tracker->report_queue_tail + 1) % DK1_TRACKER_REPORT_QUEUE_CAPACITY;
    tracker->report_queue_count++;
    pthread_cond_signal(&tracker->report_cond);
    pthread_mutex_unlock(&tracker->report_mutex);
}

static int tracker_pop_report(DK1Tracker *tracker, DK1QueuedReport *out_report) {
    pthread_mutex_lock(&tracker->report_mutex);
    while (tracker->report_queue_count == 0 && tracker->worker_thread_running) {
        pthread_cond_wait(&tracker->report_cond, &tracker->report_mutex);
    }

    if (tracker->report_queue_count == 0 && !tracker->worker_thread_running) {
        pthread_mutex_unlock(&tracker->report_mutex);
        return 0;
    }

    *out_report = tracker->report_queue[tracker->report_queue_head];
    tracker->report_queue_head =
        (tracker->report_queue_head + 1) % DK1_TRACKER_REPORT_QUEUE_CAPACITY;
    tracker->report_queue_count--;
    pthread_mutex_unlock(&tracker->report_mutex);
    return 1;
}

static void tracker_process_report(DK1Tracker *tracker, const DK1QueuedReport *report) {
    DK1RawReportCallback raw_callback = NULL;
    void *raw_callback_data = NULL;
    pthread_mutex_lock(&tracker->callback_mutex);
    raw_callback = tracker->raw_callback;
    raw_callback_data = tracker->raw_callback_data;
    pthread_mutex_unlock(&tracker->callback_mutex);

    if (raw_callback) {
        raw_callback(report->data, report->length, raw_callback_data);
    }

    DK1Sample samples[3];
    size_t count = 0;
    if (dk1_parse_input_report(report->data, report->length, samples, 3, &count) != DK1_OK) {
        return;
    }

    for (size_t i = 0; i < count; ++i) {
        pthread_mutex_lock(&tracker->state_mutex);
        dk1_ring_buffer_push(&tracker->ring_buffer, &samples[i]);
        dk1_estimator_update(&tracker->estimator, &samples[i]);
        tracker_publish_state_locked(tracker);
        pthread_mutex_unlock(&tracker->state_mutex);

        DK1SampleCallback user_callback = NULL;
        void *user_callback_data = NULL;
        pthread_mutex_lock(&tracker->callback_mutex);
        user_callback = tracker->user_callback;
        user_callback_data = tracker->user_callback_data;
        pthread_mutex_unlock(&tracker->callback_mutex);
        if (user_callback) {
            user_callback(&samples[i], user_callback_data);
        }
    }
}

static void *tracker_worker_main(void *user_data) {
    DK1Tracker *tracker = (DK1Tracker *)user_data;
    DK1QueuedReport report;
    while (tracker_pop_report(tracker, &report)) {
        tracker_process_report(tracker, &report);
    }
    return NULL;
}

static void internal_report_cb(const uint8_t *data, size_t length, void *user_data) {
    DK1Tracker *tracker = (DK1Tracker *)user_data;
    tracker_queue_report(tracker, data, length);
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
    dk1_estimator_set_gyro_bias(&tracker->estimator, tracker->config.gyro_bias);
    int head_neck_result = dk1_estimator_set_head_neck_config(
        &tracker->estimator,
        &tracker->config.head_neck
    );
    if (head_neck_result != DK1_OK) {
        dk1_distortion_mesh_destroy(&tracker->distortion_meshes[DK1_EYE_LEFT]);
        dk1_distortion_mesh_destroy(&tracker->distortion_meshes[DK1_EYE_RIGHT]);
        free(tracker);
        return head_neck_result;
    }
    dk1_ring_buffer_init(&tracker->ring_buffer);
    atomic_init(&tracker->dropped_report_count, 0);
    int sync_result = tracker_init_sync(tracker);
    if (sync_result != DK1_OK) {
        dk1_distortion_mesh_destroy(&tracker->distortion_meshes[DK1_EYE_LEFT]);
        dk1_distortion_mesh_destroy(&tracker->distortion_meshes[DK1_EYE_RIGHT]);
        free(tracker);
        return sync_result;
    }
    pthread_mutex_lock(&tracker->state_mutex);
    tracker_publish_state_locked(tracker);
    pthread_mutex_unlock(&tracker->state_mutex);
    tracker->keepalive_cmd_id = 0;
    
    *out_tracker = tracker;
    return DK1_OK;
}

void dk1_tracker_destroy(DK1Tracker *tracker) {
    if (!tracker) return;
    dk1_tracker_close(tracker);
    dk1_distortion_mesh_destroy(&tracker->distortion_meshes[DK1_EYE_LEFT]);
    dk1_distortion_mesh_destroy(&tracker->distortion_meshes[DK1_EYE_RIGHT]);
    tracker_destroy_sync(tracker);
    free(tracker);
}

static uint8_t tracker_physical_sample_flags(uint8_t flags) {
    flags &= (uint8_t)~(DK1_CONFIG_USE_RAW | DK1_CONFIG_USE_CALIBRATION);
    flags |= DK1_CONFIG_AUTOCALIBRATION;
    return flags;
}

static int tracker_configure_physical_samples(DK1Tracker *tracker) {
    DK1TrackerConfiguration cfg;
    int result = dk1_tracker_get_configuration(tracker, &cfg);
    if (result != DK1_OK) return result;

    return dk1_tracker_set_configuration(
        tracker,
        tracker_physical_sample_flags(cfg.flags),
        cfg.packet_interval,
        cfg.sample_rate
    );
}

int dk1_tracker_open(DK1Tracker *tracker) {
    if (!tracker) return DK1_ERROR_INVALID_ARGUMENT;
    int res = tracker->backend.open(&tracker->backend, 0x2833, 0x0001);
    if (res == DK1_OK) {
        tracker->is_open = 1;

        int config_result = tracker_configure_physical_samples(tracker);
        if (config_result != DK1_OK) {
            tracker->backend.close(&tracker->backend);
            tracker->is_open = 0;
            return config_result;
        }

        tracker->backend.set_raw_report_callback(&tracker->backend, internal_report_cb, tracker);
    }
    return res;
}

void dk1_tracker_close(DK1Tracker *tracker) {
    if (!tracker || !tracker->is_open) return;
    dk1_tracker_stop(tracker);
    tracker->backend.close(&tracker->backend);
    tracker->is_open = 0;
}

int dk1_tracker_is_open(const DK1Tracker *tracker) {
    return (tracker && tracker->is_open) ? DK1_OK : DK1_ERROR_NOT_OPEN;
}

int dk1_tracker_start(DK1Tracker *tracker) {
    if (!tracker || !tracker->is_open) return DK1_ERROR_NOT_OPEN;
    if (tracker->is_started) return DK1_OK;

    pthread_mutex_lock(&tracker->report_mutex);
    tracker->report_queue_head = 0;
    tracker->report_queue_tail = 0;
    tracker->report_queue_count = 0;
    atomic_store_explicit(&tracker->dropped_report_count, 0, memory_order_relaxed);
    tracker->worker_thread_running = 1;
    pthread_mutex_unlock(&tracker->report_mutex);

    int thread_rc = pthread_create(
        &tracker->worker_thread,
        NULL,
        tracker_worker_main,
        tracker
    );
    if (thread_rc != 0) {
        pthread_mutex_lock(&tracker->report_mutex);
        tracker->worker_thread_running = 0;
        pthread_mutex_unlock(&tracker->report_mutex);
        return DK1_ERROR_IO;
    }
    tracker->worker_thread_started = 1;

    int res = tracker->backend.start(&tracker->backend);
    if (res != DK1_OK) {
        pthread_mutex_lock(&tracker->report_mutex);
        tracker->worker_thread_running = 0;
        pthread_cond_signal(&tracker->report_cond);
        pthread_mutex_unlock(&tracker->report_mutex);
        pthread_join(tracker->worker_thread, NULL);
        tracker->worker_thread_started = 0;
        return res;
    }

    tracker->is_started = 1;
    return res;
}

void dk1_tracker_stop(DK1Tracker *tracker) {
    if (!tracker || !tracker->is_started) return;
    tracker->backend.stop(&tracker->backend);

    pthread_mutex_lock(&tracker->report_mutex);
    tracker->worker_thread_running = 0;
    pthread_cond_signal(&tracker->report_cond);
    pthread_mutex_unlock(&tracker->report_mutex);

    if (tracker->worker_thread_started) {
        pthread_join(tracker->worker_thread, NULL);
        tracker->worker_thread_started = 0;
    }
    tracker->is_started = 0;
}

static uint16_t dk1_read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

int dk1_tracker_get_configuration(
    DK1Tracker *tracker,
    DK1TrackerConfiguration *out_config
) {
    if (!tracker || !out_config) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }

    if (!tracker->is_open) {
        return DK1_ERROR_NOT_OPEN;
    }

    if (!tracker->backend.get_feature_report) {
        return DK1_ERROR_UNSUPPORTED;
    }

    uint8_t report[7];
    int result = tracker->backend.get_feature_report(
        &tracker->backend,
        2,          // Configuration report ID
        report,
        sizeof(report)
    );

    if (result != DK1_OK) {
        return result;
    }

    if (report[0] != 2) {
        return DK1_ERROR_IO;
    }

    out_config->command_id = dk1_read_u16_le(report + 1);
    out_config->flags = report[3];
    out_config->packet_interval = report[4];
    out_config->sample_rate = dk1_read_u16_le(report + 5);

    return DK1_OK;
}

int dk1_tracker_set_configuration(
    DK1Tracker *tracker,
    uint8_t flags,
    uint8_t packet_interval,
    uint16_t sample_rate
) {
    if (!tracker) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }

    if (!tracker->is_open) {
        return DK1_ERROR_NOT_OPEN;
    }

    /*
     * Configuration Feature Report, Report ID 2.
     *
     * Layout:
     *   byte 0: ReportID = 2
     *   byte 1: CommandID low
     *   byte 2: CommandID high
     *   byte 3: flags
     *   byte 4: PacketInterval
     *   byte 5: SampleRate low
     *   byte 6: SampleRate high
     */
    uint16_t cmd_id = tracker->keepalive_cmd_id++;

    uint8_t report[7];
    report[0] = 2;
    report[1] = (uint8_t)(cmd_id & 0xff);
    report[2] = (uint8_t)((cmd_id >> 8) & 0xff);
    report[3] = flags;
    report[4] = packet_interval;
    report[5] = (uint8_t)(sample_rate & 0xff);
    report[6] = (uint8_t)((sample_rate >> 8) & 0xff);

    return tracker->backend.set_feature_report(
        &tracker->backend,
        report,
        sizeof(report)
    );
}

int dk1_tracker_configure_full_rate_no_keepalive(DK1Tracker *tracker)
{
    DK1TrackerConfiguration cfg;
    int result = dk1_tracker_get_configuration(tracker, &cfg);
    if (result != DK1_OK) return result;

    cfg.flags = tracker_physical_sample_flags(cfg.flags);
    cfg.flags &= (uint8_t)~DK1_CONFIG_USE_MOTION_KEEPALIVE;
    cfg.flags &= (uint8_t)~DK1_CONFIG_USE_COMMAND_KEEPALIVE;
    cfg.packet_interval = 0;
    if (cfg.sample_rate == 0) cfg.sample_rate = 1000;

    result = dk1_tracker_set_configuration(
        tracker,
        cfg.flags,
        cfg.packet_interval,
        cfg.sample_rate
    );
    if (result != DK1_OK) return result;

    DK1TrackerConfiguration after;
    return dk1_tracker_get_configuration(tracker, &after);
}

void dk1_tracker_set_sample_callback(DK1Tracker *tracker, DK1SampleCallback callback, void *user_data) {
    if (!tracker) return;
    pthread_mutex_lock(&tracker->callback_mutex);
    tracker->user_callback = callback;
    tracker->user_callback_data = user_data;
    pthread_mutex_unlock(&tracker->callback_mutex);
}

int dk1_tracker_set_raw_report_callback(DK1Tracker *tracker, DK1RawReportCallback callback, void *user_data) {
    if (!tracker) return DK1_ERROR_INVALID_ARGUMENT;
    pthread_mutex_lock(&tracker->callback_mutex);
    tracker->raw_callback = callback;
    tracker->raw_callback_data = user_data;
    pthread_mutex_unlock(&tracker->callback_mutex);
    return DK1_OK;
}

int dk1_tracker_poll_sample(DK1Tracker *tracker, DK1Sample *out_sample) {
    if (!tracker || !out_sample) return DK1_ERROR_INVALID_ARGUMENT;
    pthread_mutex_lock(&tracker->state_mutex);
    bool popped = dk1_ring_buffer_pop(&tracker->ring_buffer, out_sample);
    pthread_mutex_unlock(&tracker->state_mutex);
    return popped ? DK1_OK : DK1_ERROR_IO;
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
    pthread_mutex_lock(&tracker->state_mutex);
    tracker_publish_state_locked(tracker);
    *out_q = tracker->latest_state.orientation;
    pthread_mutex_unlock(&tracker->state_mutex);
    return DK1_OK;
}

int dk1_tracker_get_state(DK1Tracker *tracker, DK1TrackerState *out_state) {
    if (!tracker || !out_state) return DK1_ERROR_INVALID_ARGUMENT;
    pthread_mutex_lock(&tracker->state_mutex);
    tracker_publish_state_locked(tracker);
    *out_state = tracker->latest_state;
    pthread_mutex_unlock(&tracker->state_mutex);
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
    pthread_mutex_lock(&tracker->state_mutex);
    dk1_estimator_set_gyro_bias(&tracker->estimator, bias);
    tracker_publish_state_locked(tracker);
    pthread_mutex_unlock(&tracker->state_mutex);
    return DK1_OK;
}

int dk1_tracker_set_mag_calibration(
    DK1Tracker *tracker,
    const DK1MagCalibration *calibration
) {
    if (!tracker) return DK1_ERROR_INVALID_ARGUMENT;
    pthread_mutex_lock(&tracker->state_mutex);
    int result = dk1_estimator_set_mag_calibration(&tracker->estimator, calibration);
    tracker_publish_state_locked(tracker);
    pthread_mutex_unlock(&tracker->state_mutex);
    return result;
}

int dk1_tracker_set_head_neck_config(
    DK1Tracker *tracker,
    const DK1HeadNeckConfig *config
) {
    if (!tracker) return DK1_ERROR_INVALID_ARGUMENT;
    pthread_mutex_lock(&tracker->state_mutex);
    int result = dk1_estimator_set_head_neck_config(&tracker->estimator, config);
    tracker_publish_state_locked(tracker);
    pthread_mutex_unlock(&tracker->state_mutex);
    return result;
}

int dk1_tracker_get_head_neck_config(
    DK1Tracker *tracker,
    DK1HeadNeckConfig *out_config
) {
    if (!tracker || !out_config) return DK1_ERROR_INVALID_ARGUMENT;
    pthread_mutex_lock(&tracker->state_mutex);
    dk1_estimator_get_head_neck_config(&tracker->estimator, out_config);
    pthread_mutex_unlock(&tracker->state_mutex);
    return DK1_OK;
}
