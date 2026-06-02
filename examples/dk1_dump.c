#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include "DK1Tracker/DK1Tracker.h"
#include <IOKit/hid/IOHIDDevice.h>
#include <IOKit/hid/IOHIDManager.h>
#include <IOKit/hid/IOHIDKeys.h>
#include <CoreFoundation/CoreFoundation.h>
#include <stdint.h>
#include <time.h>

static volatile int keep_running = 1;
// By default, **do not** dump raw reports.  Users can enable with
// `--raw`.  The `--no-raw` flag keeps the old behaviour (no raw
// dumping).  The logic that sets the flag was updated accordingly.
static int raw_enabled = 0;

// Helper to read little‑endian u16
static uint16_t read_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// Helper to read little‑endian i16
static int16_t read_i16_le(const uint8_t *p) {
    return (int16_t)p[0] | ((int16_t)p[1] << 8);
}

static void raw_print_cb(const uint8_t *data, size_t len, void *ud) {
    (void)ud;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    double host_ts = ts.tv_sec + ts.tv_nsec * 1e-9;
    if (len >= 62 && data[0] == 1) {
        uint16_t sample_count = data[1];
        uint16_t timestamp = read_u16_le(data + 2);
        uint16_t last_cmd = read_u16_le(data + 4);
        int16_t temp_raw = read_i16_le(data + 6);
        double temp_c = temp_raw / 100.0;
        int16_t magx = read_i16_le(data + 56);
        int16_t magy = read_i16_le(data + 58);
        int16_t magz = read_i16_le(data + 60);
        printf("raw[%zu] host=%.6f id=%u count=%u ts=%u last=%u temp=%.2fC mag=(%d,%d,%d): ",
               len, host_ts, data[0], sample_count, timestamp, last_cmd, temp_c, magx, magy, magz);
    } else {
        printf("raw[%zu] host=%.6f unrecognized: ", len, host_ts);
    }
    for (size_t i = 0; i < len; ++i) {
        printf("%02X", data[i]);
        if (i + 1 < len) printf(" ");
    }
    printf("\n");
}

static void handle_sigint(int sig) {
    keep_running = 0;
}

static void on_sample(const DK1Sample *sample, void *user_data) {
    printf("S: %u | Acc: %.2f %.2f %.2f | Gyro: %.2f %.2f %.2f | Mag: %.2f %.2f %.2f | Temp: %.2f C\n",
           sample->timestamp,
           sample->accel.x, sample->accel.y, sample->accel.z,
           sample->gyro.x, sample->gyro.y, sample->gyro.z,
           sample->mag.x, sample->mag.y, sample->mag.z,
           sample->temperature_c);
}

int main(int argc, char *argv[]) {
    signal(SIGINT, handle_sigint);
    DK1Tracker *tracker = NULL;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--raw") == 0) {
            raw_enabled = 1;
        } else if (strcmp(argv[i], "--no-raw") == 0) {
            raw_enabled = 0;
        }
    }
    printf("Creating tracker instance...\n");
    if (dk1_tracker_create(&tracker) != DK1_OK) {
        fprintf(stderr, "Failed to create tracker\n");
        return 1;
    }
    printf("Finding and opening backend device...\n");
    if (dk1_tracker_open(tracker) != DK1_OK) {
        fprintf(stderr, "Failed to open tracker\n");
        dk1_tracker_destroy(tracker);
        return 1;
    }
    printf("Device opened successfully.\n");
    dk1_tracker_set_keepalive(tracker, 10000);
    dk1_tracker_set_sample_callback(tracker, on_sample, NULL);
    if (raw_enabled) {
        dk1_tracker_set_raw_report_callback(tracker, raw_print_cb, NULL);
    }
    if (dk1_tracker_start(tracker) != DK1_OK) {
        fprintf(stderr, "Failed to start tracker\n");
        dk1_tracker_close(tracker);
        dk1_tracker_destroy(tracker);
        return 1;
    }
    printf("Tracking started. Press Ctrl-C to stop.\n");
    while (keep_running) {
        usleep(100000);
    }
    printf("\nStopping...\n");
    dk1_tracker_stop(tracker);
    dk1_tracker_close(tracker);
    dk1_tracker_destroy(tracker);
    return 0;
}
