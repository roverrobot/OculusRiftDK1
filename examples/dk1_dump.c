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

static volatile int keep_running = 1;
// By default, dump raw reports.  Users can disable with --no-raw.
static int raw_enabled = 1;

static void raw_print_cb(const uint8_t *data, size_t len, void *ud) {
    (void)ud;
    printf("raw[%zu]: ", len);
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
