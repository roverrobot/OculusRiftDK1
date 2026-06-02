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
#include "../src/DK1Parser.h"

static volatile int keep_running = 1;
// By default, **do not** dump raw reports.  Users can enable with
// `--raw`.  The `--no-raw` flag keeps the old behaviour (no raw
// dumping).  The logic that sets the flag was updated accordingly.
static int raw_enabled = 0;
static int decode_blocks_enabled = 0;

// Helper to read little‑endian u16
static uint16_t read_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// Helper to read little‑endian i16
static int16_t read_i16_le(const uint8_t *p) {
    return (int16_t)p[0] | ((int16_t)p[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *p) {
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void print_block_bytes(const uint8_t *p) {
    for (size_t i = 0; i < 16; ++i) {
        printf("%02X", p[i]);
        if (i + 1 < 16) printf(" ");
    }
}

static void print_decode_blocks(const uint8_t *data, size_t len) {
    DK1Sample samples[3];
    size_t parsed_count = 0;
    int parse_status = dk1_parse_input_report(data, len, samples, 3, &parsed_count);

    if (len < 56) {
        printf("decode-blocks: report too short for 16-byte motion blocks (len=%zu)\n", len);
        return;
    }

    for (size_t block = 0; block < 3; ++block) {
        size_t offset = 8 + block * 16;
        const uint8_t *p = data + offset;
        printf("block%zu offset=%zu:\n", block, offset);
        printf("  bytes:  ");
        print_block_bytes(p);
        printf("\n");
        printf("  u32:    0x%08X 0x%08X 0x%08X 0x%08X\n",
               read_u32_le(p),
               read_u32_le(p + 4),
               read_u32_le(p + 8),
               read_u32_le(p + 12));
        if (parse_status == DK1_OK && block < parsed_count) {
            const DK1Sample *s = &samples[block];
            printf("  parsed: accel=(%.6f, %.6f, %.6f) gyro=(%.6f, %.6f, %.6f)\n",
                   s->accel.x, s->accel.y, s->accel.z,
                   s->gyro.x, s->gyro.y, s->gyro.z);
        } else if (parse_status == DK1_OK) {
            printf("  parsed: unavailable (parser returned %zu samples)\n", parsed_count);
        } else {
            printf("  parsed: unavailable (parse status %d)\n", parse_status);
        }
    }
}

static void raw_print_cb(const uint8_t *data, size_t len, void *ud) {
    (void)ud;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    double host_ts = ts.tv_sec + ts.tv_nsec * 1e-9;
    if (raw_enabled) {
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

    if (decode_blocks_enabled) {
        print_decode_blocks(data, len);
    }
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
        } else if (strcmp(argv[i], "--decode-blocks") == 0) {
            decode_blocks_enabled = 1;
        }
    }
    if (decode_blocks_enabled) {
        raw_enabled = 1;
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
    if (raw_enabled || decode_blocks_enabled) {
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
