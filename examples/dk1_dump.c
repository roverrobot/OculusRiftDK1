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
#include <math.h>
#include "../src/DK1Parser.h"

static volatile int keep_running = 1;
// By default, **do not** dump raw reports.  Users can enable with
// `--raw`.  The `--no-raw` flag keeps the old behaviour (no raw
// dumping).  The logic that sets the flag was updated accordingly.
static int raw_enabled = 0;
static int decode_blocks_enabled = 0;
static int summary_enabled = 0;
static int sample_print_enabled = 1;

typedef struct RunningStats {
    uint64_t count;
    double mean;
    double m2;
} RunningStats;

typedef struct SummaryStats {
    RunningStats accel_mag;
    RunningStats gyro_mag;
    uint64_t report_count;
    uint64_t sample_count_hist[256];
    uint64_t timestamp_gap_hist[65536];
    uint16_t last_timestamp;
    int have_last_timestamp;
    double first_report_host_ts;
    double last_summary_host_ts;
} SummaryStats;

static SummaryStats summary_stats;

static double monotonic_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

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

static void stats_update(RunningStats *stats, double value) {
    stats->count++;
    double delta = value - stats->mean;
    stats->mean += delta / (double)stats->count;
    double delta2 = value - stats->mean;
    stats->m2 += delta * delta2;
}

static double stats_stddev(const RunningStats *stats) {
    if (stats->count < 2) return 0.0;
    return sqrt(stats->m2 / (double)(stats->count - 1));
}

static double vec3_mag(DK1Vector3 v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

static void print_histogram_u64(const uint64_t *hist, size_t len) {
    int printed = 0;
    for (size_t i = 0; i < len; ++i) {
        if (hist[i] == 0) continue;
        printf("%s%zu=%llu", printed ? " " : "", i, (unsigned long long)hist[i]);
        printed = 1;
    }
    if (!printed) {
        printf("(none)");
    }
}

static void print_summary(double host_ts) {
    double elapsed = host_ts - summary_stats.first_report_host_ts;
    double report_rate = 0.0;
    if (summary_stats.report_count > 1 && elapsed > 0.0) {
        report_rate = (double)(summary_stats.report_count - 1) / elapsed;
    }

    printf("summary reports=%llu rate=%.2fHz "
           "accel_mag(mean=%.4f std=%.4f n=%llu) "
           "gyro_mag(mean=%.6f std=%.6f n=%llu)\n",
           (unsigned long long)summary_stats.report_count,
           report_rate,
           summary_stats.accel_mag.mean,
           stats_stddev(&summary_stats.accel_mag),
           (unsigned long long)summary_stats.accel_mag.count,
           summary_stats.gyro_mag.mean,
           stats_stddev(&summary_stats.gyro_mag),
           (unsigned long long)summary_stats.gyro_mag.count);
    printf("  sample-count: ");
    print_histogram_u64(summary_stats.sample_count_hist, 256);
    printf("\n");
    printf("  timestamp-gap: ");
    print_histogram_u64(summary_stats.timestamp_gap_hist, 65536);
    printf("\n");
}

static void update_summary(const uint8_t *data, size_t len, double host_ts) {
    if (len < 62 || data[0] != 1) return;

    uint8_t sample_count = data[1];
    uint16_t timestamp = read_u16_le(data + 2);

    summary_stats.report_count++;
    summary_stats.sample_count_hist[sample_count]++;
    if (summary_stats.first_report_host_ts == 0.0) {
        summary_stats.first_report_host_ts = host_ts;
        summary_stats.last_summary_host_ts = host_ts;
    }

    if (summary_stats.have_last_timestamp) {
        uint16_t gap = (uint16_t)(timestamp - summary_stats.last_timestamp);
        summary_stats.timestamp_gap_hist[gap]++;
    }
    summary_stats.last_timestamp = timestamp;
    summary_stats.have_last_timestamp = 1;

    DK1Sample samples[3];
    size_t parsed_count = 0;
    if (dk1_parse_input_report(data, len, samples, 3, &parsed_count) == DK1_OK) {
        for (size_t i = 0; i < parsed_count; ++i) {
            stats_update(&summary_stats.accel_mag, vec3_mag(samples[i].accel));
            stats_update(&summary_stats.gyro_mag, vec3_mag(samples[i].gyro));
        }
    }

    if (host_ts - summary_stats.last_summary_host_ts >= 1.0) {
        print_summary(host_ts);
        summary_stats.last_summary_host_ts = host_ts;
    }
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
    double host_ts = monotonic_seconds();
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

    if (summary_enabled) {
        update_summary(data, len, host_ts);
    }
}

static void handle_sigint(int sig) {
    keep_running = 0;
}

static void on_sample(const DK1Sample *sample, void *user_data) {
    (void)user_data;
    if (!sample_print_enabled) return;
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
        } else if (strcmp(argv[i], "--summary") == 0) {
            summary_enabled = 1;
        }
    }
    if (decode_blocks_enabled) {
        raw_enabled = 1;
    }
    if (summary_enabled) {
        sample_print_enabled = 0;
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
    if (raw_enabled || decode_blocks_enabled || summary_enabled) {
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
    if (summary_enabled && summary_stats.report_count > 0) {
        print_summary(monotonic_seconds());
    }
    dk1_tracker_close(tracker);
    dk1_tracker_destroy(tracker);
    return 0;
}
