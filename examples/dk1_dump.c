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
static int csv_enabled = 0;
static int stationary_enabled = 0;
static int sample_print_enabled = 1;
static int use_time_limit = 0;
static const char *csv_path = NULL;
static FILE *text_output = NULL;
static FILE *csv_output = NULL;
static double run_seconds = 0.0;
static double stationary_seconds = 10.0;
static double program_start_host_ts = 0.0;

typedef struct RunningStats {
    uint64_t count;
    double mean;
    double m2;
} RunningStats;

typedef struct VectorMean {
    uint64_t count;
    double x;
    double y;
    double z;
} VectorMean;

typedef struct SummaryStats {
    uint64_t report_count;
    uint64_t parse_failure_count;
    uint64_t sample_count;
    uint64_t sample_count_hist[256];

    double first_host_ts;
    double last_host_ts;
    int have_host_ts;

    uint16_t first_device_timestamp;
    uint16_t last_device_timestamp;
    uint16_t previous_device_timestamp;
    int have_device_timestamp;
    uint64_t timestamp_gap_count;
    uint64_t timestamp_gap_sum;
    uint16_t timestamp_gap_min;
    uint16_t timestamp_gap_max;

    VectorMean accel;
    RunningStats accel_mag;
    double accel_mag_min;
    double accel_mag_max;
    int have_accel_mag;

    VectorMean gyro;
    RunningStats gyro_mag;
    double gyro_mag_min;
    double gyro_mag_max;
    int have_gyro_mag;

    VectorMean mag;
    double temp_sum;
    double temp_min;
    double temp_max;
    int have_temp;
} SummaryStats;

typedef struct StationaryStats {
    VectorMean accel;
    VectorMean gyro;
    uint64_t report_count;
    double start_host_ts;
    double last_host_ts;
    double temp_min;
    double temp_max;
    int have_temp;
} StationaryStats;

static SummaryStats summary_stats;
static StationaryStats stationary_stats;

static FILE *text_stream(void) {
    return text_output ? text_output : stdout;
}

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

static void vector_mean_update(VectorMean *mean, DK1Vector3 v) {
    mean->count++;
    double n = (double)mean->count;
    mean->x += (v.x - mean->x) / n;
    mean->y += (v.y - mean->y) / n;
    mean->z += (v.z - mean->z) / n;
}

static double vector_mean_mag(const VectorMean *mean) {
    return sqrt(mean->x * mean->x + mean->y * mean->y + mean->z * mean->z);
}

static void print_histogram_u64(const uint64_t *hist, size_t len) {
    FILE *out = text_stream();
    int printed = 0;
    for (size_t i = 0; i < len; ++i) {
        if (hist[i] == 0) continue;
        fprintf(out, "%s%zu=%llu", printed ? " " : "", i, (unsigned long long)hist[i]);
        printed = 1;
    }
    if (!printed) {
        fprintf(out, "(none)");
    }
}

static double relative_host_time(double host_ts) {
    if (program_start_host_ts <= 0.0) return 0.0;
    return host_ts - program_start_host_ts;
}

static void update_double_minmax(double value, double *min_value, double *max_value, int *have_value) {
    if (!*have_value) {
        *min_value = value;
        *max_value = value;
        *have_value = 1;
        return;
    }
    if (value < *min_value) *min_value = value;
    if (value > *max_value) *max_value = value;
}

static void update_summary_report(const uint8_t *data, size_t len) {
    if (len < 62 || data[0] != 1) return;
    summary_stats.report_count++;
    summary_stats.sample_count_hist[data[1]]++;

    DK1Sample samples[3];
    size_t parsed_count = 0;
    if (dk1_parse_input_report(data, len, samples, 3, &parsed_count) != DK1_OK) {
        summary_stats.parse_failure_count++;
    }
}

static void update_summary_sample(const DK1Sample *sample, double host_ts) {
    if (!sample) return;

    if (!summary_stats.have_host_ts) {
        summary_stats.first_host_ts = host_ts;
        summary_stats.have_host_ts = 1;
    }
    summary_stats.last_host_ts = host_ts;

    if (!summary_stats.have_device_timestamp) {
        summary_stats.first_device_timestamp = sample->timestamp;
        summary_stats.previous_device_timestamp = sample->timestamp;
        summary_stats.have_device_timestamp = 1;
    } else if (sample->timestamp != summary_stats.previous_device_timestamp) {
        uint16_t gap = (uint16_t)(sample->timestamp - summary_stats.previous_device_timestamp);
        summary_stats.timestamp_gap_sum += gap;
        if (summary_stats.timestamp_gap_count == 0 || gap < summary_stats.timestamp_gap_min) {
            summary_stats.timestamp_gap_min = gap;
        }
        if (summary_stats.timestamp_gap_count == 0 || gap > summary_stats.timestamp_gap_max) {
            summary_stats.timestamp_gap_max = gap;
        }
        summary_stats.timestamp_gap_count++;
        summary_stats.previous_device_timestamp = sample->timestamp;
    }
    summary_stats.last_device_timestamp = sample->timestamp;

    summary_stats.sample_count++;
    vector_mean_update(&summary_stats.accel, sample->accel);
    vector_mean_update(&summary_stats.gyro, sample->gyro);
    vector_mean_update(&summary_stats.mag, sample->mag);

    double accel_mag = vec3_mag(sample->accel);
    double gyro_mag = vec3_mag(sample->gyro);
    stats_update(&summary_stats.accel_mag, accel_mag);
    stats_update(&summary_stats.gyro_mag, gyro_mag);
    update_double_minmax(
        accel_mag,
        &summary_stats.accel_mag_min,
        &summary_stats.accel_mag_max,
        &summary_stats.have_accel_mag
    );
    update_double_minmax(
        gyro_mag,
        &summary_stats.gyro_mag_min,
        &summary_stats.gyro_mag_max,
        &summary_stats.have_gyro_mag
    );

    summary_stats.temp_sum += sample->temperature_c;
    update_double_minmax(
        sample->temperature_c,
        &summary_stats.temp_min,
        &summary_stats.temp_max,
        &summary_stats.have_temp
    );
}

static void print_summary(void) {
    FILE *out = text_stream();
    fprintf(out, "Summary:\n");
    fprintf(out, "  samples: %llu\n", (unsigned long long)summary_stats.sample_count);
    fprintf(out, "  reports: %llu\n", (unsigned long long)summary_stats.report_count);
    fprintf(out, "  parse_failures: %llu\n", (unsigned long long)summary_stats.parse_failure_count);

    if (summary_stats.sample_count == 0) {
        fprintf(out, "  no parsed samples collected\n");
        return;
    }

    double elapsed = summary_stats.last_host_ts - summary_stats.first_host_ts;
    double sample_rate = elapsed > 0.0 ? (double)summary_stats.sample_count / elapsed : 0.0;
    fprintf(out, "  elapsed_host_s: %.6f\n", elapsed);
    fprintf(out, "  host_time_first_s: %.6f\n", relative_host_time(summary_stats.first_host_ts));
    fprintf(out, "  host_time_last_s: %.6f\n", relative_host_time(summary_stats.last_host_ts));
    fprintf(out, "  sample_rate_hz: %.3f\n", sample_rate);
    fprintf(out, "  device_ts_first: %u\n", summary_stats.first_device_timestamp);
    fprintf(out, "  device_ts_last: %u\n", summary_stats.last_device_timestamp);

    fprintf(out, "  accel_mean: %.9f %.9f %.9f\n",
            summary_stats.accel.x,
            summary_stats.accel.y,
            summary_stats.accel.z);
    fprintf(out, "  accel_mag_mean: %.9f\n", summary_stats.accel_mag.mean);
    fprintf(out, "  accel_mag_std: %.9f\n", stats_stddev(&summary_stats.accel_mag));
    fprintf(out, "  accel_mag_min/max: %.9f %.9f\n",
            summary_stats.accel_mag_min,
            summary_stats.accel_mag_max);

    fprintf(out, "  gyro_mean: %.9f %.9f %.9f\n",
            summary_stats.gyro.x,
            summary_stats.gyro.y,
            summary_stats.gyro.z);
    fprintf(out, "  gyro_mag_mean: %.9f\n", summary_stats.gyro_mag.mean);
    fprintf(out, "  gyro_mag_std: %.9f\n", stats_stddev(&summary_stats.gyro_mag));
    fprintf(out, "  gyro_mag_min/max: %.9f %.9f\n",
            summary_stats.gyro_mag_min,
            summary_stats.gyro_mag_max);

    fprintf(out, "  mag_mean: %.9f %.9f %.9f\n",
            summary_stats.mag.x,
            summary_stats.mag.y,
            summary_stats.mag.z);
    fprintf(out, "  temp_mean/min/max: %.6f %.6f %.6f\n",
            summary_stats.temp_sum / (double)summary_stats.sample_count,
            summary_stats.temp_min,
            summary_stats.temp_max);

    if (summary_stats.timestamp_gap_count > 0) {
        fprintf(out, "  timestamp_gap_mean/min/max/count: %.3f %u %u %llu\n",
                (double)summary_stats.timestamp_gap_sum / (double)summary_stats.timestamp_gap_count,
                summary_stats.timestamp_gap_min,
                summary_stats.timestamp_gap_max,
                (unsigned long long)summary_stats.timestamp_gap_count);
    } else {
        fprintf(out, "  timestamp_gap_mean/min/max/count: unavailable\n");
    }
    fprintf(out, "  sample-count-field: ");
    print_histogram_u64(summary_stats.sample_count_hist, 256);
    fprintf(out, "\n");
}

static void print_stationary_estimate(void) {
    FILE *out = text_stream();
    double elapsed = 0.0;
    if (stationary_stats.start_host_ts > 0.0) {
        elapsed = stationary_stats.last_host_ts - stationary_stats.start_host_ts;
    }

    fprintf(out, "stationary seconds=%.3f elapsed=%.3f reports=%llu samples=%llu\n",
            stationary_seconds,
            elapsed,
            (unsigned long long)stationary_stats.report_count,
            (unsigned long long)stationary_stats.gyro.count);

    if (stationary_stats.gyro.count > 0) {
        fprintf(out,
                "  gyro zero-rate bias: (%.9f, %.9f, %.9f) magnitude=%.9f\n",
                stationary_stats.gyro.x,
                stationary_stats.gyro.y,
                stationary_stats.gyro.z,
                vector_mean_mag(&stationary_stats.gyro));
        fprintf(out,
                "  accel mean: (%.9f, %.9f, %.9f) magnitude=%.9f\n",
                stationary_stats.accel.x,
                stationary_stats.accel.y,
                stationary_stats.accel.z,
                vector_mean_mag(&stationary_stats.accel));
    } else {
        fprintf(out, "  gyro zero-rate bias: unavailable\n");
        fprintf(out, "  accel mean: unavailable\n");
    }

    if (stationary_stats.have_temp) {
        fprintf(out, "  temperature range: %.2f..%.2f C\n",
                stationary_stats.temp_min,
                stationary_stats.temp_max);
    } else {
        fprintf(out, "  temperature range: unavailable\n");
    }
}

static void update_stationary(const uint8_t *data, size_t len, double host_ts) {
    if (len < 62 || data[0] != 1) return;

    if (stationary_stats.report_count == 0) {
        stationary_stats.start_host_ts = host_ts;
    }
    stationary_stats.last_host_ts = host_ts;
    stationary_stats.report_count++;

    DK1Sample samples[3];
    size_t parsed_count = 0;
    if (dk1_parse_input_report(data, len, samples, 3, &parsed_count) == DK1_OK) {
        for (size_t i = 0; i < parsed_count; ++i) {
            const DK1Sample *s = &samples[i];
            vector_mean_update(&stationary_stats.accel, s->accel);
            vector_mean_update(&stationary_stats.gyro, s->gyro);
            if (!stationary_stats.have_temp) {
                stationary_stats.temp_min = s->temperature_c;
                stationary_stats.temp_max = s->temperature_c;
                stationary_stats.have_temp = 1;
            } else {
                if (s->temperature_c < stationary_stats.temp_min) {
                    stationary_stats.temp_min = s->temperature_c;
                }
                if (s->temperature_c > stationary_stats.temp_max) {
                    stationary_stats.temp_max = s->temperature_c;
                }
            }
        }
    }

    if (host_ts - stationary_stats.start_host_ts >= stationary_seconds) {
        keep_running = 0;
    }
}

static void print_block_bytes(const uint8_t *p) {
    FILE *out = text_stream();
    for (size_t i = 0; i < 16; ++i) {
        fprintf(out, "%02X", p[i]);
        if (i + 1 < 16) fprintf(out, " ");
    }
}

static void print_decode_blocks(const uint8_t *data, size_t len) {
    FILE *out = text_stream();
    DK1Sample samples[3];
    size_t parsed_count = 0;
    int parse_status = dk1_parse_input_report(data, len, samples, 3, &parsed_count);

    if (len < 56) {
        fprintf(out, "decode-blocks: report too short for 16-byte motion blocks (len=%zu)\n", len);
        return;
    }

    for (size_t block = 0; block < 3; ++block) {
        size_t offset = 8 + block * 16;
        const uint8_t *p = data + offset;
        fprintf(out, "block%zu offset=%zu:\n", block, offset);
        fprintf(out, "  bytes:  ");
        print_block_bytes(p);
        fprintf(out, "\n");
        fprintf(out, "  u32:    0x%08X 0x%08X 0x%08X 0x%08X\n",
                read_u32_le(p),
                read_u32_le(p + 4),
                read_u32_le(p + 8),
                read_u32_le(p + 12));
        if (parse_status == DK1_OK && block < parsed_count) {
            const DK1Sample *s = &samples[block];
            fprintf(out, "  parsed: accel=(%.6f, %.6f, %.6f) gyro=(%.6f, %.6f, %.6f)\n",
                    s->accel.x, s->accel.y, s->accel.z,
                    s->gyro.x, s->gyro.y, s->gyro.z);
        } else if (parse_status == DK1_OK) {
            fprintf(out, "  parsed: unavailable (parser returned %zu samples)\n", parsed_count);
        } else {
            fprintf(out, "  parsed: unavailable (parse status %d)\n", parse_status);
        }
    }
}

static void write_csv_report(const uint8_t *data, size_t len, double host_ts) {
    if (!csv_output) return;

    DK1Sample samples[3];
    size_t parsed_count = 0;
    if (dk1_parse_input_report(data, len, samples, 3, &parsed_count) != DK1_OK) {
        return;
    }

    for (size_t i = 0; i < parsed_count; ++i) {
        const DK1Sample *s = &samples[i];
        fprintf(csv_output,
                "%.9f,%u,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                relative_host_time(host_ts),
                s->timestamp,
                s->accel.x, s->accel.y, s->accel.z,
                s->gyro.x, s->gyro.y, s->gyro.z,
                s->mag.x, s->mag.y, s->mag.z,
                s->temperature_c);
    }
}

static void raw_print_cb(const uint8_t *data, size_t len, void *ud) {
    (void)ud;
    double host_ts = monotonic_seconds();
    FILE *out = text_stream();
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
            fprintf(out, "raw[%zu] host=%.6f id=%u count=%u ts=%u last=%u temp=%.2fC mag=(%d,%d,%d): ",
                    len, host_ts, data[0], sample_count, timestamp, last_cmd, temp_c, magx, magy, magz);
        } else {
            fprintf(out, "raw[%zu] host=%.6f unrecognized: ", len, host_ts);
        }
        for (size_t i = 0; i < len; ++i) {
            fprintf(out, "%02X", data[i]);
            if (i + 1 < len) fprintf(out, " ");
        }
        fprintf(out, "\n");
    }

    if (decode_blocks_enabled) {
        print_decode_blocks(data, len);
    }

    if (summary_enabled) {
        update_summary_report(data, len);
    }

    if (csv_enabled) {
        write_csv_report(data, len, host_ts);
    }

    if (stationary_enabled) {
        update_stationary(data, len, host_ts);
    }
}

static void handle_sigint(int sig) {
    keep_running = 0;
}

static void on_sample(const DK1Sample *sample, void *user_data) {
    (void)user_data;
    if (summary_enabled) {
        update_summary_sample(sample, monotonic_seconds());
    }
    if (!sample_print_enabled) return;
    fprintf(text_stream(), "S: %u | Acc: %.2f %.2f %.2f | Gyro: %.2f %.2f %.2f | Mag: %.2f %.2f %.2f | Temp: %.2f C\n",
            sample->timestamp,
            sample->accel.x, sample->accel.y, sample->accel.z,
            sample->gyro.x, sample->gyro.y, sample->gyro.z,
            sample->mag.x, sample->mag.y, sample->mag.z,
            sample->temperature_c);
}

static int parse_positive_double(const char *text, double *out_value) {
    if (!text) return 0;
    char *end = NULL;
    double value = strtod(text, &end);
    if (text == end || *end != '\0' || value <= 0.0) {
        return 0;
    }
    *out_value = value;
    return 1;
}

static void print_usage(FILE *out, const char *argv0) {
    fprintf(out, "Usage: %s [options]\n", argv0);
    fprintf(out, "Options:\n");
    fprintf(out, "  --raw                 Print raw normalized HID reports.\n");
    fprintf(out, "  --no-raw              Disable raw HID report printing.\n");
    fprintf(out, "  --decode-blocks       Print raw 16-byte motion block diagnostics.\n");
    fprintf(out, "  --summary             Print end-of-run sample summary statistics.\n");
    fprintf(out, "  --csv [PATH|-]        Write CSV samples to stdout, '-' or PATH.\n");
    fprintf(out, "  --csv=PATH            Write CSV samples to PATH.\n");
    fprintf(out, "  --stationary [SEC]    Estimate stationary bias/means, default 10 sec.\n");
    fprintf(out, "  --stationary=SEC      Estimate stationary bias/means for SEC seconds.\n");
    fprintf(out, "  --time SECONDS        Run for a fixed positive duration.\n");
    fprintf(out, "  --time=SECONDS        Run for a fixed positive duration.\n");
    fprintf(out, "  --help                Show this help.\n");
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
        } else if (strcmp(argv[i], "--csv") == 0) {
            csv_enabled = 1;
            if (i + 1 < argc &&
                (argv[i + 1][0] != '-' || strcmp(argv[i + 1], "-") == 0)) {
                csv_path = argv[++i];
            }
        } else if (strncmp(argv[i], "--csv=", 6) == 0) {
            csv_enabled = 1;
            csv_path = argv[i] + 6;
        } else if (strcmp(argv[i], "--stationary") == 0) {
            stationary_enabled = 1;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                if (!parse_positive_double(argv[++i], &stationary_seconds)) {
                    fprintf(stderr, "Invalid --stationary seconds value\n");
                    print_usage(stderr, argv[0]);
                    return 1;
                }
            }
        } else if (strncmp(argv[i], "--stationary=", 13) == 0) {
            stationary_enabled = 1;
            if (!parse_positive_double(argv[i] + 13, &stationary_seconds)) {
                fprintf(stderr, "Invalid --stationary seconds value\n");
                print_usage(stderr, argv[0]);
                return 1;
            }
        } else if (strcmp(argv[i], "--time") == 0) {
            if (i + 1 >= argc || !parse_positive_double(argv[++i], &run_seconds)) {
                fprintf(stderr, "Invalid or missing --time seconds value\n");
                print_usage(stderr, argv[0]);
                return 1;
            }
            use_time_limit = 1;
        } else if (strncmp(argv[i], "--time=", 7) == 0) {
            if (!parse_positive_double(argv[i] + 7, &run_seconds)) {
                fprintf(stderr, "Invalid --time seconds value\n");
                print_usage(stderr, argv[0]);
                return 1;
            }
            use_time_limit = 1;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(stdout, argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(stderr, argv[0]);
            return 1;
        }
    }
    if (decode_blocks_enabled) {
        raw_enabled = 1;
    }
    if (summary_enabled || csv_enabled || stationary_enabled) {
        sample_print_enabled = 0;
    }

    if (csv_enabled) {
        if (csv_path && strcmp(csv_path, "-") != 0) {
            csv_output = fopen(csv_path, "w");
            if (!csv_output) {
                fprintf(stderr, "Failed to open CSV output: %s\n", csv_path);
                return 1;
            }
            text_output = stdout;
        } else {
            csv_output = stdout;
            text_output = stderr;
        }
        if (raw_enabled) {
            fprintf(stderr, "Warning: --raw with --csv sends raw/status output to stderr.\n");
            text_output = stderr;
        }
        fprintf(csv_output,
                "host_time,device_timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,temp_c\n");
    } else {
        text_output = stdout;
    }

    fprintf(text_stream(), "Creating tracker instance...\n");
    if (dk1_tracker_create(&tracker) != DK1_OK) {
        fprintf(stderr, "Failed to create tracker\n");
        if (csv_output && csv_output != stdout) fclose(csv_output);
        return 1;
    }
    fprintf(text_stream(), "Finding and opening backend device...\n");
    if (dk1_tracker_open(tracker) != DK1_OK) {
        fprintf(stderr, "Failed to open tracker\n");
        dk1_tracker_destroy(tracker);
        if (csv_output && csv_output != stdout) fclose(csv_output);
        return 1;
    }
    fprintf(text_stream(), "Device opened successfully.\n");
    dk1_tracker_set_keepalive(tracker, 10000);
    dk1_tracker_set_sample_callback(tracker, on_sample, NULL);
    if (raw_enabled || decode_blocks_enabled || summary_enabled || csv_enabled || stationary_enabled) {
        dk1_tracker_set_raw_report_callback(tracker, raw_print_cb, NULL);
    }
    program_start_host_ts = monotonic_seconds();
    if (dk1_tracker_start(tracker) != DK1_OK) {
        fprintf(stderr, "Failed to start tracker\n");
        dk1_tracker_close(tracker);
        dk1_tracker_destroy(tracker);
        if (csv_output && csv_output != stdout) fclose(csv_output);
        return 1;
    }
    if (stationary_enabled) {
        fprintf(text_stream(), "Stationary collection started for %.3f seconds.\n", stationary_seconds);
    } else {
        fprintf(text_stream(), "Tracking started. Press Ctrl-C to stop.\n");
    }
    while (keep_running) {
        if (use_time_limit && monotonic_seconds() - program_start_host_ts >= run_seconds) {
            break;
        }
        usleep(10000);
    }
    fprintf(text_stream(), "\nStopping...\n");
    dk1_tracker_stop(tracker);
    if (summary_enabled) {
        print_summary();
    }
    if (stationary_enabled) {
        print_stationary_estimate();
    }
    dk1_tracker_close(tracker);
    dk1_tracker_destroy(tracker);
    if (csv_output && csv_output != stdout) {
        fclose(csv_output);
    }
    return 0;
}
