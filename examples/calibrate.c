#include "DK1Tracker/DK1Tracker.h"
#include "DK1Config.h"

#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define DK1_PI 3.14159265358979323846

typedef struct CalibrationStats {
    pthread_mutex_t mutex;
    uint64_t sample_count;
    DK1Vector3 gyro_sum;
    DK1Vector3 accel_sum;
    double first_sample_time;
    double last_sample_time;
    int have_sample;
} CalibrationStats;

static volatile sig_atomic_t keep_running = 1;

static void handle_signal(int signo) {
    (void)signo;
    keep_running = 0;
}

static double monotonic_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static double vec3_norm(DK1Vector3 v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

static DK1Vector3 vec3_scale(DK1Vector3 v, double scale) {
    return (DK1Vector3){v.x * scale, v.y * scale, v.z * scale};
}

static void sample_callback(const DK1Sample *sample, void *user_data) {
    CalibrationStats *stats = (CalibrationStats *)user_data;
    if (!sample || !stats) return;

    pthread_mutex_lock(&stats->mutex);
    if (!stats->have_sample) {
        stats->first_sample_time = monotonic_seconds();
        stats->have_sample = 1;
    }
    stats->last_sample_time = monotonic_seconds();
    stats->sample_count++;
    stats->gyro_sum.x += sample->gyro.x;
    stats->gyro_sum.y += sample->gyro.y;
    stats->gyro_sum.z += sample->gyro.z;
    stats->accel_sum.x += sample->accel.x;
    stats->accel_sum.y += sample->accel.y;
    stats->accel_sum.z += sample->accel.z;
    pthread_mutex_unlock(&stats->mutex);
}

static int parse_positive_double(const char *text, double *out_value) {
    if (!text || !out_value) return 0;

    char *end = NULL;
    double value = strtod(text, &end);
    if (end == text || *end != '\0' || value <= 0.0 || !isfinite(value)) {
        return 0;
    }

    *out_value = value;
    return 1;
}

static void print_usage(FILE *stream, const char *program) {
    fprintf(stream, "Usage: %s [--seconds N] [--config PATH] [--dry-run]\n", program);
    fprintf(stream, "Collect stationary DK1 gyro samples and store the zero-rate bias.\n");
}

static int load_config_for_save(const char *path, DK1Config *out_config) {
    dk1_config_set_defaults(out_config);
    int result = path ?
        dk1_config_load_path(path, out_config) :
        dk1_config_load_default(out_config);
    if (result == DK1_ERROR_NOT_FOUND) {
        dk1_config_set_defaults(out_config);
        return DK1_OK;
    }
    return result;
}

int main(int argc, char **argv) {
    double seconds = 30.0;
    const char *config_path = NULL;
    int dry_run = 0;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--seconds") == 0) {
            if (i + 1 >= argc || !parse_positive_double(argv[++i], &seconds)) {
                fprintf(stderr, "Invalid or missing --seconds value\n");
                print_usage(stderr, argv[0]);
                return 1;
            }
        } else if (strncmp(argv[i], "--seconds=", 10) == 0) {
            if (!parse_positive_double(argv[i] + 10, &seconds)) {
                fprintf(stderr, "Invalid --seconds value\n");
                print_usage(stderr, argv[0]);
                return 1;
            }
        } else if (strcmp(argv[i], "--config") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing --config path\n");
                print_usage(stderr, argv[0]);
                return 1;
            }
            config_path = argv[++i];
        } else if (strncmp(argv[i], "--config=", 9) == 0) {
            config_path = argv[i] + 9;
        } else if (strcmp(argv[i], "--dry-run") == 0) {
            dry_run = 1;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(stdout, argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(stderr, argv[0]);
            return 1;
        }
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    CalibrationStats stats;
    memset(&stats, 0, sizeof(stats));
    if (pthread_mutex_init(&stats.mutex, NULL) != 0) {
        fprintf(stderr, "Failed to initialize calibration stats\n");
        return 1;
    }

    DK1Tracker *tracker = NULL;
    int result = dk1_tracker_create(&tracker);
    if (result != DK1_OK) {
        fprintf(stderr, "dk1_tracker_create failed: %d\n", result);
        pthread_mutex_destroy(&stats.mutex);
        return 1;
    }

    result = dk1_tracker_open(tracker);
    if (result != DK1_OK) {
        fprintf(stderr, "dk1_tracker_open failed: %d\n", result);
        dk1_tracker_destroy(tracker);
        pthread_mutex_destroy(&stats.mutex);
        return 1;
    }

    dk1_tracker_set_keepalive(tracker, 10000);
    dk1_tracker_set_sample_callback(tracker, sample_callback, &stats);

    result = dk1_tracker_start(tracker);
    if (result != DK1_OK) {
        fprintf(stderr, "dk1_tracker_start failed: %d\n", result);
        dk1_tracker_close(tracker);
        dk1_tracker_destroy(tracker);
        pthread_mutex_destroy(&stats.mutex);
        return 1;
    }

    fprintf(stderr, "Keep the HMD stationary for %.3f seconds...\n", seconds);
    double start_time = monotonic_seconds();
    while (keep_running && monotonic_seconds() - start_time < seconds) {
        usleep(10000);
    }

    dk1_tracker_stop(tracker);
    dk1_tracker_close(tracker);
    dk1_tracker_destroy(tracker);

    pthread_mutex_lock(&stats.mutex);
    uint64_t sample_count = stats.sample_count;
    DK1Vector3 gyro_sum = stats.gyro_sum;
    DK1Vector3 accel_sum = stats.accel_sum;
    double sample_span = stats.have_sample ?
        stats.last_sample_time - stats.first_sample_time :
        0.0;
    pthread_mutex_unlock(&stats.mutex);

    if (sample_count == 0) {
        fprintf(stderr, "No samples collected.\n");
        pthread_mutex_destroy(&stats.mutex);
        return 1;
    }

    DK1Vector3 gyro_bias = vec3_scale(gyro_sum, 1.0 / (double)sample_count);
    DK1Vector3 accel_mean = vec3_scale(accel_sum, 1.0 / (double)sample_count);
    double gyro_bias_mag = vec3_norm(gyro_bias);
    double accel_mag = vec3_norm(accel_mean);
    double drift_deg_x = gyro_bias.x * sample_span * 180.0 / DK1_PI;
    double drift_deg_y = gyro_bias.y * sample_span * 180.0 / DK1_PI;
    double drift_deg_z = gyro_bias.z * sample_span * 180.0 / DK1_PI;

    printf("samples: %llu\n", (unsigned long long)sample_count);
    printf("sample_span_s: %.6f\n", sample_span);
    printf(
        "gyro_bias_rad_s: %.9f %.9f %.9f magnitude=%.9f\n",
        gyro_bias.x,
        gyro_bias.y,
        gyro_bias.z,
        gyro_bias_mag
    );
    printf(
        "drift_over_span_deg: %.6f %.6f %.6f\n",
        drift_deg_x,
        drift_deg_y,
        drift_deg_z
    );
    printf(
        "accel_mean_m_s2: %.9f %.9f %.9f magnitude=%.9f\n",
        accel_mean.x,
        accel_mean.y,
        accel_mean.z,
        accel_mag
    );

    if (!dry_run) {
        DK1Config config;
        result = load_config_for_save(config_path, &config);
        if (result != DK1_OK) {
            fprintf(stderr, "Failed to load existing config: %d\n", result);
            pthread_mutex_destroy(&stats.mutex);
            return 1;
        }

        config.gyro_bias = gyro_bias;
        result = config_path ?
            dk1_config_save_path(config_path, &config) :
            dk1_config_save_default(&config);
        if (result != DK1_OK) {
            fprintf(stderr, "Failed to save config: %d\n", result);
            pthread_mutex_destroy(&stats.mutex);
            return 1;
        }

        char default_path[4096];
        if (config_path) {
            printf("saved_config: %s\n", config_path);
        } else if (dk1_config_default_path(default_path, sizeof(default_path)) == DK1_OK) {
            printf("saved_config: %s\n", default_path);
        }
    }

    pthread_mutex_destroy(&stats.mutex);
    return 0;
}
