// examples/dk1_state_60hz.c

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#include "DK1Tracker/DK1Tracker.h"

static volatile sig_atomic_t keep_running = 1;

static void handle_sigint(int sig)
{
    (void)sig;
    keep_running = 0;
}

static double monotonic_seconds(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void sleep_until(double target_time_s)
{
    for (;;) {
        double now = monotonic_seconds();
        double remaining = target_time_s - now;
        if (remaining <= 0.0) {
            return;
        }

        struct timespec req;
        req.tv_sec = (time_t)remaining;
        req.tv_nsec = (long)((remaining - (double)req.tv_sec) * 1e9);
        nanosleep(&req, NULL);
    }
}

static double quat_norm(DK1Quaternion q)
{
    return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    signal(SIGINT, handle_sigint);

    DK1Tracker *tracker = NULL;

    int result = dk1_tracker_create(&tracker);
    if (result != DK1_OK) {
        fprintf(stderr, "dk1_tracker_create failed: %d\n", result);
        return 1;
    }

    result = dk1_tracker_open(tracker);
    if (result != DK1_OK) {
        fprintf(stderr, "dk1_tracker_open failed: %d\n", result);
        dk1_tracker_destroy(tracker);
        return 1;
    }

    result = dk1_tracker_start(tracker);
    if (result != DK1_OK) {
        fprintf(stderr, "dk1_tracker_start failed: %d\n", result);
        dk1_tracker_close(tracker);
        dk1_tracker_destroy(tracker);
        return 1;
    }

    fprintf(stdout,
            "t,dt,sample_index,initialized,device_timestamp,"
            "q_w,q_x,q_y,q_z,q_norm,"
            "unbiased_gyro_x,unbiased_gyro_y,unbiased_gyro_z,"
            "angular_accel_x,angular_accel_y,angular_accel_z,"
            "drops\n");

    const double period_s = 1.0 / 60.0;
    double t0 = monotonic_seconds();
    double next_t = t0;
    uint64_t frame = 0;

    while (keep_running) {
        double now = monotonic_seconds();

        DK1TrackerState state;
        result = dk1_tracker_get_state(tracker, &state);
        if (result == DK1_OK) {
            DK1Quaternion q = state.orientation;

            fprintf(stdout,
                    "%.9f,%.9f,%llu,%d,%u,"
                    "%.9f,%.9f,%.9f,%.9f,%.9f,"
                    "%.9f,%.9f,%.9f,"
                    "%.9f,%.9f,%.9f,"
                    "%llu\n",
                    now - t0,
                    now - next_t + period_s,
                    (unsigned long long)state.sample_index,
                    state.initialized,
                    state.device_timestamp,
                    q.w, q.x, q.y, q.z, quat_norm(q),
                    state.unbiased_gyro.x,
                    state.unbiased_gyro.y,
                    state.unbiased_gyro.z,
                    state.angular_accel.x,
                    state.angular_accel.y,
                    state.angular_accel.z,
                    (unsigned long long)state.report_queue_dropped_count);
            fflush(stdout);
        } else {
            fprintf(stderr, "dk1_tracker_get_state failed: %d\n", result);
        }

        frame++;
        next_t = t0 + (double)frame * period_s;
        sleep_until(next_t);
    }

    dk1_tracker_stop(tracker);
    dk1_tracker_close(tracker);
    dk1_tracker_destroy(tracker);

    return 0;
}
