#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include "DK1Tracker/DK1Tracker.h"

static volatile int keep_running = 1;

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

int main() {
    signal(SIGINT, handle_sigint);
    
    DK1Tracker *tracker = NULL;
    if (dk1_tracker_create(&tracker) != DK1_OK) {
        fprintf(stderr, "Failed to create tracker\n");
        return 1;
    }
    
    if (dk1_tracker_open(tracker) != DK1_OK) {
        fprintf(stderr, "Failed to open tracker\n");
        dk1_tracker_destroy(tracker);
        return 1;
    }
    
    dk1_tracker_set_keepalive(tracker, 10000);
    dk1_tracker_set_sample_callback(tracker, on_sample, NULL);
    
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
