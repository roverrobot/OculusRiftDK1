#include "DK1Tracker/DK1Error.h"
#include "DK1Tracker/DK1Tracker.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static int failures = 0;

static void check_int_equal(const char *name, int actual, int expected) {
    if (actual != expected) {
        fprintf(stderr, "%s: expected %d, got %d\n", name, expected, actual);
        failures++;
    }
}

static void check_true(const char *name, int condition) {
    if (!condition) {
        fprintf(stderr, "%s: condition failed\n", name);
        failures++;
    }
}

int main(void) {
    char home_template[] = "/tmp/dk1_keepalive_test_XXXXXX";
    char *home = mkdtemp(home_template);
    if (!home) {
        fprintf(stderr, "failed to create temporary HOME\n");
        return 1;
    }
    check_int_equal("setenv_home", setenv("HOME", home, 1), 0);

    DK1Tracker *tracker = NULL;
    check_int_equal("tracker_create", dk1_tracker_create(&tracker), DK1_OK);
    check_true(
        "default_refresh_before_timeout",
        DK1_DEFAULT_KEEPALIVE_REFRESH_INTERVAL_MS < DK1_KEEPALIVE_TIMEOUT_MS
    );

    check_int_equal(
        "refresh_interval_one_ms",
        dk1_tracker_set_keepalive_refresh_interval(tracker, 1),
        DK1_OK
    );
    check_int_equal(
        "refresh_interval_last_safe_ms",
        dk1_tracker_set_keepalive_refresh_interval(
            tracker,
            (uint16_t)(DK1_KEEPALIVE_TIMEOUT_MS - 1u)
        ),
        DK1_OK
    );
    check_int_equal(
        "refresh_interval_zero",
        dk1_tracker_set_keepalive_refresh_interval(tracker, 0),
        DK1_ERROR_INVALID_ARGUMENT
    );
    check_int_equal(
        "refresh_interval_timeout",
        dk1_tracker_set_keepalive_refresh_interval(
            tracker,
            (uint16_t)DK1_KEEPALIVE_TIMEOUT_MS
        ),
        DK1_ERROR_INVALID_ARGUMENT
    );
    check_int_equal(
        "refresh_interval_null_tracker",
        dk1_tracker_set_keepalive_refresh_interval(NULL, 1),
        DK1_ERROR_INVALID_ARGUMENT
    );

    dk1_tracker_pause_keepalive(tracker);
    dk1_tracker_pause_keepalive(NULL);
    check_int_equal("resume_keepalive", dk1_tracker_resume_keepalive(tracker), DK1_OK);
    check_int_equal(
        "resume_keepalive_null_tracker",
        dk1_tracker_resume_keepalive(NULL),
        DK1_ERROR_INVALID_ARGUMENT
    );
    check_int_equal(
        "one_shot_before_open",
        dk1_tracker_set_keepalive(tracker, (uint16_t)DK1_KEEPALIVE_TIMEOUT_MS),
        DK1_ERROR_NOT_OPEN
    );

    dk1_tracker_destroy(tracker);

    if (failures != 0) {
        fprintf(stderr, "%d keepalive test failure(s)\n", failures);
        return 1;
    }

    printf("keepalive tests passed\n");
    return 0;
}
