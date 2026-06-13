#include "DK1Config.h"
#include "DK1Tracker/DK1Error.h"
#include "DK1Tracker/DK1Tracker.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

static int failures = 0;

static void check_int_equal(const char *name, int actual, int expected) {
    if (actual != expected) {
        fprintf(stderr, "%s: expected %d, got %d\n", name, expected, actual);
        failures++;
    }
}

static void check_near(
    const char *name,
    double actual,
    double expected,
    double tolerance
) {
    double error = fabs(actual - expected);
    if (error > tolerance) {
        fprintf(
            stderr,
            "%s: expected %.15f, got %.15f (error %.15f)\n",
            name,
            expected,
            actual,
            error
        );
        failures++;
    }
}

static void check_true(const char *name, int condition) {
    if (!condition) {
        fprintf(stderr, "%s: condition failed\n", name);
        failures++;
    }
}

static int make_config_dir(char *buffer, size_t buffer_size, const char *home) {
    int written = snprintf(buffer, buffer_size, "%s/.OculusRiftDK1", home);
    if (written <= 0 || (size_t)written >= buffer_size) return 0;
    if (mkdir(buffer, 0700) == 0) return 1;
    return errno == EEXIST;
}

static int make_config_path(char *buffer, size_t buffer_size, const char *home) {
    int written = snprintf(
        buffer,
        buffer_size,
        "%s/.OculusRiftDK1/config.txt",
        home
    );
    return written > 0 && (size_t)written < buffer_size;
}

static int write_text_file(const char *path, const char *text) {
    FILE *file = fopen(path, "w");
    if (!file) return 0;
    int ok = fputs(text, file) >= 0;
    if (fclose(file) != 0) ok = 0;
    return ok;
}

static void check_projection(
    const char *prefix,
    DK1EyeProjection projection,
    double left_tan,
    double right_tan,
    double top_tan,
    double bottom_tan,
    double eye_offset_m
) {
    char name[128];
    snprintf(name, sizeof(name), "%s.left_tan", prefix);
    check_near(name, projection.left_tan, left_tan, 1e-12);
    snprintf(name, sizeof(name), "%s.right_tan", prefix);
    check_near(name, projection.right_tan, right_tan, 1e-12);
    snprintf(name, sizeof(name), "%s.top_tan", prefix);
    check_near(name, projection.top_tan, top_tan, 1e-12);
    snprintf(name, sizeof(name), "%s.bottom_tan", prefix);
    check_near(name, projection.bottom_tan, bottom_tan, 1e-12);
    snprintf(name, sizeof(name), "%s.eye_offset_m", prefix);
    check_near(name, projection.eye_offset_m, eye_offset_m, 1e-12);
}

static void test_default_config_projection(void) {
    DK1Config config;
    dk1_config_set_defaults(&config);

    DK1EyeProjection left;
    DK1EyeProjection right;
    check_int_equal(
        "default_left_projection",
        dk1_config_make_eye_projection(&config, DK1_EYE_LEFT, &left),
        DK1_OK
    );
    check_int_equal(
        "default_right_projection",
        dk1_config_make_eye_projection(&config, DK1_EYE_RIGHT, &right),
        DK1_OK
    );

    check_projection(
        "default_left",
        left,
        -1.351831614286783,
        1.391015139338574,
        1.371423376812679,
        -1.371423376812679,
        -0.032
    );
    check_projection(
        "default_right",
        right,
        -1.391015139338574,
        1.351831614286783,
        1.371423376812679,
        -1.371423376812679,
        0.032
    );

    check_true("default_left_negative", left.left_tan < 0.0);
    check_true("default_right_positive", left.right_tan > 0.0);
    check_true("default_top_positive", left.top_tan > 0.0);
    check_true("default_bottom_negative", left.bottom_tan < 0.0);
}

static void test_asymmetric_ipd_and_dial_projection(void) {
    DK1Config config;
    dk1_config_set_defaults(&config);
    config.left_dial = 0;
    config.right_dial = 10;
    config.ipd_mm = 70;
    config.head_neck.ipd_m = 0.070;

    DK1EyeProjection left;
    DK1EyeProjection right;
    check_int_equal(
        "asymmetric_left_projection",
        dk1_config_make_eye_projection(&config, DK1_EYE_LEFT, &left),
        DK1_OK
    );
    check_int_equal(
        "asymmetric_right_projection",
        dk1_config_make_eye_projection(&config, DK1_EYE_RIGHT, &right),
        DK1_OK
    );

    check_projection(
        "asymmetric_left",
        left,
        -1.836230174351666,
        2.673808850371724,
        2.255019512361695,
        -2.255019512361695,
        -0.035
    );
    check_projection(
        "asymmetric_right",
        right,
        -1.168325266258513,
        0.802343857551027,
        0.985334561904770,
        -0.985334561904770,
        0.035
    );

    check_true("asymmetric_left_is_wider", left.top_tan > right.top_tan);
    check_true(
        "asymmetric_left_horizontal_shift",
        fabs(left.left_tan + left.right_tan) > 0.5
    );
    check_true(
        "asymmetric_right_horizontal_shift",
        fabs(right.left_tan + right.right_tan) > 0.3
    );
}

static void test_invalid_projection_arguments(void) {
    DK1Config config;
    dk1_config_set_defaults(&config);
    DK1EyeProjection projection;

    check_int_equal(
        "projection_null_config",
        dk1_config_make_eye_projection(NULL, DK1_EYE_LEFT, &projection),
        DK1_ERROR_INVALID_ARGUMENT
    );
    check_int_equal(
        "projection_null_out",
        dk1_config_make_eye_projection(&config, DK1_EYE_LEFT, NULL),
        DK1_ERROR_INVALID_ARGUMENT
    );
    check_int_equal(
        "projection_invalid_eye",
        dk1_config_make_eye_projection(&config, (DK1Eye)2, &projection),
        DK1_ERROR_INVALID_ARGUMENT
    );

    config.left_dial = 11;
    check_int_equal(
        "projection_invalid_config",
        dk1_config_make_eye_projection(&config, DK1_EYE_LEFT, &projection),
        DK1_ERROR_PARSE
    );
}

static void test_tracker_projection_wrapper(const char *home) {
    char dir_path[512];
    char config_path[512];
    DK1Tracker *tracker = NULL;
    DK1EyeProjection projection;

    if (!make_config_dir(dir_path, sizeof(dir_path), home) ||
        !make_config_path(config_path, sizeof(config_path), home) ||
        !write_text_file(
            config_path,
            "left_dial 0\n"
            "right_dial 10\n"
            "grid_width 8\n"
            "grid_height 8\n"
            "ipd_mm 70\n"
        )) {
        fprintf(stderr, "failed to write projection config\n");
        failures++;
        return;
    }

    check_int_equal("setenv_projection", setenv("HOME", home, 1), 0);
    check_int_equal("projection_tracker_create", dk1_tracker_create(&tracker), DK1_OK);
    check_int_equal(
        "tracker_get_left_projection",
        dk1_tracker_get_eye_projection(tracker, DK1_EYE_LEFT, &projection),
        DK1_OK
    );
    check_projection(
        "tracker_left",
        projection,
        -1.836230174351666,
        2.673808850371724,
        2.255019512361695,
        -2.255019512361695,
        -0.035
    );
    check_int_equal(
        "tracker_get_invalid_eye_projection",
        dk1_tracker_get_eye_projection(tracker, (DK1Eye)2, &projection),
        DK1_ERROR_INVALID_ARGUMENT
    );
    check_int_equal(
        "tracker_get_null_projection",
        dk1_tracker_get_eye_projection(NULL, DK1_EYE_LEFT, &projection),
        DK1_ERROR_INVALID_ARGUMENT
    );

    dk1_tracker_destroy(tracker);
}

int main(void) {
    char tracker_home_template[] = "/tmp/dk1_projection_test_XXXXXX";
    char *tracker_home = mkdtemp(tracker_home_template);
    if (!tracker_home) {
        fprintf(stderr, "failed to create temporary HOME\n");
        return 1;
    }

    test_default_config_projection();
    test_asymmetric_ipd_and_dial_projection();
    test_invalid_projection_arguments();
    test_tracker_projection_wrapper(tracker_home);

    if (failures != 0) {
        fprintf(stderr, "%d eye projection test failure(s)\n", failures);
        return 1;
    }

    printf("eye projection tests passed\n");
    return 0;
}
