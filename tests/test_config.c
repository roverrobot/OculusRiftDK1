#include "DK1Config.h"
#include "DK1Tracker/DK1Error.h"
#include "DK1Tracker/DK1Tracker.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static int failures = 0;

static void check_double_close(const char *name, double actual, double expected) {
    if (fabs(actual - expected) > 1e-12) {
        fprintf(stderr, "%s: expected %.12f, got %.12f\n", name, expected, actual);
        failures++;
    }
}

static void check_int_equal(const char *name, int actual, int expected) {
    if (actual != expected) {
        fprintf(stderr, "%s: expected %d, got %d\n", name, expected, actual);
        failures++;
    }
}

static void check_config(
    const char *prefix,
    const DK1Config *config,
    int left_dial,
    int right_dial,
    int grid_width,
    int grid_height,
    int ipd_mm,
    double eye_height_m
) {
    char name[128];
    snprintf(name, sizeof(name), "%s.left_dial", prefix);
    check_int_equal(name, config->left_dial, left_dial);
    snprintf(name, sizeof(name), "%s.right_dial", prefix);
    check_int_equal(name, config->right_dial, right_dial);
    snprintf(name, sizeof(name), "%s.grid_width", prefix);
    check_int_equal(name, config->grid_width, grid_width);
    snprintf(name, sizeof(name), "%s.grid_height", prefix);
    check_int_equal(name, config->grid_height, grid_height);
    snprintf(name, sizeof(name), "%s.ipd_mm", prefix);
    check_int_equal(name, config->ipd_mm, ipd_mm);
    snprintf(name, sizeof(name), "%s.eye_height_m", prefix);
    check_double_close(name, config->eye_height_m, eye_height_m);
}

static void check_vector_close(
    const char *prefix,
    DK1Vector3 actual,
    DK1Vector3 expected
) {
    char name[128];
    snprintf(name, sizeof(name), "%s.x", prefix);
    check_double_close(name, actual.x, expected.x);
    snprintf(name, sizeof(name), "%s.y", prefix);
    check_double_close(name, actual.y, expected.y);
    snprintf(name, sizeof(name), "%s.z", prefix);
    check_double_close(name, actual.z, expected.z);
}

static void check_head_neck_config(
    const char *prefix,
    const DK1HeadNeckConfig *config,
    double h_m,
    double ell_m,
    double ipd_m
) {
    char name[128];
    snprintf(name, sizeof(name), "%s.h_m", prefix);
    check_double_close(name, config->h_m, h_m);
    snprintf(name, sizeof(name), "%s.ell_m", prefix);
    check_double_close(name, config->ell_m, ell_m);
    snprintf(name, sizeof(name), "%s.ipd_m", prefix);
    check_double_close(name, config->ipd_m, ipd_m);
    snprintf(name, sizeof(name), "%s.pivot_damping_per_second", prefix);
    check_double_close(
        name,
        config->pivot_damping_per_second,
        DK1_DEFAULT_HEAD_NECK_PIVOT_DAMPING_PER_SECOND
    );
    snprintf(name, sizeof(name), "%s.max_dt_s", prefix);
    check_double_close(name, config->max_dt_s, DK1_DEFAULT_HEAD_NECK_MAX_DT_S);
    snprintf(name, sizeof(name), "%s.max_report_sample_count", prefix);
    check_int_equal(
        name,
        (int)config->max_report_sample_count,
        (int)DK1_DEFAULT_HEAD_NECK_MAX_REPORT_SAMPLE_COUNT
    );
    snprintf(name, sizeof(name), "%s.use_pivot_inference", prefix);
    check_int_equal(name, config->use_pivot_inference, 1);
}

static int write_text_file(const char *path, const char *text) {
    FILE *file = fopen(path, "w");
    if (!file) return 0;
    int ok = fputs(text, file) >= 0;
    if (fclose(file) != 0) ok = 0;
    return ok;
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

static int make_config_dir(char *buffer, size_t buffer_size, const char *home) {
    int written = snprintf(buffer, buffer_size, "%s/.OculusRiftDK1", home);
    if (written <= 0 || (size_t)written >= buffer_size) return 0;
    if (mkdir(buffer, 0700) == 0) return 1;
    return errno == EEXIST;
}

static void test_defaults(void) {
    DK1Config config;
    dk1_config_set_defaults(&config);
    check_config("defaults", &config, 5, 5, 64, 64, 64, DK1_DEFAULT_EYE_HEIGHT_M);
    check_vector_close("defaults.gyro_bias", config.gyro_bias, (DK1Vector3){0.0, 0.0, 0.0});
    check_head_neck_config("defaults.head_neck", &config.head_neck, 0.10, 0.16, 0.064);
    check_int_equal("defaults_validate", dk1_config_validate(&config), DK1_OK);
}

static void test_missing_file_uses_defaults(const char *home) {
    DK1Config config = {0};

    check_int_equal("setenv_missing", setenv("HOME", home, 1), 0);
    check_int_equal("missing_load", dk1_config_load_default(&config), DK1_OK);
    check_config("missing_load", &config, 5, 5, 64, 64, 64, DK1_DEFAULT_EYE_HEIGHT_M);
    check_vector_close("missing_load.gyro_bias", config.gyro_bias, (DK1Vector3){0.0, 0.0, 0.0});
    check_head_neck_config("missing_load.head_neck", &config.head_neck, 0.10, 0.16, 0.064);
}

static void test_valid_file(const char *home) {
    char dir_path[512];
    char config_path[512];
    DK1Config config = {0};

    if (!make_config_dir(dir_path, sizeof(dir_path), home)) {
        fprintf(stderr, "failed to make config dir\n");
        failures++;
        return;
    }
    if (!make_config_path(config_path, sizeof(config_path), home) ||
        !write_text_file(
            config_path,
            "left_dial 2\n"
            "right_dial 8\n"
            "grid_width 32\n"
            "grid_height 48\n"
            "ipd_mm 67\n"
            "eye_height 1.62\n"
            "h 101.13\n"
            "ell 159.02\n"
            "use_pivot_inference 1\n"
            "gyro_bias_rad_s -0.0412516 0.0256156 0.0005428\n"
        )) {
        fprintf(stderr, "failed to write valid config\n");
        failures++;
        return;
    }

    check_int_equal("setenv_valid", setenv("HOME", home, 1), 0);
    check_int_equal("valid_load", dk1_config_load_default(&config), DK1_OK);
    check_config("valid_load", &config, 2, 8, 32, 48, 67, 1.62);
    check_vector_close(
        "valid_load.gyro_bias",
        config.gyro_bias,
        (DK1Vector3){-0.0412516, 0.0256156, 0.0005428}
    );
    check_head_neck_config(
        "valid_load.head_neck",
        &config.head_neck,
        0.10113,
        0.15902,
        0.067
    );
}

static void test_save_file(const char *home) {
    char config_path[512];
    DK1Config config;
    DK1Config loaded = {0};

    dk1_config_set_defaults(&config);
    config.left_dial = 1;
    config.right_dial = 9;
    config.grid_width = 40;
    config.grid_height = 50;
    config.ipd_mm = 68;
    config.eye_height_m = 1.70;
    config.gyro_bias = (DK1Vector3){-0.01, 0.02, -0.03};
    config.head_neck.h_m = 0.10113;
    config.head_neck.ell_m = 0.15902;
    config.head_neck.ipd_m = 0.068;
    config.head_neck.use_pivot_inference = 0;

    if (!make_config_path(config_path, sizeof(config_path), home)) {
        fprintf(stderr, "failed to format config path\n");
        failures++;
        return;
    }

    check_int_equal("save_path", dk1_config_save_path(config_path, &config), DK1_OK);
    check_int_equal("save_load_path", dk1_config_load_path(config_path, &loaded), DK1_OK);
    check_config("save_load", &loaded, 1, 9, 40, 50, 68, 1.70);
    check_vector_close("save_load.gyro_bias", loaded.gyro_bias, config.gyro_bias);
    check_double_close("save_load.head_neck.h_m", loaded.head_neck.h_m, 0.10113);
    check_double_close("save_load.head_neck.ell_m", loaded.head_neck.ell_m, 0.15902);
    check_double_close("save_load.head_neck.ipd_m", loaded.head_neck.ipd_m, 0.068);
    check_int_equal("save_load.head_neck.use_pivot_inference", loaded.head_neck.use_pivot_inference, 0);
}

static void test_invalid_file(const char *home) {
    char config_path[512];
    DK1Config config;

    if (!make_config_path(config_path, sizeof(config_path), home)) {
        fprintf(stderr, "failed to format config path\n");
        failures++;
        return;
    }

    if (!write_text_file(config_path, "left_dial 11\n")) {
        fprintf(stderr, "failed to write invalid dial config\n");
        failures++;
        return;
    }
    check_int_equal(
        "invalid_dial",
        dk1_config_load_path(config_path, &config),
        DK1_ERROR_PARSE
    );

    if (!write_text_file(config_path, "left_dial\n")) {
        fprintf(stderr, "failed to write missing-value config\n");
        failures++;
        return;
    }
    check_int_equal(
        "missing_value",
        dk1_config_load_path(config_path, &config),
        DK1_ERROR_PARSE
    );

    if (!write_text_file(config_path, "5 5 64 64 64\n")) {
        fprintf(stderr, "failed to write numeric config\n");
        failures++;
        return;
    }
    check_int_equal(
        "numeric_config",
        dk1_config_load_path(config_path, &config),
        DK1_ERROR_PARSE
    );

    if (!write_text_file(config_path, "grid_width 0\n")) {
        fprintf(stderr, "failed to write invalid grid config\n");
        failures++;
        return;
    }
    check_int_equal(
        "invalid_grid",
        dk1_config_load_path(config_path, &config),
        DK1_ERROR_PARSE
    );

    if (!write_text_file(config_path, "ipd_mm 0\n")) {
        fprintf(stderr, "failed to write invalid IPD config\n");
        failures++;
        return;
    }
    check_int_equal(
        "invalid_ipd",
        dk1_config_load_path(config_path, &config),
        DK1_ERROR_PARSE
    );

    if (!write_text_file(config_path, "eye_height 0\n")) {
        fprintf(stderr, "failed to write invalid eye height config\n");
        failures++;
        return;
    }
    check_int_equal(
        "invalid_eye_height",
        dk1_config_load_path(config_path, &config),
        DK1_ERROR_PARSE
    );

    if (!write_text_file(config_path, "gyro_bias_rad_s 1 2\n")) {
        fprintf(stderr, "failed to write invalid gyro bias config\n");
        failures++;
        return;
    }
    check_int_equal(
        "invalid_gyro_bias",
        dk1_config_load_path(config_path, &config),
        DK1_ERROR_PARSE
    );

    if (!write_text_file(config_path, "h -1\n")) {
        fprintf(stderr, "failed to write invalid h config\n");
        failures++;
        return;
    }
    check_int_equal(
        "invalid_head_neck_h",
        dk1_config_load_path(config_path, &config),
        DK1_ERROR_PARSE
    );

    if (!write_text_file(config_path, "use_pivot_inference 2\n")) {
        fprintf(stderr, "failed to write invalid pivot inference config\n");
        failures++;
        return;
    }
    check_int_equal(
        "invalid_use_pivot_inference",
        dk1_config_load_path(config_path, &config),
        DK1_ERROR_PARSE
    );
}

static void test_tracker_loads_config(const char *home) {
    char config_path[512];
    DK1Tracker *tracker = NULL;
    DK1Config config = {0};

    if (!make_config_path(config_path, sizeof(config_path), home) ||
        !write_text_file(
            config_path,
            "left_dial 3\n"
            "right_dial 7\n"
            "grid_width 80\n"
            "grid_height 96\n"
            "ipd_mm 66\n"
            "eye_height 1.64\n"
            "h 101.13\n"
            "ell 159.02\n"
            "use_pivot_inference 1\n"
            "gyro_bias_rad_s -0.1 0.2 -0.3\n"
        )) {
        fprintf(stderr, "failed to write tracker config\n");
        failures++;
        return;
    }

    check_int_equal("tracker_create", dk1_tracker_create(&tracker), DK1_OK);
    check_int_equal("tracker_get_config", dk1_tracker_get_config(tracker, &config), DK1_OK);
    check_config("tracker_config", &config, 3, 7, 80, 96, 66, 1.64);
    check_vector_close("tracker_config.gyro_bias", config.gyro_bias, (DK1Vector3){-0.1, 0.2, -0.3});
    check_head_neck_config(
        "tracker_config.head_neck",
        &config.head_neck,
        0.10113,
        0.15902,
        0.066
    );

    DK1HeadNeckConfig active_head_neck = {0};
    check_int_equal(
        "tracker_get_head_neck_config",
        dk1_tracker_get_head_neck_config(tracker, &active_head_neck),
        DK1_OK
    );
    check_head_neck_config(
        "tracker_active_head_neck",
        &active_head_neck,
        0.10113,
        0.15902,
        0.066
    );
    dk1_tracker_destroy(tracker);
}

int main(void) {
    char home_template[] = "/tmp/dk1_config_test_XXXXXX";
    char *home = mkdtemp(home_template);
    if (!home) {
        fprintf(stderr, "failed to create temporary HOME\n");
        return 1;
    }

    test_defaults();
    test_missing_file_uses_defaults(home);
    test_save_file(home);
    test_valid_file(home);
    test_invalid_file(home);
    test_tracker_loads_config(home);

    if (failures != 0) {
        fprintf(stderr, "%d config test failure(s)\n", failures);
        return 1;
    }

    printf("config tests passed\n");
    return 0;
}
