#include "DK1Config.h"
#include "DK1Tracker/DK1Error.h"
#include "DK1Tracker/DK1Tracker.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static int failures = 0;

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
    int grid_height
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
    return mkdir(buffer, 0700) == 0;
}

static void test_defaults(void) {
    DK1Config config;
    dk1_config_set_defaults(&config);
    check_config("defaults", &config, 5, 5, 64, 64);
    check_int_equal("defaults_validate", dk1_config_validate(&config), DK1_OK);
}

static void test_missing_file_uses_defaults(const char *home) {
    DK1Config config = {0, 0, 0, 0};

    check_int_equal("setenv_missing", setenv("HOME", home, 1), 0);
    check_int_equal("missing_load", dk1_config_load_default(&config), DK1_OK);
    check_config("missing_load", &config, 5, 5, 64, 64);
}

static void test_valid_file(const char *home) {
    char dir_path[512];
    char config_path[512];
    DK1Config config = {0, 0, 0, 0};

    if (!make_config_dir(dir_path, sizeof(dir_path), home)) {
        fprintf(stderr, "failed to make config dir\n");
        failures++;
        return;
    }
    if (!make_config_path(config_path, sizeof(config_path), home) ||
        !write_text_file(config_path, "2 8 32 48\n")) {
        fprintf(stderr, "failed to write valid config\n");
        failures++;
        return;
    }

    check_int_equal("setenv_valid", setenv("HOME", home, 1), 0);
    check_int_equal("valid_load", dk1_config_load_default(&config), DK1_OK);
    check_config("valid_load", &config, 2, 8, 32, 48);
}

static void test_invalid_file(const char *home) {
    char config_path[512];
    DK1Config config;

    if (!make_config_path(config_path, sizeof(config_path), home)) {
        fprintf(stderr, "failed to format config path\n");
        failures++;
        return;
    }

    if (!write_text_file(config_path, "11 5 64 64\n")) {
        fprintf(stderr, "failed to write invalid dial config\n");
        failures++;
        return;
    }
    check_int_equal(
        "invalid_dial",
        dk1_config_load_path(config_path, &config),
        DK1_ERROR_PARSE
    );

    if (!write_text_file(config_path, "5 5 64\n")) {
        fprintf(stderr, "failed to write short config\n");
        failures++;
        return;
    }
    check_int_equal(
        "short_config",
        dk1_config_load_path(config_path, &config),
        DK1_ERROR_PARSE
    );

    if (!write_text_file(config_path, "5 5 0 64\n")) {
        fprintf(stderr, "failed to write invalid grid config\n");
        failures++;
        return;
    }
    check_int_equal(
        "invalid_grid",
        dk1_config_load_path(config_path, &config),
        DK1_ERROR_PARSE
    );
}

static void test_tracker_loads_config(const char *home) {
    char config_path[512];
    DK1Tracker *tracker = NULL;
    DK1Config config = {0, 0, 0, 0};

    if (!make_config_path(config_path, sizeof(config_path), home) ||
        !write_text_file(config_path, "3 7 80 96\n")) {
        fprintf(stderr, "failed to write tracker config\n");
        failures++;
        return;
    }

    check_int_equal("tracker_create", dk1_tracker_create(&tracker), DK1_OK);
    check_int_equal("tracker_get_config", dk1_tracker_get_config(tracker, &config), DK1_OK);
    check_config("tracker_config", &config, 3, 7, 80, 96);
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
