#include "DK1Config.h"
#include "DK1Tracker/DK1Error.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#define DK1_CONFIG_GRID_MIN 1
#define DK1_CONFIG_GRID_MAX 1024
#define DK1_CONFIG_IPD_MM_MIN 40
#define DK1_CONFIG_IPD_MM_MAX 90

void dk1_config_set_defaults(DK1Config *config) {
    if (!config) return;
    config->left_dial = DK1_DEFAULT_LEFT_DIAL;
    config->right_dial = DK1_DEFAULT_RIGHT_DIAL;
    config->grid_width = DK1_DEFAULT_GRID_WIDTH;
    config->grid_height = DK1_DEFAULT_GRID_HEIGHT;
    config->ipd_mm = DK1_DEFAULT_IPD_MM;
}

int dk1_config_validate(const DK1Config *config) {
    if (!config) return DK1_ERROR_INVALID_ARGUMENT;
    if (config->left_dial < 0 || config->left_dial > 10) {
        return DK1_ERROR_PARSE;
    }
    if (config->right_dial < 0 || config->right_dial > 10) {
        return DK1_ERROR_PARSE;
    }
    if (config->grid_width < DK1_CONFIG_GRID_MIN ||
        config->grid_width > DK1_CONFIG_GRID_MAX) {
        return DK1_ERROR_PARSE;
    }
    if (config->grid_height < DK1_CONFIG_GRID_MIN ||
        config->grid_height > DK1_CONFIG_GRID_MAX) {
        return DK1_ERROR_PARSE;
    }
    if (config->ipd_mm < DK1_CONFIG_IPD_MM_MIN ||
        config->ipd_mm > DK1_CONFIG_IPD_MM_MAX) {
        return DK1_ERROR_PARSE;
    }
    return DK1_OK;
}

int dk1_config_default_path(char *buffer, size_t buffer_size) {
    if (!buffer || buffer_size == 0) return DK1_ERROR_INVALID_ARGUMENT;

    const char *home = getenv("HOME");
    if (!home || home[0] == '\0') return DK1_ERROR_NOT_FOUND;

    int written = snprintf(
        buffer,
        buffer_size,
        "%s/.OculusRiftDK1/config.txt",
        home
    );
    if (written < 0 || (size_t)written >= buffer_size) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }
    return DK1_OK;
}

int dk1_config_load_path(const char *path, DK1Config *out_config) {
    if (!path || !out_config) return DK1_ERROR_INVALID_ARGUMENT;

    DK1Config config;
    dk1_config_set_defaults(&config);

    errno = 0;
    FILE *file = fopen(path, "r");
    if (!file) {
        return (errno == ENOENT) ? DK1_ERROR_NOT_FOUND : DK1_ERROR_IO;
    }

    int extra = 0;
    int count = fscanf(
        file,
        " %d %d %d %d %d %d",
        &config.left_dial,
        &config.right_dial,
        &config.grid_width,
        &config.grid_height,
        &config.ipd_mm,
        &extra
    );
    int close_result = fclose(file);

    if (close_result != 0) {
        return DK1_ERROR_IO;
    }
    if (count != 4 && count != 5) {
        return DK1_ERROR_PARSE;
    }

    int validation = dk1_config_validate(&config);
    if (validation != DK1_OK) {
        return validation;
    }

    *out_config = config;
    return DK1_OK;
}

int dk1_config_load_default(DK1Config *out_config) {
    if (!out_config) return DK1_ERROR_INVALID_ARGUMENT;

    dk1_config_set_defaults(out_config);

    char path[4096];
    int path_result = dk1_config_default_path(path, sizeof(path));
    if (path_result == DK1_ERROR_NOT_FOUND) {
        return DK1_OK;
    }
    if (path_result != DK1_OK) {
        return path_result;
    }

    DK1Config config;
    int load_result = dk1_config_load_path(path, &config);
    if (load_result == DK1_ERROR_NOT_FOUND) {
        return DK1_OK;
    }
    if (load_result != DK1_OK) {
        return load_result;
    }

    *out_config = config;
    return DK1_OK;
}
