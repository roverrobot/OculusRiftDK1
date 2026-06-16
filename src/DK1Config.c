#include "DK1Config.h"
#include "DK1Tracker/DK1Error.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#define DK1_CONFIG_GRID_MIN 1
#define DK1_CONFIG_GRID_MAX 1024
#define DK1_CONFIG_IPD_MM_MIN 40
#define DK1_CONFIG_IPD_MM_MAX 90
#define DK1_CONFIG_EYE_HEIGHT_M_MIN 0.0
#define DK1_CONFIG_EYE_HEIGHT_M_MAX 3.0
#define DK1_CONFIG_MM_TO_M 0.001

void dk1_config_set_defaults(DK1Config *config) {
    if (!config) return;
    config->left_dial = DK1_DEFAULT_LEFT_DIAL;
    config->right_dial = DK1_DEFAULT_RIGHT_DIAL;
    config->grid_width = DK1_DEFAULT_GRID_WIDTH;
    config->grid_height = DK1_DEFAULT_GRID_HEIGHT;
    config->ipd_mm = DK1_DEFAULT_IPD_MM;
    config->eye_height_m = DK1_DEFAULT_EYE_HEIGHT_M;
    config->gyro_bias = (DK1Vector3){0.0, 0.0, 0.0};
    config->head_neck = (DK1HeadNeckConfig){
        .h_m = DK1_DEFAULT_HEAD_NECK_H_M,
        .ell_m = DK1_DEFAULT_HEAD_NECK_ELL_M,
        .ipd_m = (double)DK1_DEFAULT_IPD_MM * DK1_CONFIG_MM_TO_M,
        .pivot_damping_per_second = DK1_DEFAULT_HEAD_NECK_PIVOT_DAMPING_PER_SECOND,
        .max_dt_s = DK1_DEFAULT_HEAD_NECK_MAX_DT_S,
        .max_report_sample_count = DK1_DEFAULT_HEAD_NECK_MAX_REPORT_SAMPLE_COUNT,
        .use_pivot_inference = 1
    };
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
    if (
        config->eye_height_m <= DK1_CONFIG_EYE_HEIGHT_M_MIN ||
        config->eye_height_m > DK1_CONFIG_EYE_HEIGHT_M_MAX ||
        !isfinite(config->eye_height_m)
    ) {
        return DK1_ERROR_PARSE;
    }
    if (!isfinite(config->gyro_bias.x) ||
        !isfinite(config->gyro_bias.y) ||
        !isfinite(config->gyro_bias.z)) {
        return DK1_ERROR_PARSE;
    }
    if (config->head_neck.h_m < 0.0 || !isfinite(config->head_neck.h_m)) {
        return DK1_ERROR_PARSE;
    }
    if (config->head_neck.ell_m < 0.0 || !isfinite(config->head_neck.ell_m)) {
        return DK1_ERROR_PARSE;
    }
    if (config->head_neck.ipd_m <= 0.0 || !isfinite(config->head_neck.ipd_m)) {
        return DK1_ERROR_PARSE;
    }
    if (
        config->head_neck.pivot_damping_per_second < 0.0 ||
        !isfinite(config->head_neck.pivot_damping_per_second)
    ) {
        return DK1_ERROR_PARSE;
    }
    if (config->head_neck.max_dt_s <= 0.0 || !isfinite(config->head_neck.max_dt_s)) {
        return DK1_ERROR_PARSE;
    }
    if (config->head_neck.max_report_sample_count == 0) {
        return DK1_ERROR_PARSE;
    }
    if (
        config->head_neck.use_pivot_inference != 0 &&
        config->head_neck.use_pivot_inference != 1
    ) {
        return DK1_ERROR_PARSE;
    }
    return DK1_OK;
}

static char *trim_whitespace(char *text) {
    while (*text && isspace((unsigned char)*text)) {
        text++;
    }
    if (*text == '\0') return text;

    char *end = text + strlen(text) - 1;
    while (end >= text && isspace((unsigned char)*end)) {
        *end = '\0';
        end--;
    }
    return text;
}

static int parse_int_value(const char *value, int *out_value) {
    if (!value || !out_value) return 0;

    char *end = NULL;
    errno = 0;
    long parsed = strtol(value, &end, 10);
    if (errno != 0 || end == value) return 0;
    if (parsed < INT_MIN || parsed > INT_MAX) return 0;
    end = trim_whitespace(end);
    if (*end != '\0') return 0;

    *out_value = (int)parsed;
    return 1;
}

static int parse_double_value(const char *value, double *out_value) {
    if (!value || !out_value) return 0;

    char *end = NULL;
    errno = 0;
    double parsed = strtod(value, &end);
    if (errno != 0 || end == value) return 0;
    end = trim_whitespace(end);
    if (*end != '\0') return 0;

    *out_value = parsed;
    return 1;
}

static int parse_mm_value_as_m(const char *value, double *out_m) {
    double parsed_mm = 0.0;
    if (!parse_double_value(value, &parsed_mm)) return 0;
    *out_m = parsed_mm * DK1_CONFIG_MM_TO_M;
    return 1;
}

static int parse_flag_value(const char *value, int *out_value) {
    int parsed = 0;
    if (!parse_int_value(value, &parsed)) return 0;
    if (parsed != 0 && parsed != 1) return 0;
    *out_value = parsed;
    return 1;
}

static int parse_vector3_value(const char *value, DK1Vector3 *out_value) {
    if (!value || !out_value) return 0;

    char *end = NULL;
    errno = 0;
    double x = strtod(value, &end);
    if (errno != 0 || end == value) return 0;

    char *previous_end = end;
    errno = 0;
    double y = strtod(end, &end);
    if (errno != 0 || end == previous_end) return 0;

    previous_end = end;
    errno = 0;
    double z = strtod(end, &end);
    if (errno != 0 || end == previous_end) return 0;

    end = trim_whitespace(end);
    if (*end != '\0') return 0;

    out_value->x = x;
    out_value->y = y;
    out_value->z = z;
    return 1;
}

static int parse_key_value_line(char *line, DK1Config *config) {
    char *comment = strchr(line, '#');
    if (comment) *comment = '\0';

    char *text = trim_whitespace(line);
    if (*text == '\0') return DK1_OK;

    char *key = text;
    char *value = NULL;
    char *equals = strchr(text, '=');
    if (equals) {
        *equals = '\0';
        value = equals + 1;
    } else {
        value = text;
        while (*value && !isspace((unsigned char)*value)) {
            value++;
        }
        if (*value == '\0') return DK1_ERROR_PARSE;
        *value = '\0';
        value++;
    }

    key = trim_whitespace(key);
    value = trim_whitespace(value);
    if (*key == '\0' || *value == '\0') return DK1_ERROR_PARSE;

    if (strcmp(key, "left_dial") == 0) {
        return parse_int_value(value, &config->left_dial) ? DK1_OK : DK1_ERROR_PARSE;
    }
    if (strcmp(key, "right_dial") == 0) {
        return parse_int_value(value, &config->right_dial) ? DK1_OK : DK1_ERROR_PARSE;
    }
    if (strcmp(key, "grid_width") == 0) {
        return parse_int_value(value, &config->grid_width) ? DK1_OK : DK1_ERROR_PARSE;
    }
    if (strcmp(key, "grid_height") == 0) {
        return parse_int_value(value, &config->grid_height) ? DK1_OK : DK1_ERROR_PARSE;
    }
    if (strcmp(key, "ipd_mm") == 0) {
        int ipd_mm = 0;
        if (!parse_int_value(value, &ipd_mm)) return DK1_ERROR_PARSE;
        config->ipd_mm = ipd_mm;
        config->head_neck.ipd_m = (double)ipd_mm * DK1_CONFIG_MM_TO_M;
        return DK1_OK;
    }
    if (strcmp(key, "eye_height") == 0 || strcmp(key, "eye_height_m") == 0) {
        return parse_double_value(value, &config->eye_height_m) ?
            DK1_OK :
            DK1_ERROR_PARSE;
    }
    if (strcmp(key, "eye_height_mm") == 0) {
        return parse_mm_value_as_m(value, &config->eye_height_m) ?
            DK1_OK :
            DK1_ERROR_PARSE;
    }
    if (strcmp(key, "gyro_bias_rad_s") == 0) {
        return parse_vector3_value(value, &config->gyro_bias) ? DK1_OK : DK1_ERROR_PARSE;
    }
    if (
        strcmp(key, "h") == 0 ||
        strcmp(key, "h_mm") == 0 ||
        strcmp(key, "head_neck_h_mm") == 0
    ) {
        return parse_mm_value_as_m(value, &config->head_neck.h_m) ?
            DK1_OK :
            DK1_ERROR_PARSE;
    }
    if (
        strcmp(key, "ell") == 0 ||
        strcmp(key, "ell_mm") == 0 ||
        strcmp(key, "head_neck_ell_mm") == 0
    ) {
        return parse_mm_value_as_m(value, &config->head_neck.ell_m) ?
            DK1_OK :
            DK1_ERROR_PARSE;
    }
    if (strcmp(key, "h_m") == 0 || strcmp(key, "head_neck_h_m") == 0) {
        return parse_double_value(value, &config->head_neck.h_m) ?
            DK1_OK :
            DK1_ERROR_PARSE;
    }
    if (strcmp(key, "ell_m") == 0 || strcmp(key, "head_neck_ell_m") == 0) {
        return parse_double_value(value, &config->head_neck.ell_m) ?
            DK1_OK :
            DK1_ERROR_PARSE;
    }
    if (strcmp(key, "use_pivot_inference") == 0) {
        return parse_flag_value(value, &config->head_neck.use_pivot_inference) ?
            DK1_OK :
            DK1_ERROR_PARSE;
    }

    return DK1_ERROR_PARSE;
}

static int parse_key_value_config(FILE *file, DK1Config *config) {
    char line[512];
    while (fgets(line, sizeof(line), file)) {
        int result = parse_key_value_line(line, config);
        if (result != DK1_OK) return result;
    }
    if (ferror(file)) return DK1_ERROR_IO;
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

    int parse_result = parse_key_value_config(file, &config);
    int close_result = fclose(file);

    if (close_result != 0) {
        return DK1_ERROR_IO;
    }
    if (parse_result != DK1_OK) return parse_result;

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

static int ensure_parent_directory(const char *path) {
    const char *last_slash = strrchr(path, '/');
    if (!last_slash || last_slash == path) return DK1_OK;

    size_t len = (size_t)(last_slash - path);
    char dir[4096];
    if (len >= sizeof(dir)) return DK1_ERROR_INVALID_ARGUMENT;
    memcpy(dir, path, len);
    dir[len] = '\0';

    if (mkdir(dir, 0700) == 0) return DK1_OK;
    if (errno == EEXIST) return DK1_OK;
    return DK1_ERROR_IO;
}

int dk1_config_save_path(const char *path, const DK1Config *config) {
    if (!path || !config) return DK1_ERROR_INVALID_ARGUMENT;

    int validation = dk1_config_validate(config);
    if (validation != DK1_OK) return validation;

    int dir_result = ensure_parent_directory(path);
    if (dir_result != DK1_OK) return dir_result;

    errno = 0;
    FILE *file = fopen(path, "w");
    if (!file) return DK1_ERROR_IO;

    int ok = 1;
    ok = ok && fprintf(file, "# DK1Tracker configuration\n") >= 0;
    ok = ok && fprintf(file, "left_dial %d\n", config->left_dial) >= 0;
    ok = ok && fprintf(file, "right_dial %d\n", config->right_dial) >= 0;
    ok = ok && fprintf(file, "grid_width %d\n", config->grid_width) >= 0;
    ok = ok && fprintf(file, "grid_height %d\n", config->grid_height) >= 0;
    ok = ok && fprintf(file, "ipd_mm %d\n", config->ipd_mm) >= 0;
    ok = ok && fprintf(file, "eye_height %.17g\n", config->eye_height_m) >= 0;
    ok = ok && fprintf(file, "h %.17g\n", config->head_neck.h_m / DK1_CONFIG_MM_TO_M) >= 0;
    ok = ok && fprintf(file, "ell %.17g\n", config->head_neck.ell_m / DK1_CONFIG_MM_TO_M) >= 0;
    ok = ok && fprintf(
        file,
        "use_pivot_inference %d\n",
        config->head_neck.use_pivot_inference
    ) >= 0;
    ok = ok && fprintf(
        file,
        "gyro_bias_rad_s %.17g %.17g %.17g\n",
        config->gyro_bias.x,
        config->gyro_bias.y,
        config->gyro_bias.z
    ) >= 0;

    if (fclose(file) != 0) ok = 0;
    return ok ? DK1_OK : DK1_ERROR_IO;
}

int dk1_config_save_default(const DK1Config *config) {
    if (!config) return DK1_ERROR_INVALID_ARGUMENT;

    char path[4096];
    int path_result = dk1_config_default_path(path, sizeof(path));
    if (path_result != DK1_OK) return path_result;
    return dk1_config_save_path(path, config);
}
