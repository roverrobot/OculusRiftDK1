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

static void check_size_equal(const char *name, size_t actual, size_t expected) {
    if (actual != expected) {
        fprintf(
            stderr,
            "%s: expected %llu, got %llu\n",
            name,
            (unsigned long long)expected,
            (unsigned long long)actual
        );
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
            "%s: expected %.12f, got %.12f (error %.12f)\n",
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

static double vec2_norm(DK1Vector2 v) {
    return sqrt(v.x * v.x + v.y * v.y);
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

static void check_indices_in_range(const char *name, const DK1DistortionMesh *mesh) {
    for (size_t i = 0; i < mesh->index_count; ++i) {
        if (mesh->indices[i] >= mesh->vertex_count) {
            fprintf(
                stderr,
                "%s: index %llu out of range: %u >= %llu\n",
                name,
                (unsigned long long)i,
                mesh->indices[i],
                (unsigned long long)mesh->vertex_count
            );
            failures++;
            return;
        }
    }
}

static void check_default_mesh(const DK1DistortionMesh *mesh, DK1Eye eye) {
    check_int_equal("default.grid_width", mesh->grid_width, 64);
    check_int_equal("default.grid_height", mesh->grid_height, 64);
    check_size_equal("default.vertex_count", mesh->vertex_count, 65u * 65u);
    check_size_equal("default.triangle_count", mesh->triangle_count, 64u * 64u * 2u);
    check_size_equal("default.index_count", mesh->index_count, 64u * 64u * 6u);
    check_true("default.vertices", mesh->vertices != NULL);
    check_true("default.indices", mesh->indices != NULL);
    check_indices_in_range("default.indices", mesh);

    size_t center_index = 32u * 65u + 32u;
    const DK1DistortionMeshVertex *center = &mesh->vertices[center_index];
    double center_x = eye == DK1_EYE_LEFT ? -0.412783067216 : 0.412783067216;

    check_near("center.screen_x", center->screen_pos_ndc.x, center_x, 1e-9);
    check_near("center.screen_y", center->screen_pos_ndc.y, 0.0, 1e-12);
    check_near("center.r.x", center->tan_eye_angles_r.x, 0.0, 3e-2);
    check_near("center.g.x", center->tan_eye_angles_g.x, 0.0, 3e-2);
    check_near("center.b.x", center->tan_eye_angles_b.x, 0.0, 3e-2);
    check_near("center.r.y", center->tan_eye_angles_r.y, 0.0, 1e-12);
    check_near("center.g.y", center->tan_eye_angles_g.y, 0.0, 1e-12);
    check_near("center.b.y", center->tan_eye_angles_b.y, 0.0, 1e-12);
    check_near("center.timewarp", center->timewarp_lerp, 0.0, 1e-12);
    check_true("center.shade_finite", isfinite(center->shade));
    check_true("center.shade_max", center->shade <= 1.0);

    size_t edge_index = 32u * 65u + 64u;
    const DK1DistortionMeshVertex *edge = &mesh->vertices[edge_index];
    double red_norm = vec2_norm(edge->tan_eye_angles_r);
    double green_norm = vec2_norm(edge->tan_eye_angles_g);
    double blue_norm = vec2_norm(edge->tan_eye_angles_b);

    check_true("edge.red_green_order", red_norm < green_norm);
    check_true("edge.green_blue_order", green_norm < blue_norm);
    check_true("edge.screen_finite", isfinite(edge->screen_pos_ndc.x));
    check_true("edge.rgb_finite", isfinite(blue_norm));
}

static void test_default_tracker_mesh(const char *home) {
    DK1Tracker *tracker = NULL;
    const DK1DistortionMesh *left = NULL;
    const DK1DistortionMesh *right = NULL;

    check_int_equal("setenv_default", setenv("HOME", home, 1), 0);
    check_int_equal("tracker_create_default", dk1_tracker_create(&tracker), DK1_OK);
    check_int_equal(
        "get_left_mesh",
        dk1_tracker_get_distortion_mesh(tracker, DK1_EYE_LEFT, &left),
        DK1_OK
    );
    check_int_equal(
        "get_right_mesh",
        dk1_tracker_get_distortion_mesh(tracker, DK1_EYE_RIGHT, &right),
        DK1_OK
    );
    check_int_equal(
        "get_invalid_eye",
        dk1_tracker_get_distortion_mesh(tracker, (DK1Eye)2, &left),
        DK1_ERROR_INVALID_ARGUMENT
    );

    check_default_mesh(left, DK1_EYE_LEFT);
    check_default_mesh(right, DK1_EYE_RIGHT);

    dk1_tracker_destroy(tracker);
}

static void test_custom_grid_mesh(const char *home) {
    char dir_path[512];
    char config_path[512];
    DK1Tracker *tracker = NULL;
    const DK1DistortionMesh *left = NULL;
    const DK1DistortionMesh *right = NULL;

    if (!make_config_dir(dir_path, sizeof(dir_path), home) ||
        !make_config_path(config_path, sizeof(config_path), home) ||
        !write_text_file(config_path, "0 10 8 10 70\n")) {
        fprintf(stderr, "failed to write custom mesh config\n");
        failures++;
        return;
    }

    check_int_equal("setenv_custom", setenv("HOME", home, 1), 0);
    check_int_equal("tracker_create_custom", dk1_tracker_create(&tracker), DK1_OK);
    check_int_equal(
        "get_custom_left_mesh",
        dk1_tracker_get_distortion_mesh(tracker, DK1_EYE_LEFT, &left),
        DK1_OK
    );
    check_int_equal(
        "get_custom_right_mesh",
        dk1_tracker_get_distortion_mesh(tracker, DK1_EYE_RIGHT, &right),
        DK1_OK
    );

    check_int_equal("custom.left.grid_width", left->grid_width, 8);
    check_int_equal("custom.left.grid_height", left->grid_height, 10);
    check_size_equal("custom.left.vertex_count", left->vertex_count, 9u * 11u);
    check_size_equal("custom.left.triangle_count", left->triangle_count, 8u * 10u * 2u);
    check_size_equal("custom.left.index_count", left->index_count, 8u * 10u * 6u);
    check_indices_in_range("custom.left.indices", left);

    check_int_equal("custom.right.grid_width", right->grid_width, 8);
    check_int_equal("custom.right.grid_height", right->grid_height, 10);
    check_size_equal("custom.right.vertex_count", right->vertex_count, 9u * 11u);
    check_size_equal("custom.right.triangle_count", right->triangle_count, 8u * 10u * 2u);
    check_size_equal("custom.right.index_count", right->index_count, 8u * 10u * 6u);
    check_indices_in_range("custom.right.indices", right);

    dk1_tracker_destroy(tracker);
}

static void test_ipd_changes_mesh(const char *home) {
    char dir_path[512];
    char config_path[512];
    DK1Tracker *tracker_58 = NULL;
    DK1Tracker *tracker_70 = NULL;
    const DK1DistortionMesh *left_58 = NULL;
    const DK1DistortionMesh *left_70 = NULL;

    if (!make_config_dir(dir_path, sizeof(dir_path), home) ||
        !make_config_path(config_path, sizeof(config_path), home) ||
        !write_text_file(config_path, "5 5 64 64 58\n")) {
        fprintf(stderr, "failed to write IPD mesh config\n");
        failures++;
        return;
    }

    check_int_equal("setenv_ipd_58", setenv("HOME", home, 1), 0);
    check_int_equal("tracker_create_ipd_58", dk1_tracker_create(&tracker_58), DK1_OK);
    check_int_equal(
        "get_ipd_58_left_mesh",
        dk1_tracker_get_distortion_mesh(tracker_58, DK1_EYE_LEFT, &left_58),
        DK1_OK
    );

    if (!write_text_file(config_path, "5 5 64 64 70\n")) {
        fprintf(stderr, "failed to rewrite IPD mesh config\n");
        failures++;
        dk1_tracker_destroy(tracker_58);
        return;
    }

    check_int_equal("tracker_create_ipd_70", dk1_tracker_create(&tracker_70), DK1_OK);
    check_int_equal(
        "get_ipd_70_left_mesh",
        dk1_tracker_get_distortion_mesh(tracker_70, DK1_EYE_LEFT, &left_70),
        DK1_OK
    );

    size_t center_index = 32u * 65u + 32u;
    const DK1DistortionMeshVertex *center_58 = &left_58->vertices[center_index];
    const DK1DistortionMeshVertex *center_70 = &left_70->vertices[center_index];
    double screen_delta = fabs(center_70->screen_pos_ndc.x - center_58->screen_pos_ndc.x);
    double tan_delta = fabs(center_70->tan_eye_angles_g.x - center_58->tan_eye_angles_g.x);

    check_true("ipd_changes_screen_x", screen_delta > 1e-4);
    check_true("ipd_changes_tan_x", tan_delta > 1e-4);

    dk1_tracker_destroy(tracker_58);
    dk1_tracker_destroy(tracker_70);
}

int main(void) {
    char default_home_template[] = "/tmp/dk1_mesh_default_test_XXXXXX";
    char custom_home_template[] = "/tmp/dk1_mesh_custom_test_XXXXXX";
    char ipd_home_template[] = "/tmp/dk1_mesh_ipd_test_XXXXXX";
    char *default_home = mkdtemp(default_home_template);
    char *custom_home = mkdtemp(custom_home_template);
    char *ipd_home = mkdtemp(ipd_home_template);
    if (!default_home || !custom_home || !ipd_home) {
        fprintf(stderr, "failed to create temporary HOME\n");
        return 1;
    }

    test_default_tracker_mesh(default_home);
    test_custom_grid_mesh(custom_home);
    test_ipd_changes_mesh(ipd_home);

    if (failures != 0) {
        fprintf(stderr, "%d distortion mesh test failure(s)\n", failures);
        return 1;
    }

    printf("distortion mesh tests passed\n");
    return 0;
}
