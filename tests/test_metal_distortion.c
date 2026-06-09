#include "DK1Config.h"
#include "DK1Tracker/DK1Error.h"
#include "DK1Tracker/DK1MetalDistortion.h"
#include "DK1Tracker/DK1Tracker.h"

#include <stddef.h>
#include <math.h>
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

static void test_vertex_layout(void) {
    check_size_equal("metal_vertex.size", sizeof(DK1MetalDistortionVertex), 40);
    check_size_equal(
        "metal_vertex.screen_offset",
        offsetof(DK1MetalDistortionVertex, screen_pos_ndc),
        0
    );
    check_size_equal(
        "metal_vertex.timewarp_offset",
        offsetof(DK1MetalDistortionVertex, timewarp_lerp),
        8
    );
    check_size_equal(
        "metal_vertex.shade_offset",
        offsetof(DK1MetalDistortionVertex, shade),
        12
    );
    check_size_equal(
        "metal_vertex.red_offset",
        offsetof(DK1MetalDistortionVertex, tan_eye_angles_r),
        16
    );
    check_size_equal(
        "metal_vertex.green_offset",
        offsetof(DK1MetalDistortionVertex, tan_eye_angles_g),
        24
    );
    check_size_equal(
        "metal_vertex.blue_offset",
        offsetof(DK1MetalDistortionVertex, tan_eye_angles_b),
        32
    );
    check_size_equal("metal_uniforms.size", sizeof(DK1MetalDistortionUniforms), 16);
}

static void test_eye_texture_uniforms(void) {
    DK1Config config;
    dk1_config_set_defaults(&config);

    DK1MetalDistortionUniforms left;
    DK1MetalDistortionUniforms right;
    check_int_equal(
        "left_uniforms",
        dk1_metal_distortion_make_eye_texture_uniforms(
            &config,
            DK1_EYE_LEFT,
            DK1_METAL_SOURCE_ORIGIN_TOP_LEFT,
            &left
        ),
        DK1_OK
    );
    check_int_equal(
        "right_uniforms",
        dk1_metal_distortion_make_eye_texture_uniforms(
            &config,
            DK1_EYE_RIGHT,
            DK1_METAL_SOURCE_ORIGIN_TOP_LEFT,
            &right
        ),
        DK1_OK
    );

    check_near("left.scale.x", left.source_uv_scale[0], 0.364584714286, 1e-7);
    check_near("left.scale.y", left.source_uv_scale[1], -0.364584714286, 1e-7);
    check_near("left.offset.x", left.source_uv_offset[0], 0.492857142857, 1e-7);
    check_near("left.offset.y", left.source_uv_offset[1], 0.5, 1e-7);
    check_near("right.offset.x", right.source_uv_offset[0], 0.507142857143, 1e-7);

    check_int_equal(
        "invalid_origin",
        dk1_metal_distortion_make_eye_texture_uniforms(
            &config,
            DK1_EYE_LEFT,
            (DK1MetalSourceOrigin)99,
            &left
        ),
        DK1_ERROR_INVALID_ARGUMENT
    );
}

static void test_vertex_copy(const char *home) {
    DK1Tracker *tracker = NULL;
    const DK1DistortionMesh *mesh = NULL;

    check_int_equal("setenv_vertex_copy", setenv("HOME", home, 1), 0);
    check_int_equal("tracker_create", dk1_tracker_create(&tracker), DK1_OK);
    check_int_equal(
        "get_left_mesh",
        dk1_tracker_get_distortion_mesh(tracker, DK1_EYE_LEFT, &mesh),
        DK1_OK
    );

    DK1MetalDistortionVertex *vertices =
        calloc(mesh->vertex_count, sizeof(DK1MetalDistortionVertex));
    check_true("calloc_vertices", vertices != NULL);
    if (vertices) {
        check_int_equal(
            "copy_small_buffer",
            dk1_metal_distortion_copy_vertices(mesh, vertices, mesh->vertex_count - 1),
            DK1_ERROR_INVALID_ARGUMENT
        );
        check_int_equal(
            "copy_vertices",
            dk1_metal_distortion_copy_vertices(mesh, vertices, mesh->vertex_count),
            DK1_OK
        );

        size_t center_index = 32u * 65u + 32u;
        const DK1DistortionMeshVertex *src = &mesh->vertices[center_index];
        const DK1MetalDistortionVertex *dst = &vertices[center_index];
        check_near("copy.screen.x", dst->screen_pos_ndc[0], src->screen_pos_ndc.x, 1e-6);
        check_near("copy.screen.y", dst->screen_pos_ndc[1], src->screen_pos_ndc.y, 1e-6);
        check_near("copy.shade", dst->shade, src->shade, 1e-6);
        check_near("copy.green.x", dst->tan_eye_angles_g[0], src->tan_eye_angles_g.x, 1e-6);
        free(vertices);
    }

    dk1_tracker_destroy(tracker);
}

int main(void) {
    char home_template[] = "/tmp/dk1_metal_distortion_test_XXXXXX";
    char *home = mkdtemp(home_template);
    if (!home) {
        fprintf(stderr, "failed to create temporary HOME\n");
        return 1;
    }

    test_vertex_layout();
    test_eye_texture_uniforms();
    test_vertex_copy(home);

    if (failures != 0) {
        fprintf(stderr, "%d Metal distortion test failure(s)\n", failures);
        return 1;
    }

    printf("Metal distortion tests passed\n");
    return 0;
}
