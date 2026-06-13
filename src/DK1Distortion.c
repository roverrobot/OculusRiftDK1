#include "DK1Distortion.h"
#include "DK1Config.h"
#include "DK1Tracker/DK1Error.h"
#include "DK1Tracker/DK1Tracker.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define DK1_DISTORTION_COEFFICIENT_COUNT 11

static const double DK1_SCREEN_WIDTH_M = 0.1498;
static const double DK1_SCREEN_HEIGHT_M = 0.0936;
static const double DK1_SCREEN_GAP_M = 0.0;
static const double DK1_LENS_SEPARATION_M = 0.0635;
static const double DK1_CENTER_FROM_TOP_M = 0.0468;
static const double DK1_METERS_PER_TAN_ANGLE = 0.0425;
static const double DK1_VISIBLE_LENS_RADIUS_M = 0.0175;
static const double DK1_BASE_EYE_RELIEF_M = 0.012760465;

typedef struct DK1LensConfig {
    double k[DK1_DISTORTION_COEFFICIENT_COUNT];
    double max_r;
    double chroma[4];
} DK1LensConfig;

typedef struct DK1DistortionAnchor {
    double eye_relief_m;
    double max_r;
    double k[DK1_DISTORTION_COEFFICIENT_COUNT];
} DK1DistortionAnchor;

static const DK1DistortionAnchor DK1_DISTORTION_ANCHORS[3] = {
    {
        0.012760465 - 0.005,
        1.3416407864998738178,
        {1.0000, 1.06505, 1.14725, 1.2705, 1.48, 1.87, 2.534, 3.6, 5.1, 7.4, 11.0}
    },
    {
        0.012760465,
        1.0,
        {1.0, 1.032407264, 1.07160462, 1.11998388, 1.1808606, 1.2590494, 1.361915, 1.5014339, 1.6986004, 1.9940577, 2.4783147}
    },
    {
        0.012760465 + 0.005,
        1.0,
        {1.0102, 1.0371, 1.0831, 1.1353, 1.2, 1.2851, 1.3979, 1.56, 1.8, 2.25, 3.0}
    }
};

static double min_double(double a, double b) {
    return a < b ? a : b;
}

static double max_double(double a, double b) {
    return a > b ? a : b;
}

static double clamp_double(double value, double min_value, double max_value) {
    return max_double(min_value, min_double(value, max_value));
}

static double vec2_length(DK1Vector2 v) {
    return sqrt(v.x * v.x + v.y * v.y);
}

static DK1Vector2 vec2_scale(DK1Vector2 v, double scale) {
    return (DK1Vector2){v.x * scale, v.y * scale};
}

static double eval_catmull_rom10_spline(const double *k_values, double scaled_value) {
    const int num_segments = DK1_DISTORTION_COEFFICIENT_COUNT;
    double scaled_floor = floor(scaled_value);
    scaled_floor = clamp_double(scaled_floor, 0.0, (double)(num_segments - 1));
    double t = scaled_value - scaled_floor;
    int k = (int)scaled_floor;

    double p0;
    double p1;
    double m0;
    double m1;

    if (k == 0) {
        p0 = 1.0;
        m0 = k_values[1] - k_values[0];
        p1 = k_values[1];
        m1 = 0.5 * (k_values[2] - k_values[0]);
    } else if (k == num_segments - 2) {
        p0 = k_values[num_segments - 2];
        m0 = 0.5 * (k_values[num_segments - 1] - k_values[num_segments - 2]);
        p1 = k_values[num_segments - 1];
        m1 = k_values[num_segments - 1] - k_values[num_segments - 2];
    } else if (k == num_segments - 1) {
        p0 = k_values[num_segments - 1];
        m0 = k_values[num_segments - 1] - k_values[num_segments - 2];
        p1 = p0 + m0;
        m1 = m0;
    } else {
        p0 = k_values[k];
        m0 = 0.5 * (k_values[k + 1] - k_values[k - 1]);
        p1 = k_values[k + 1];
        m1 = 0.5 * (k_values[k + 2] - k_values[k]);
    }

    double one_minus_t = 1.0 - t;
    return (p0 * (1.0 + 2.0 * t) + m0 * t) * one_minus_t * one_minus_t +
           (p1 * (1.0 + 2.0 * one_minus_t) - m1 * one_minus_t) * t * t;
}

static double lens_scale_radius_squared(const DK1LensConfig *lens, double radius_squared) {
    double scaled_radius_squared =
        (double)(DK1_DISTORTION_COEFFICIENT_COUNT - 1) *
        radius_squared /
        (lens->max_r * lens->max_r);
    return eval_catmull_rom10_spline(lens->k, scaled_radius_squared);
}

static double lens_distortion_radius(const DK1LensConfig *lens, double radius) {
    return radius * lens_scale_radius_squared(lens, radius * radius);
}

static double lens_inverse_distortion_radius(const DK1LensConfig *lens, double radius) {
    double delta = radius * 0.25;
    double estimate = radius * 0.25;
    double error = fabs(radius - lens_distortion_radius(lens, estimate));

    for (int i = 0; i < 20; ++i) {
        double estimate_up = estimate + delta;
        double estimate_down = estimate - delta;
        double error_up = fabs(radius - lens_distortion_radius(lens, estimate_up));
        double error_down = fabs(radius - lens_distortion_radius(lens, estimate_down));

        if (error_up < error) {
            estimate = estimate_up;
            error = error_up;
        } else if (error_down < error) {
            estimate = estimate_down;
            error = error_down;
        } else {
            delta *= 0.5;
        }
    }

    return estimate;
}

static void anchor_to_lens(const DK1DistortionAnchor *anchor, DK1LensConfig *lens) {
    memcpy(lens->k, anchor->k, sizeof(lens->k));
    lens->max_r = anchor->max_r;
    lens->chroma[0] = -0.006;
    lens->chroma[1] = 0.0;
    lens->chroma[2] = 0.014;
    lens->chroma[3] = 0.0;
}

static void build_lens_config(double eye_relief_m, DK1LensConfig *lens) {
    const DK1DistortionAnchor *lower = &DK1_DISTORTION_ANCHORS[0];
    const DK1DistortionAnchor *upper = &DK1_DISTORTION_ANCHORS[0];
    double t = 0.0;

    if (eye_relief_m <= DK1_DISTORTION_ANCHORS[0].eye_relief_m) {
        lower = &DK1_DISTORTION_ANCHORS[0];
        upper = &DK1_DISTORTION_ANCHORS[0];
    } else if (eye_relief_m < DK1_DISTORTION_ANCHORS[1].eye_relief_m) {
        lower = &DK1_DISTORTION_ANCHORS[0];
        upper = &DK1_DISTORTION_ANCHORS[1];
        t = (eye_relief_m - lower->eye_relief_m) /
            (upper->eye_relief_m - lower->eye_relief_m);
    } else if (eye_relief_m < DK1_DISTORTION_ANCHORS[2].eye_relief_m) {
        lower = &DK1_DISTORTION_ANCHORS[1];
        upper = &DK1_DISTORTION_ANCHORS[2];
        t = (eye_relief_m - lower->eye_relief_m) /
            (upper->eye_relief_m - lower->eye_relief_m);
    } else {
        lower = &DK1_DISTORTION_ANCHORS[2];
        upper = &DK1_DISTORTION_ANCHORS[2];
    }

    DK1LensConfig lower_lens;
    DK1LensConfig upper_lens;
    anchor_to_lens(lower, &lower_lens);
    anchor_to_lens(upper, &upper_lens);

    double inv_t = 1.0 - t;
    lens->max_r = inv_t * lower->max_r + t * upper->max_r;
    lens->k[0] = inv_t * lower->k[0] + t * upper->k[0];

    for (int i = 1; i < DK1_DISTORTION_COEFFICIENT_COUNT; ++i) {
        double radius_squared =
            ((double)i / (double)(DK1_DISTORTION_COEFFICIENT_COUNT - 1)) *
            lens->max_r *
            lens->max_r;
        double lower_scale = lens_scale_radius_squared(&lower_lens, radius_squared);
        double upper_scale = lens_scale_radius_squared(&upper_lens, radius_squared);
        lens->k[i] = inv_t * lower_scale + t * upper_scale;
    }

    lens->chroma[0] = -0.006;
    lens->chroma[1] = 0.0;
    lens->chroma[2] = 0.014;
    lens->chroma[3] = 0.0;
}

static double eye_relief_for_dial(int dial) {
    return DK1_BASE_EYE_RELIEF_M + ((double)dial - 5.0) * 0.001;
}

static double lens_center_x_for_eye(DK1Eye eye) {
    double visible_width_of_one_eye = 0.5 * (DK1_SCREEN_WIDTH_M - DK1_SCREEN_GAP_M);
    double center_from_left_m =
        (DK1_SCREEN_WIDTH_M - DK1_LENS_SEPARATION_M) * 0.5;
    double center_x = (center_from_left_m / visible_width_of_one_eye) * 2.0 - 1.0;
    return eye == DK1_EYE_RIGHT ? -center_x : center_x;
}

static double lens_center_y(void) {
    return (DK1_CENTER_FROM_TOP_M / DK1_SCREEN_HEIGHT_M) * 2.0 - 1.0;
}

static DK1Vector2 tan_eye_angle_scale(void) {
    return (DK1Vector2){
        0.25 * DK1_SCREEN_WIDTH_M / DK1_METERS_PER_TAN_ANGLE,
        0.5 * DK1_SCREEN_HEIGHT_M / DK1_METERS_PER_TAN_ANGLE
    };
}

static double ipd_m_for_config(const DK1Config *config) {
    return (double)config->ipd_mm * 0.001;
}

static double eye_offset_to_right_m(DK1Eye eye, double ipd_m) {
    double offset = (ipd_m - DK1_LENS_SEPARATION_M) * 0.5;
    return eye == DK1_EYE_RIGHT ? offset : -offset;
}

static double eye_offset_from_center_m(DK1Eye eye, double ipd_m) {
    double offset = ipd_m * 0.5;
    return eye == DK1_EYE_RIGHT ? offset : -offset;
}

static void chroma_scales(const DK1LensConfig *lens, double radius_squared, double scales[3]) {
    double base = lens_scale_radius_squared(lens, radius_squared);
    scales[0] = base * (1.0 + lens->chroma[0] + radius_squared * lens->chroma[1]);
    scales[1] = base;
    scales[2] = base * (1.0 + lens->chroma[2] + radius_squared * lens->chroma[3]);
}

static double vertex_shade(
    DK1Eye eye,
    DK1Vector2 screen_ndc,
    DK1Vector2 tan_eye_angles_b,
    double source_scale,
    double source_offset_x
) {
    const double fade_texture = 0.3;
    const double fade_texture_inner = 0.075;
    const double fade_screen = 0.075;
    const double fade_floor = 0.25;

    DK1Vector2 source_blue_ndc = {
        tan_eye_angles_b.x * source_scale + source_offset_x,
        tan_eye_angles_b.y * source_scale
    };
    if (eye == DK1_EYE_RIGHT) {
        source_blue_ndc.x = -source_blue_ndc.x;
    }

    double edge_fade = (1.0 / fade_texture_inner) * (1.0 - source_blue_ndc.x);
    edge_fade = min_double(edge_fade, (1.0 / fade_texture) * (1.0 + source_blue_ndc.x));
    edge_fade = min_double(edge_fade, (1.0 / fade_texture) * (1.0 - source_blue_ndc.y));
    edge_fade = min_double(edge_fade, (1.0 / fade_texture) * (1.0 + source_blue_ndc.y));

    double screen_edge = max_double(fabs(screen_ndc.x), fabs(screen_ndc.y));
    double edge_fade_screen = (1.0 / fade_screen) * (1.0 - screen_edge);
    edge_fade = min_double(edge_fade, edge_fade_screen) + fade_floor;

    return min_double(edge_fade, 1.0);
}

static int compute_mesh_counts(
    int grid_width,
    int grid_height,
    size_t *vertex_count,
    size_t *triangle_count,
    size_t *index_count
) {
    size_t width = (size_t)grid_width;
    size_t height = (size_t)grid_height;
    size_t vertex_width = width + 1;
    size_t vertex_height = height + 1;

    if (vertex_width == 0 || vertex_height == 0 ||
        vertex_width > SIZE_MAX / vertex_height) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }
    *vertex_count = vertex_width * vertex_height;
    if (*vertex_count > UINT32_MAX) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }

    if (width != 0 && height > SIZE_MAX / width) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }
    size_t cell_count = width * height;
    if (cell_count > SIZE_MAX / 2) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }
    *triangle_count = cell_count * 2;
    if (*triangle_count > SIZE_MAX / 3) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }
    *index_count = *triangle_count * 3;
    return DK1_OK;
}

static int allocate_mesh_storage(DK1DistortionMesh *mesh) {
    if (mesh->vertex_count > SIZE_MAX / sizeof(DK1DistortionMeshVertex) ||
        mesh->index_count > SIZE_MAX / sizeof(uint32_t)) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }

    DK1DistortionMeshVertex *vertices =
        calloc(mesh->vertex_count, sizeof(DK1DistortionMeshVertex));
    uint32_t *indices = calloc(mesh->index_count, sizeof(uint32_t));
    if (!vertices || !indices) {
        free(vertices);
        free(indices);
        return DK1_ERROR_IO;
    }

    mesh->vertices = vertices;
    mesh->indices = indices;
    return DK1_OK;
}

static void build_vertices(
    DK1DistortionMesh *mesh,
    DK1Eye eye,
    const DK1LensConfig *lens,
    double eye_relief_m,
    double offset_to_right_m
) {
    DK1DistortionMeshVertex *vertices = (DK1DistortionMeshVertex *)mesh->vertices;
    DK1Vector2 tan_scale = tan_eye_angle_scale();
    DK1Vector2 lens_center = {lens_center_x_for_eye(eye), lens_center_y()};
    double tan_half_fov = DK1_VISIBLE_LENS_RADIUS_M / eye_relief_m;
    double source_scale = 1.0 / tan_half_fov;
    double source_offset_x = offset_to_right_m / DK1_VISIBLE_LENS_RADIUS_M;
    double tan_eye_angle_offset_x = offset_to_right_m / eye_relief_m;
    double eye_offset = eye == DK1_EYE_RIGHT ? 1.0 : 0.0;

    for (int y = 0; y <= mesh->grid_height; ++y) {
        for (int x = 0; x <= mesh->grid_width; ++x) {
            size_t vertex_index =
                (size_t)y * ((size_t)mesh->grid_width + 1) + (size_t)x;

            DK1Vector2 source_ndc = {
                2.0 * ((double)x / (double)mesh->grid_width) - 1.0,
                2.0 * ((double)y / (double)mesh->grid_height) - 1.0
            };
            DK1Vector2 tan_eye_angle = {
                source_ndc.x * tan_half_fov - tan_eye_angle_offset_x,
                source_ndc.y * tan_half_fov
            };

            double tan_radius = vec2_length(tan_eye_angle);
            double distorted_radius = lens_inverse_distortion_radius(lens, tan_radius);
            DK1Vector2 tan_eye_angle_distorted = {0.0, 0.0};
            if (tan_radius > 0.0) {
                tan_eye_angle_distorted =
                    vec2_scale(tan_eye_angle, distorted_radius / tan_radius);
            }

            DK1Vector2 screen_ndc = {
                tan_eye_angle_distorted.x / tan_scale.x + lens_center.x,
                tan_eye_angle_distorted.y / tan_scale.y + lens_center.y
            };
            screen_ndc.x = clamp_double(screen_ndc.x, -1.0, 1.0);
            screen_ndc.y = clamp_double(screen_ndc.y, -1.0, 1.0);

            DK1Vector2 screen_tan = {
                (screen_ndc.x - lens_center.x) * tan_scale.x,
                (screen_ndc.y - lens_center.y) * tan_scale.y
            };
            double radius_squared =
                screen_tan.x * screen_tan.x + screen_tan.y * screen_tan.y;
            double scales[3];
            chroma_scales(lens, radius_squared, scales);

            vertices[vertex_index].tan_eye_angles_r = vec2_scale(screen_tan, scales[0]);
            vertices[vertex_index].tan_eye_angles_g = vec2_scale(screen_tan, scales[1]);
            vertices[vertex_index].tan_eye_angles_b = vec2_scale(screen_tan, scales[2]);
            vertices[vertex_index].screen_pos_ndc = (DK1Vector2){
                0.5 * screen_ndc.x - 0.5 + eye_offset,
                -screen_ndc.y
            };
            vertices[vertex_index].timewarp_lerp = 0.0;
            vertices[vertex_index].shade = vertex_shade(
                eye,
                screen_ndc,
                vertices[vertex_index].tan_eye_angles_b,
                source_scale,
                source_offset_x
            );
        }
    }
}

static void build_indices(DK1DistortionMesh *mesh) {
    uint32_t *indices = (uint32_t *)mesh->indices;
    size_t cursor = 0;
    uint32_t stride = (uint32_t)mesh->grid_width + 1;
    int half_width = mesh->grid_width / 2;
    int half_height = mesh->grid_height / 2;

    for (int y = 0; y < mesh->grid_height; ++y) {
        for (int x = 0; x < mesh->grid_width; ++x) {
            uint32_t a = (uint32_t)y * stride + (uint32_t)x;
            uint32_t b = a + 1;
            uint32_t c = a + stride;
            uint32_t d = c + 1;

            if ((x < half_width) != (y < half_height)) {
                indices[cursor++] = a;
                indices[cursor++] = b;
                indices[cursor++] = d;
                indices[cursor++] = d;
                indices[cursor++] = c;
                indices[cursor++] = a;
            } else {
                indices[cursor++] = a;
                indices[cursor++] = b;
                indices[cursor++] = c;
                indices[cursor++] = b;
                indices[cursor++] = d;
                indices[cursor++] = c;
            }
        }
    }
}

int dk1_config_make_eye_projection(
    const DK1Config *config,
    DK1Eye eye,
    DK1EyeProjection *out_projection
) {
    if (!config || !out_projection) return DK1_ERROR_INVALID_ARGUMENT;
    if (eye != DK1_EYE_LEFT && eye != DK1_EYE_RIGHT) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }

    int validation = dk1_config_validate(config);
    if (validation != DK1_OK) return validation;

    int dial = eye == DK1_EYE_RIGHT ? config->right_dial : config->left_dial;
    double eye_relief_m = eye_relief_for_dial(dial);
    double tan_half_fov = DK1_VISIBLE_LENS_RADIUS_M / eye_relief_m;
    double ipd_m = ipd_m_for_config(config);
    double offset_to_right_m = eye_offset_to_right_m(eye, ipd_m);
    double tan_eye_angle_offset_x = offset_to_right_m / eye_relief_m;

    out_projection->left_tan = -tan_half_fov - tan_eye_angle_offset_x;
    out_projection->right_tan = tan_half_fov - tan_eye_angle_offset_x;
    out_projection->top_tan = tan_half_fov;
    out_projection->bottom_tan = -tan_half_fov;
    out_projection->eye_offset_m = eye_offset_from_center_m(eye, ipd_m);
    return DK1_OK;
}

int dk1_distortion_mesh_build(
    DK1DistortionMesh *mesh,
    DK1Eye eye,
    const DK1Config *config
) {
    if (!mesh || !config) return DK1_ERROR_INVALID_ARGUMENT;
    if (eye != DK1_EYE_LEFT && eye != DK1_EYE_RIGHT) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }
    if (config->grid_width <= 0 || config->grid_height <= 0 || config->ipd_mm <= 0) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }

    DK1DistortionMesh result;
    memset(&result, 0, sizeof(result));
    result.grid_width = config->grid_width;
    result.grid_height = config->grid_height;

    int count_result = compute_mesh_counts(
        result.grid_width,
        result.grid_height,
        &result.vertex_count,
        &result.triangle_count,
        &result.index_count
    );
    if (count_result != DK1_OK) {
        return count_result;
    }

    int allocation_result = allocate_mesh_storage(&result);
    if (allocation_result != DK1_OK) {
        return allocation_result;
    }

    int dial = eye == DK1_EYE_RIGHT ? config->right_dial : config->left_dial;
    double eye_relief_m = eye_relief_for_dial(dial);
    double ipd_m = ipd_m_for_config(config);
    double offset_to_right_m = eye_offset_to_right_m(eye, ipd_m);
    DK1LensConfig lens;
    build_lens_config(eye_relief_m, &lens);

    build_vertices(&result, eye, &lens, eye_relief_m, offset_to_right_m);
    build_indices(&result);

    dk1_distortion_mesh_destroy(mesh);
    *mesh = result;
    return DK1_OK;
}

void dk1_distortion_mesh_destroy(DK1DistortionMesh *mesh) {
    if (!mesh) return;
    free((void *)mesh->vertices);
    free((void *)mesh->indices);
    memset(mesh, 0, sizeof(*mesh));
}
