#include "DK1Tracker/DK1MetalDistortion.h"
#include "DK1Config.h"
#include "DK1Tracker/DK1Error.h"

#define DK1_LENS_SEPARATION_M 0.0635
#define DK1_VISIBLE_LENS_RADIUS_M 0.0175
#define DK1_BASE_EYE_RELIEF_M 0.012760465

static double eye_relief_for_dial(int dial) {
    return DK1_BASE_EYE_RELIEF_M + ((double)dial - 5.0) * 0.001;
}

static double eye_offset_to_right_m(DK1Eye eye, double ipd_m) {
    double offset = (ipd_m - DK1_LENS_SEPARATION_M) * 0.5;
    return eye == DK1_EYE_RIGHT ? offset : -offset;
}

int dk1_metal_distortion_copy_vertices(
    const DK1DistortionMesh *mesh,
    DK1MetalDistortionVertex *out_vertices,
    size_t out_vertex_count
) {
    if (!mesh || !out_vertices) return DK1_ERROR_INVALID_ARGUMENT;
    if (!mesh->vertices || out_vertex_count < mesh->vertex_count) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }

    for (size_t i = 0; i < mesh->vertex_count; ++i) {
        const DK1DistortionMeshVertex *src = &mesh->vertices[i];
        DK1MetalDistortionVertex *dst = &out_vertices[i];

        dst->screen_pos_ndc[0] = (float)src->screen_pos_ndc.x;
        dst->screen_pos_ndc[1] = (float)src->screen_pos_ndc.y;
        dst->timewarp_lerp = (float)src->timewarp_lerp;
        dst->shade = (float)src->shade;
        dst->tan_eye_angles_r[0] = (float)src->tan_eye_angles_r.x;
        dst->tan_eye_angles_r[1] = (float)src->tan_eye_angles_r.y;
        dst->tan_eye_angles_g[0] = (float)src->tan_eye_angles_g.x;
        dst->tan_eye_angles_g[1] = (float)src->tan_eye_angles_g.y;
        dst->tan_eye_angles_b[0] = (float)src->tan_eye_angles_b.x;
        dst->tan_eye_angles_b[1] = (float)src->tan_eye_angles_b.y;
    }

    return DK1_OK;
}

int dk1_metal_distortion_make_eye_texture_uniforms(
    const DK1Config *config,
    DK1Eye eye,
    DK1MetalSourceOrigin source_origin,
    DK1MetalDistortionUniforms *out_uniforms
) {
    if (!config || !out_uniforms) return DK1_ERROR_INVALID_ARGUMENT;
    if (eye != DK1_EYE_LEFT && eye != DK1_EYE_RIGHT) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }
    if (source_origin != DK1_METAL_SOURCE_ORIGIN_TOP_LEFT &&
        source_origin != DK1_METAL_SOURCE_ORIGIN_BOTTOM_LEFT) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }

    int validation = dk1_config_validate(config);
    if (validation != DK1_OK) return validation;

    int dial = eye == DK1_EYE_RIGHT ? config->right_dial : config->left_dial;
    double eye_relief_m = eye_relief_for_dial(dial);
    double tan_half_fov = DK1_VISIBLE_LENS_RADIUS_M / eye_relief_m;
    double source_ndc_scale = 1.0 / tan_half_fov;
    double ipd_m = (double)config->ipd_mm * 0.001;
    double source_ndc_offset_x =
        eye_offset_to_right_m(eye, ipd_m) / DK1_VISIBLE_LENS_RADIUS_M;

    out_uniforms->source_uv_scale[0] = (float)(0.5 * source_ndc_scale);
    out_uniforms->source_uv_offset[0] = (float)(0.5 * source_ndc_offset_x + 0.5);
    if (source_origin == DK1_METAL_SOURCE_ORIGIN_TOP_LEFT) {
        out_uniforms->source_uv_scale[1] = (float)(-0.5 * source_ndc_scale);
    } else {
        out_uniforms->source_uv_scale[1] = (float)(0.5 * source_ndc_scale);
    }
    out_uniforms->source_uv_offset[1] = 0.5f;

    return DK1_OK;
}
