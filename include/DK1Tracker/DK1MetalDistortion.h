#ifndef DK1_METAL_DISTORTION_H
#define DK1_METAL_DISTORTION_H

#include "DK1Types.h"
#include <stddef.h>

typedef enum DK1MetalSourceOrigin {
    DK1_METAL_SOURCE_ORIGIN_TOP_LEFT = 0,
    DK1_METAL_SOURCE_ORIGIN_BOTTOM_LEFT = 1
} DK1MetalSourceOrigin;

typedef struct DK1MetalDistortionVertex {
    float screen_pos_ndc[2];
    float timewarp_lerp;
    float shade;
    float tan_eye_angles_r[2];
    float tan_eye_angles_g[2];
    float tan_eye_angles_b[2];
} DK1MetalDistortionVertex;

typedef struct DK1MetalDistortionUniforms {
    float source_uv_scale[2];
    float source_uv_offset[2];
} DK1MetalDistortionUniforms;

int dk1_metal_distortion_copy_vertices(
    const DK1DistortionMesh *mesh,
    DK1MetalDistortionVertex *out_vertices,
    size_t out_vertex_count
);

int dk1_metal_distortion_make_eye_texture_uniforms(
    const DK1Config *config,
    DK1Eye eye,
    DK1MetalSourceOrigin source_origin,
    DK1MetalDistortionUniforms *out_uniforms
);

#endif // DK1_METAL_DISTORTION_H
