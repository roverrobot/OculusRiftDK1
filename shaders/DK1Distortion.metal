#include <metal_stdlib>

using namespace metal;

struct DK1DistortionVertexIn {
    packed_float2 screen_pos_ndc;
    float timewarp_lerp;
    float shade;
    packed_float2 tan_eye_angles_r;
    packed_float2 tan_eye_angles_g;
    packed_float2 tan_eye_angles_b;
};

struct DK1DistortionUniforms {
    packed_float2 source_uv_scale;
    packed_float2 source_uv_offset;
};

struct DK1DistortionRasterData {
    float4 position [[position]];
    float shade;
    float2 uv_r;
    float2 uv_g;
    float2 uv_b;
};

static inline float2 dk1_unpack_float2(packed_float2 value) {
    return float2(value.x, value.y);
}

vertex DK1DistortionRasterData dk1_distortion_vertex(
    const device DK1DistortionVertexIn *vertices [[buffer(0)]],
    constant DK1DistortionUniforms &uniforms [[buffer(1)]],
    uint vertex_id [[vertex_id]]
) {
    DK1DistortionVertexIn vertex = vertices[vertex_id];
    float2 uv_scale = dk1_unpack_float2(uniforms.source_uv_scale);
    float2 uv_offset = dk1_unpack_float2(uniforms.source_uv_offset);

    DK1DistortionRasterData out;
    out.position = float4(dk1_unpack_float2(vertex.screen_pos_ndc), 0.0, 1.0);
    out.shade = vertex.shade;
    out.uv_r = dk1_unpack_float2(vertex.tan_eye_angles_r) * uv_scale + uv_offset;
    out.uv_g = dk1_unpack_float2(vertex.tan_eye_angles_g) * uv_scale + uv_offset;
    out.uv_b = dk1_unpack_float2(vertex.tan_eye_angles_b) * uv_scale + uv_offset;
    return out;
}

fragment float4 dk1_distortion_fragment(
    DK1DistortionRasterData raster [[stage_in]],
    texture2d<float> source_texture [[texture(0)]]
) {
    constexpr sampler source_sampler(
        coord::normalized,
        address::clamp_to_edge,
        filter::linear
    );

    float r = source_texture.sample(source_sampler, raster.uv_r).r;
    float g = source_texture.sample(source_sampler, raster.uv_g).g;
    float b = source_texture.sample(source_sampler, raster.uv_b).b;
    float shade = saturate(raster.shade);
    return float4(float3(r, g, b) * shade, 1.0);
}
