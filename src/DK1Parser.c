#include "DK1Parser.h"
#include "DK1Tracker/DK1Error.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

// Read an unsigned value from a little‑endian bit stream.
// bit_offset is the offset of the least‑significant bit.
// Helper to read little‑endian unsigned 16‑bit.
static uint16_t read_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// Helper to read little‑endian signed 16‑bit.
static int16_t read_i16_le(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_bits_le(const uint8_t *p, int bit_offset, int bit_count) {
    uint32_t val = 0;
    for (int i = 0; i < bit_count; ++i) {
        int byte_idx = (bit_offset + i) / 8;
        int bit_idx = (bit_offset + i) % 8;
        val |= ((uint32_t)(p[byte_idx] >> bit_idx) & 1u) << i;
    }
    return val;
}

// Sign‑extend a 21‑bit two’s‑complement value to 32 bits.
static int32_t sign_extend_21(uint32_t v) {
    v &= 0x1FFFFF;
    if (v & 0x100000) {
        v |= 0xFFE00000u;
    }
    return (int32_t)v;
}

static void decode_vec3_21(const uint8_t *b, DK1Vector3 *out, double scale) {
    uint32_t x =
        ((uint32_t)b[0] << 13) |
        ((uint32_t)b[1] << 5)  |
        ((uint32_t)b[2] >> 3);

    uint32_t y =
        (((uint32_t)b[2] & 0x07) << 18) |
        ((uint32_t)b[3] << 10) |
        ((uint32_t)b[4] << 2)  |
        ((uint32_t)b[5] >> 6);

    uint32_t z =
        (((uint32_t)b[5] & 0x3F) << 15) |
        ((uint32_t)b[6] << 7) |
        ((uint32_t)b[7] >> 1);

    out->x = sign_extend_21(x) * scale;
    out->y = sign_extend_21(y) * scale;
    out->z = sign_extend_21(z) * scale;
}

// Parse a 16‑byte motion sample block containing 6 × 21‑bit values.
// The layout is assumed to be: accel X, Y, Z followed by gyro X, Y, Z.
static void parse_motion_sample_16bytes(
    const uint8_t *p,
    DK1Vector3 *accel,
    DK1Vector3 *gyro
) {
    const double scale = 1.0 / 10000.0;

    decode_vec3_21(p,     accel, scale);  // bytes 0..7
    decode_vec3_21(p + 8, gyro,  scale);  // bytes 8..15
}

int dk1_parse_input_report(
    const uint8_t *data,
    size_t length,
    DK1Sample *samples,
    size_t max_samples,
    size_t *out_count
) {
    if (!data || !samples || !out_count) return DK1_ERROR_INVALID_ARGUMENT;
    if (length < 62) return DK1_ERROR_PARSE;

    /* Expect first byte to be report ID (1) */
    if (data[0] != 1) return DK1_ERROR_PARSE;
    const uint8_t *p = data;
    uint8_t report_id = p[0];
    if (report_id != 1) return DK1_ERROR_PARSE;

    uint8_t sample_count = p[1];
    uint16_t timestamp = read_u16_le(p + 2);
    uint16_t last_cmd = read_u16_le(p + 4);
    int16_t temp_raw = (int16_t)read_i16_le(p + 6);
    (void)last_cmd; /* unused */

    *out_count = 0;
    size_t parsed = 0;
    for (size_t i = 0; i < sample_count && parsed < max_samples && parsed < 3; ++i, ++parsed) {
        DK1Sample *s = &samples[parsed];
        s->timestamp = timestamp;
        s->sample_count = sample_count;
        s->temperature_c = temp_raw / 100.0;
        /* Magnetometer */
        s->mag.x = read_i16_le(p + 56) / 1.0;
        s->mag.y = read_i16_le(p + 58) / 1.0;
        s->mag.z = read_i16_le(p + 60) / 1.0;
        /* Motion sample */
        parse_motion_sample_16bytes(p + 8 + i * 16, &s->accel, &s->gyro);
        (*out_count)++;
    }
    return DK1_OK;
}
