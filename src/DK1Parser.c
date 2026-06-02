#include "DK1Parser.h"
#include "DK1Tracker/DK1Error.h"
#include <string.h>

static uint32_t read_bits_le(const uint8_t *p, int bit_offset, int bit_count) {
    uint64_t val = 0;
    for (int i = 0; i < 8; ++i) {
        val |= (uint64_t)p[bit_offset / 8 + i] << (8 * i);
    }
    // This is a stub implementation; real bit-packing varies.
    // Typically would involve shifting and masking based on the specific bit range.
    return (uint32_t)((val >> (bit_offset % 8)) & ((1ULL << bit_count) - 1));
}

static int32_t sign_extend_21(uint32_t v) {
    if (v & (1U << 20)) {
        return (int32_t)(v | 0xFFFFF00000); // This is a simplification
    }
    return (int32_t)v;
}

static void parse_motion_sample_16bytes(const uint8_t *p, DK1Sample *s) {
    // TODO: Implement exact bit unpacking for accel/gyro (21-bit packed)
    // Currently just stubs
    s->accel = (DK1Vector3){0, 0, 0};
    s->gyro = (DK1Vector3){0, 0, 0};
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

    // macOS HID often prefixes with Report ID
    const uint8_t *p = data;
    if (data[0] == 1 && length >= 63) {
        // Likely Report ID included
    } else if (data[0] != 1) {
        // If it's not 1, we might be missing the ID or it's wrong
        // but for this skeleton we assume report 1
    }

    uint8_t report_id = p[0];
    if (report_id != 1) return DK1_ERROR_PARSE;

    uint8_t sample_count = p[1];
    uint16_t timestamp = p[2] | (p[3] << 8);
    uint8_t last_cmd = p[4];
    uint8_t temp_raw = p[5];

    *out_count = 0;
    for (int i = 0; i < sample_count && i < max_samples; ++i) {
        DK1Sample *s = &samples[i];
        s->timestamp = timestamp;
        s->sample_count = sample_count;
        s->temperature_c = (double)temp_raw / 100.0; // Centidegrees stub

        // Mag is usually at the end of the report
        // For the skeleton, we'll just point to a guess area
        s->mag = (DK1Vector3){(double)p[58], (double)p[59], (double)p[60]};

        // Unpack motion data (each sample is roughly 16 bytes)
        parse_motion_sample_16bytes(&p[6 + i * 16], s);

        (*out_count)++;
    }

    return DK1_OK;
}
