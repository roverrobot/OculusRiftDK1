#include "DK1Parser.h"
#include "DK1Tracker/DK1Error.h"

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

static int failures = 0;

static void check_int_equal(const char *name, int actual, int expected) {
    if (actual != expected) {
        fprintf(stderr, "%s: expected %d, got %d\n", name, expected, actual);
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

static void write_u16_le(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)(value & 0xff);
    p[1] = (uint8_t)((value >> 8) & 0xff);
}

static uint32_t encode_i21(int32_t value) {
    return (uint32_t)value & 0x1fffffu;
}

static void write_packed_vec3_21(
    uint8_t *b,
    int32_t x_value,
    int32_t y_value,
    int32_t z_value
) {
    uint32_t x = encode_i21(x_value);
    uint32_t y = encode_i21(y_value);
    uint32_t z = encode_i21(z_value);

    b[0] = (uint8_t)((x >> 13) & 0xff);
    b[1] = (uint8_t)((x >> 5) & 0xff);
    b[2] = (uint8_t)(((x & 0x1f) << 3) | ((y >> 18) & 0x07));
    b[3] = (uint8_t)((y >> 10) & 0xff);
    b[4] = (uint8_t)((y >> 2) & 0xff);
    b[5] = (uint8_t)(((y & 0x03) << 6) | ((z >> 15) & 0x3f));
    b[6] = (uint8_t)((z >> 7) & 0xff);
    b[7] = (uint8_t)((z & 0x7f) << 1);
}

static void test_report_frame_to_model_frame(void) {
    uint8_t report[62];
    DK1Sample samples[3];
    size_t count = 0;

    memset(report, 0, sizeof(report));
    memset(samples, 0, sizeof(samples));

    report[0] = 1;
    report[1] = 1;
    write_u16_le(report + 2, 1234);
    write_u16_le(report + 6, 2512);
    write_packed_vec3_21(report + 8, 10000, -20000, 30000);
    write_packed_vec3_21(report + 16, -40000, 50000, -60000);
    write_u16_le(report + 56, 1234);
    write_u16_le(report + 58, (uint16_t)-2345);
    write_u16_le(report + 60, 3456);

    int result = dk1_parse_input_report(
        report,
        sizeof(report),
        samples,
        3,
        &count
    );

    check_int_equal("parse_result", result, DK1_OK);
    check_int_equal("sample_count", (int)count, 1);

    check_near("timestamp", samples[0].timestamp, 1234.0, 0.0);
    check_near("temperature", samples[0].temperature_c, 25.12, 1e-12);

    check_near("accel.x", samples[0].accel.x, 1.0, 1e-12);
    check_near("accel.y", samples[0].accel.y, -2.0, 1e-12);
    check_near("accel.z", samples[0].accel.z, 3.0, 1e-12);
    check_near("gyro.x", samples[0].gyro.x, -4.0, 1e-12);
    check_near("gyro.y", samples[0].gyro.y, 5.0, 1e-12);
    check_near("gyro.z", samples[0].gyro.z, -6.0, 1e-12);

    check_near("mag.x", samples[0].mag.x, -1234.0, 1e-12);
    check_near("mag.y", samples[0].mag.y, 2345.0, 1e-12);
    check_near("mag.z", samples[0].mag.z, -3456.0, 1e-12);
}

int main(void) {
    test_report_frame_to_model_frame();

    if (failures != 0) {
        fprintf(stderr, "%d parser test failure(s)\n", failures);
        return 1;
    }

    printf("parser tests passed\n");
    return 0;
}
