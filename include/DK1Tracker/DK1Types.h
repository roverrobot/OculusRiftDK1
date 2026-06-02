#ifndef DK1_TYPES_H
#define DK1_TYPES_H

#include <stdint.h>

typedef struct DK1Vector3 {
    double x;
    double y;
    double z;
} DK1Vector3;

typedef struct DK1Quaternion {
    double w;
    double x;
    double y;
    double z;
} DK1Quaternion;

typedef struct DK1Sample {
    uint16_t timestamp;
    uint8_t sample_count;

    DK1Vector3 accel;   // m/s^2
    DK1Vector3 gyro;    // rad/s
    DK1Vector3 mag;     // raw or calibrated magnetometer
    double temperature_c;
} DK1Sample;

typedef void (*DK1SampleCallback)(
    const DK1Sample *sample,
    void *user_data
);

#endif // DK1_TYPES_H
