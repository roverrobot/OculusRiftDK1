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

typedef struct DK1MagCalibration {
    DK1Vector3 hard_iron;
    int axis_order[3];
    DK1Vector3 axis_signs;
    double correction_rate;
    uint32_t correction_interval_samples;
} DK1MagCalibration;

typedef struct DK1TrackerState {
    int initialized;
    uint64_t sample_index;
    uint16_t device_timestamp;
    double time_s;
    double dt_s;

    DK1Quaternion orientation;
    DK1Vector3 unbiased_gyro;
    DK1Vector3 angular_accel;

    DK1Vector3 look_dir_world;
    DK1Vector3 eye_center_world;
    DK1Vector3 left_eye_world;
    DK1Vector3 right_eye_world;

    DK1Vector3 predicted_accel;
    DK1Vector3 accel_residual;
    DK1Vector3 pivot_accel_body;
    DK1Vector3 pivot_accel_world;

    DK1Vector3 mag_calibrated;
    DK1Vector3 expected_north_world;
    DK1Vector3 observed_north_world;
    int expected_north_initialized;
    uint32_t north_window_sample_count;
    double heading_residual_deg;
    double mag_correction_rate;
    uint32_t mag_correction_interval_samples;
    uint64_t mag_correction_update_count;
} DK1TrackerState;

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
