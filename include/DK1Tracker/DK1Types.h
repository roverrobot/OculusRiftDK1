#ifndef DK1_TYPES_H
#define DK1_TYPES_H

#include <stddef.h>
#include <stdint.h>

typedef enum DK1Eye {
    DK1_EYE_LEFT = 0,
    DK1_EYE_RIGHT = 1
} DK1Eye;

typedef struct DK1Vector2 {
    double x;
    double y;
} DK1Vector2;

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

/* Gamma head/neck geometry and pivot-integration settings.
 * The neck-to-tracker vector is (0, h_m, -ell_m) in the body frame. */
typedef struct DK1HeadNeckConfig {
    double h_m;
    double ell_m;
    double ipd_m;
    double pivot_damping_per_second;
    double max_dt_s;
    uint8_t max_report_sample_count;
} DK1HeadNeckConfig;

#define DK1_DEFAULT_HEAD_NECK_H_M 0.10
#define DK1_DEFAULT_HEAD_NECK_ELL_M 0.16
#define DK1_DEFAULT_HEAD_NECK_PIVOT_DAMPING_PER_SECOND 2.0
#define DK1_DEFAULT_HEAD_NECK_MAX_DT_S 0.02
#define DK1_DEFAULT_HEAD_NECK_MAX_REPORT_SAMPLE_COUNT 3u

enum {
    DK1_DEFAULT_LEFT_DIAL = 5,
    DK1_DEFAULT_RIGHT_DIAL = 5,
    DK1_DEFAULT_GRID_WIDTH = 64,
    DK1_DEFAULT_GRID_HEIGHT = 64,
    DK1_DEFAULT_IPD_MM = 64
};

typedef struct DK1Config {
    int left_dial;
    int right_dial;
    int grid_width;
    int grid_height;
    int ipd_mm;
    DK1Vector3 gyro_bias;
    DK1HeadNeckConfig head_neck;
} DK1Config;

typedef struct DK1DistortionMeshVertex {
    DK1Vector2 screen_pos_ndc;
    double timewarp_lerp;
    double shade;
    DK1Vector2 tan_eye_angles_r;
    DK1Vector2 tan_eye_angles_g;
    DK1Vector2 tan_eye_angles_b;
} DK1DistortionMeshVertex;

typedef struct DK1DistortionMesh {
    int grid_width;
    int grid_height;
    size_t vertex_count;
    size_t triangle_count;
    size_t index_count;
    const DK1DistortionMeshVertex *vertices;
    const uint32_t *indices;
} DK1DistortionMesh;

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
    /* Diagnostic neck-pivot translation inferred by double-integrating
     * pivot_accel_world. This is sensitive to timing quality. */
    DK1Vector3 pivot_velocity_world;
    DK1Vector3 pivot_position_world;
    int pivot_position_reliable;

    DK1Vector3 mag_calibrated;
    DK1Vector3 expected_north_world;
    DK1Vector3 observed_north_world;
    int expected_north_initialized;
    uint32_t north_window_sample_count;
    double heading_residual_deg;
    double mag_correction_rate;
    uint32_t mag_correction_interval_samples;
    uint64_t mag_correction_update_count;

    /* Timing-quality diagnostics for pivot inference and report grouping. */
    double pivot_damping_per_second;
    double timing_max_dt_s;
    double timing_last_raw_dt_s;
    double timing_max_raw_dt_s;
    uint8_t timing_last_report_sample_count;
    uint8_t timing_max_report_sample_count;
    uint64_t timing_report_group_count;
    uint64_t timing_repeated_group_count;
    uint64_t timing_oversized_group_count;
    uint64_t timing_capped_dt_count;
    uint64_t timing_nonpositive_dt_count;
    uint64_t pivot_integration_skipped_count;
    uint64_t report_queue_dropped_count;
} DK1TrackerState;

typedef struct DK1PredictedState {
    DK1Vector3 look_dir_world;
    DK1Vector3 left_eye_world;
    DK1Vector3 right_eye_world;
} DK1PredictedState;

typedef struct DK1SCNCameraEyeSettings {
    DK1Vector3 position_world;
    DK1Vector3 right_world;
    DK1Vector3 up_world;
    DK1Vector3 forward_world;
    double node_transform_column_major[16];
    double view_transform_column_major[16];
} DK1SCNCameraEyeSettings;

typedef struct DK1SCNCameraSettings {
    DK1SCNCameraEyeSettings eyes[2];
} DK1SCNCameraSettings;

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
