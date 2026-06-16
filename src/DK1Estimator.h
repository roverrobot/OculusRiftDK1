#ifndef DK1_ESTIMATOR_H
#define DK1_ESTIMATOR_H

#include "DK1Tracker/DK1Types.h"
#include "DK1HeadModel.h"
#include <stdint.h>

enum {
    DK1_ESTIMATOR_MAX_NORTH_WINDOW_SAMPLES = 128
};

typedef struct {
    DK1Quaternion orientation;
    DK1TrackerState state;
    DK1HeadModel head_model;
    DK1HeadNeckConfig head_neck_config;
    double eye_height_m;
    DK1Vector3 gyro_bias;
    DK1MagCalibration mag_calibration;
    DK1Vector3 previous_unbiased_gyro;
    DK1Vector3 previous_pivot_accel_world;
    DK1Vector3 pivot_velocity_world;
    DK1Vector3 pivot_position_world;
    uint16_t last_report_timestamp;
    uint16_t timing_last_group_timestamp;
    double last_report_sample_dt_s;
    double last_report_raw_dt_s;
    double max_raw_dt_s;
    DK1Vector3 north_window[DK1_ESTIMATOR_MAX_NORTH_WINDOW_SAMPLES];
    DK1Vector3 north_window_sum;
    uint32_t north_window_index;
    uint32_t north_window_count;
    int have_previous_sample;
    int have_previous_pivot_accel;
    int timing_have_group;
    int current_report_pivot_skip;
} DK1Estimator;

void dk1_estimator_init(DK1Estimator *est);
void dk1_estimator_update(DK1Estimator *est, const DK1Sample *sample);
void dk1_estimator_get_state(const DK1Estimator *est, DK1TrackerState *out_state);
int dk1_estimator_set_eye_height(DK1Estimator *est, double eye_height_m);
void dk1_estimator_set_gyro_bias(DK1Estimator *est, DK1Vector3 bias);
int dk1_estimator_set_mag_calibration(
    DK1Estimator *est,
    const DK1MagCalibration *calibration
);
int dk1_estimator_set_head_neck_config(
    DK1Estimator *est,
    const DK1HeadNeckConfig *config
);
void dk1_estimator_get_head_neck_config(
    const DK1Estimator *est,
    DK1HeadNeckConfig *out_config
);

#endif
