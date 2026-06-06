#ifndef DK1_ESTIMATOR_H
#define DK1_ESTIMATOR_H

#include "DK1Tracker/DK1Types.h"
#include "DK1HeadModel.h"
#include <stdint.h>

typedef struct {
    DK1Quaternion orientation;
    DK1TrackerState state;
    DK1HeadModel head_model;
    DK1Vector3 gyro_bias;
    DK1MagCalibration mag_calibration;
    DK1Vector3 previous_unbiased_gyro;
    uint16_t last_report_timestamp;
    double last_report_sample_dt_s;
    double last_mag_correction_time_s;
    int have_previous_sample;
} DK1Estimator;

void dk1_estimator_init(DK1Estimator *est);
void dk1_estimator_update(DK1Estimator *est, const DK1Sample *sample);
void dk1_estimator_get_state(const DK1Estimator *est, DK1TrackerState *out_state);
void dk1_estimator_set_gyro_bias(DK1Estimator *est, DK1Vector3 bias);
int dk1_estimator_set_mag_calibration(
    DK1Estimator *est,
    const DK1MagCalibration *calibration
);

#endif
