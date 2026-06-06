#include "DK1Estimator.h"
#include "DK1Tracker/DK1Error.h"

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int failures = 0;

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

static void check_int_equal(const char *name, uint64_t actual, uint64_t expected) {
    if (actual != expected) {
        fprintf(
            stderr,
            "%s: expected %llu, got %llu\n",
            name,
            (unsigned long long)expected,
            (unsigned long long)actual
        );
        failures++;
    }
}

static void check_vec3_near(
    const char *name,
    DK1Vector3 actual,
    double x,
    double y,
    double z,
    double tolerance
) {
    char component_name[128];
    snprintf(component_name, sizeof(component_name), "%s.x", name);
    check_near(component_name, actual.x, x, tolerance);
    snprintf(component_name, sizeof(component_name), "%s.y", name);
    check_near(component_name, actual.y, y, tolerance);
    snprintf(component_name, sizeof(component_name), "%s.z", name);
    check_near(component_name, actual.z, z, tolerance);
}

static DK1Sample stationary_sample(uint16_t timestamp) {
    DK1Sample sample;
    sample.timestamp = timestamp;
    sample.sample_count = 1;
    sample.accel = (DK1Vector3){0.0, 9.80665, 0.0};
    sample.gyro = (DK1Vector3){0.0, 0.0, 0.0};
    sample.mag = (DK1Vector3){1.0, 0.0, 0.0};
    sample.temperature_c = 25.0;
    return sample;
}

static void test_initial_orientation_and_pose(void) {
    DK1Estimator estimator;
    DK1TrackerState state;
    DK1Sample sample = stationary_sample(10);

    dk1_estimator_init(&estimator);
    dk1_estimator_update(&estimator, &sample);
    dk1_estimator_get_state(&estimator, &state);

    check_int_equal("initialized", (uint64_t)state.initialized, 1);
    check_int_equal("sample_index", state.sample_index, 1);
    check_near("orientation.w", state.orientation.w, 1.0, 1e-12);
    check_near("orientation.x", state.orientation.x, 0.0, 1e-12);
    check_near("orientation.y", state.orientation.y, 0.0, 1e-12);
    check_near("orientation.z", state.orientation.z, 0.0, 1e-12);
    check_vec3_near("predicted_accel", state.predicted_accel, 0.0, 9.80665, 0.0, 1e-9);
    check_vec3_near("accel_residual", state.accel_residual, 0.0, 0.0, 0.0, 1e-9);
    check_vec3_near("look_dir_world", state.look_dir_world, 0.0, 0.0, -1.0, 1e-12);
    check_vec3_near("eye_center_world", state.eye_center_world, 0.0, 0.10, -0.16, 1e-12);
    check_vec3_near("expected_north_world", state.expected_north_world, 1.0, 0.0, 0.0, 1e-12);
}

static void test_gyro_integration_uses_trapezoidal_step(void) {
    DK1Estimator estimator;
    DK1TrackerState state;
    DK1Sample sample = stationary_sample(0);
    double rate = M_PI * 0.5;
    double s = sqrt(0.5);

    sample.gyro = (DK1Vector3){0.0, 0.0, rate};

    dk1_estimator_init(&estimator);
    dk1_estimator_update(&estimator, &sample);

    for (uint16_t i = 1; i <= 1000; ++i) {
        sample.timestamp = i;
        dk1_estimator_update(&estimator, &sample);
    }
    dk1_estimator_get_state(&estimator, &state);

    check_near("gyro_integration.w", state.orientation.w, s, 1e-9);
    check_near("gyro_integration.x", state.orientation.x, 0.0, 1e-9);
    check_near("gyro_integration.y", state.orientation.y, 0.0, 1e-9);
    check_near("gyro_integration.z", state.orientation.z, s, 1e-9);
    check_near("gyro_integration.time_s", state.time_s, 1.0, 1e-12);
}

static void test_report_timestamp_expansion(void) {
    DK1Estimator estimator;
    DK1TrackerState state;
    DK1Sample sample = stationary_sample(100);

    sample.sample_count = 2;

    dk1_estimator_init(&estimator);
    dk1_estimator_update(&estimator, &sample);

    dk1_estimator_update(&estimator, &sample);
    dk1_estimator_get_state(&estimator, &state);
    check_near("same_report_dt", state.dt_s, 0.001, 1e-12);
    check_near("same_report_time", state.time_s, 0.001, 1e-12);

    sample.timestamp = 102;
    dk1_estimator_update(&estimator, &sample);
    dk1_estimator_get_state(&estimator, &state);
    check_near("new_report_dt", state.dt_s, 0.001, 1e-12);
    check_near("new_report_time", state.time_s, 0.002, 1e-12);
}

static void test_mag_calibration_validation(void) {
    DK1Estimator estimator;
    DK1MagCalibration calibration = {
        .hard_iron = {0.0, 0.0, 0.0},
        .axis_order = {0, 0, 1},
        .axis_signs = {1.0, 1.0, 1.0},
        .correction_rate = 1.0,
        .correction_interval_samples = 20
    };

    dk1_estimator_init(&estimator);
    check_int_equal(
        "invalid_axis_order",
        (uint64_t)dk1_estimator_set_mag_calibration(&estimator, &calibration),
        (uint64_t)DK1_ERROR_INVALID_ARGUMENT
    );

    calibration.axis_order[0] = 0;
    calibration.axis_order[1] = 2;
    calibration.axis_order[2] = 1;
    check_int_equal(
        "valid_axis_order",
        (uint64_t)dk1_estimator_set_mag_calibration(&estimator, &calibration),
        (uint64_t)DK1_OK
    );
}

static void test_yaw_correction_uses_twenty_sample_cadence(void) {
    DK1Estimator estimator;
    DK1TrackerState state;
    DK1Sample sample = stationary_sample(0);
    DK1MagCalibration calibration = {
        .hard_iron = {0.0, 0.0, 0.0},
        .axis_order = {0, 1, 2},
        .axis_signs = {1.0, 1.0, 1.0},
        .correction_rate = 2.0,
        .correction_interval_samples = 20
    };

    dk1_estimator_init(&estimator);
    dk1_estimator_set_mag_calibration(&estimator, &calibration);
    dk1_estimator_update(&estimator, &sample);

    sample.mag = (DK1Vector3){0.0, 0.0, 1.0};
    for (uint16_t i = 1; i < 19; ++i) {
        sample.timestamp = i;
        dk1_estimator_update(&estimator, &sample);
    }
    dk1_estimator_get_state(&estimator, &state);
    check_int_equal("no_correction_before_20", state.mag_correction_update_count, 0);

    sample.timestamp = 19;
    dk1_estimator_update(&estimator, &sample);
    dk1_estimator_get_state(&estimator, &state);
    check_int_equal("correction_at_20", state.sample_index, 20);
    check_int_equal("first_correction_count", state.mag_correction_update_count, 1);

    for (uint16_t i = 20; i < 39; ++i) {
        sample.timestamp = i;
        dk1_estimator_update(&estimator, &sample);
    }
    dk1_estimator_get_state(&estimator, &state);
    check_int_equal("no_second_correction_before_40", state.mag_correction_update_count, 1);

    sample.timestamp = 39;
    dk1_estimator_update(&estimator, &sample);
    dk1_estimator_get_state(&estimator, &state);
    check_int_equal("second_correction_at_40", state.sample_index, 40);
    check_int_equal("second_correction_count", state.mag_correction_update_count, 2);
}

int main(void) {
    test_initial_orientation_and_pose();
    test_gyro_integration_uses_trapezoidal_step();
    test_report_timestamp_expansion();
    test_mag_calibration_validation();
    test_yaw_correction_uses_twenty_sample_cadence();

    if (failures != 0) {
        fprintf(stderr, "%d estimator test failure(s)\n", failures);
        return 1;
    }

    printf("estimator tests passed\n");
    return 0;
}
