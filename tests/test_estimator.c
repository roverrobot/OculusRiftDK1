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
    check_vec3_near("eye_center_world", state.eye_center_world, 0.0, 1.775, -0.16, 1e-12);
    check_vec3_near("left_eye_world", state.left_eye_world, -0.032, 1.775, -0.16, 1e-12);
    check_vec3_near("right_eye_world", state.right_eye_world, 0.032, 1.775, -0.16, 1e-12);
    check_vec3_near("expected_north_world", state.expected_north_world, 1.0, 0.0, 0.0, 1e-12);
    check_vec3_near("observed_north_world", state.observed_north_world, 1.0, 0.0, 0.0, 1e-12);
    check_int_equal("north_window_sample_count", state.north_window_sample_count, 1);
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

static void test_pivot_position_uses_trapezoidal_integration(void) {
    DK1Estimator estimator;
    DK1TrackerState state;
    DK1HeadNeckConfig config = {
        .h_m = 0.10,
        .ell_m = 0.16,
        .ipd_m = 0.064,
        .pivot_damping_per_second = 0.0,
        .max_dt_s = 2.0,
        .max_report_sample_count = 3,
        .use_pivot_inference = 1
    };
    DK1Sample sample = stationary_sample(0);

    dk1_estimator_init(&estimator);
    dk1_estimator_set_head_neck_config(&estimator, &config);
    dk1_estimator_update(&estimator, &sample);

    sample.timestamp = 1000;
    sample.accel = (DK1Vector3){1.0, 9.80665, 0.0};
    dk1_estimator_update(&estimator, &sample);
    dk1_estimator_get_state(&estimator, &state);

    check_vec3_near("pivot_accel_world", state.pivot_accel_world, 1.0, 0.0, 0.0, 1e-9);
    check_vec3_near("pivot_velocity_world", state.pivot_velocity_world, 0.5, 0.0, 0.0, 1e-9);
    check_vec3_near("pivot_position_world", state.pivot_position_world, 0.25, 0.0, 0.0, 1e-9);

    sample.timestamp = 2000;
    sample.accel = (DK1Vector3){3.0, 9.80665, 0.0};
    dk1_estimator_update(&estimator, &sample);
    dk1_estimator_get_state(&estimator, &state);

    check_vec3_near("pivot_accel_world_second", state.pivot_accel_world, 3.0, 0.0, 0.0, 1e-9);
    check_vec3_near("pivot_velocity_world_second", state.pivot_velocity_world, 2.5, 0.0, 0.0, 1e-9);
    check_vec3_near("pivot_position_world_second", state.pivot_position_world, 1.75, 0.0, 0.0, 1e-9);
    check_int_equal("pivot_reliable_clean", (uint64_t)state.pivot_position_reliable, 1);
    check_int_equal("pivot_skip_clean", state.pivot_integration_skipped_count, 0);
}

static void test_pivot_inference_can_be_disabled(void) {
    DK1Estimator estimator;
    DK1TrackerState state;
    DK1HeadNeckConfig config = {
        .h_m = 0.10,
        .ell_m = 0.16,
        .ipd_m = 0.064,
        .pivot_damping_per_second = 0.0,
        .max_dt_s = 2.0,
        .max_report_sample_count = 3,
        .use_pivot_inference = 0
    };
    DK1Sample sample = stationary_sample(0);

    dk1_estimator_init(&estimator);
    dk1_estimator_set_head_neck_config(&estimator, &config);
    dk1_estimator_update(&estimator, &sample);

    sample.timestamp = 1000;
    sample.accel = (DK1Vector3){1.0, 9.80665, 0.0};
    dk1_estimator_update(&estimator, &sample);
    dk1_estimator_get_state(&estimator, &state);

    check_vec3_near("disabled_pivot_accel_world", state.pivot_accel_world, 1.0, 0.0, 0.0, 1e-9);
    check_vec3_near("disabled_pivot_velocity", state.pivot_velocity_world, 0.0, 0.0, 0.0, 1e-12);
    check_vec3_near("disabled_pivot_position", state.pivot_position_world, 0.0, 0.0, 0.0, 1e-12);
    check_int_equal("disabled_pivot_skips", state.pivot_integration_skipped_count, 1);
    check_int_equal("disabled_pivot_reliable", (uint64_t)state.pivot_position_reliable, 0);
}

static void test_pivot_integration_skips_oversized_report_group(void) {
    DK1Estimator estimator;
    DK1TrackerState state;
    DK1Sample sample = stationary_sample(0);

    dk1_estimator_init(&estimator);
    dk1_estimator_update(&estimator, &sample);

    sample.timestamp = 4;
    sample.sample_count = 4;
    sample.accel = (DK1Vector3){1.0, 9.80665, 0.0};
    dk1_estimator_update(&estimator, &sample);
    dk1_estimator_get_state(&estimator, &state);

    check_vec3_near("oversized_pivot_velocity", state.pivot_velocity_world, 0.0, 0.0, 0.0, 1e-12);
    check_vec3_near("oversized_pivot_position", state.pivot_position_world, 0.0, 0.0, 0.0, 1e-12);
    check_int_equal("oversized_group_count", state.timing_oversized_group_count, 1);
    check_int_equal("oversized_max_group", (uint64_t)state.timing_max_report_sample_count, 4);
    check_int_equal("oversized_repeated_groups", state.timing_repeated_group_count, 1);
    check_int_equal("oversized_pivot_skips", state.pivot_integration_skipped_count, 1);
    check_int_equal("oversized_reliable", (uint64_t)state.pivot_position_reliable, 0);
}

static void test_head_neck_config_sets_recent_fit_geometry(void) {
    DK1Estimator estimator;
    DK1TrackerState state;
    DK1HeadNeckConfig config = {
        .h_m = 0.10113,
        .ell_m = 0.15902,
        .ipd_m = 0.064,
        .pivot_damping_per_second = 2.0,
        .max_dt_s = 0.02,
        .max_report_sample_count = 3,
        .use_pivot_inference = 1
    };
    DK1Sample sample = stationary_sample(10);

    dk1_estimator_init(&estimator);
    check_int_equal(
        "head_neck_config_valid",
        (uint64_t)dk1_estimator_set_head_neck_config(&estimator, &config),
        (uint64_t)DK1_OK
    );
    dk1_estimator_update(&estimator, &sample);
    dk1_estimator_get_state(&estimator, &state);

    check_vec3_near("fit_eye_center_world", state.eye_center_world, 0.0, 1.77613, -0.15902, 1e-12);
    check_near("fit_pivot_damping", state.pivot_damping_per_second, 2.0, 1e-12);
    check_near("fit_max_dt", state.timing_max_dt_s, 0.02, 1e-12);
}

static void test_eye_height_offsets_eye_positions(void) {
    DK1Estimator estimator;
    DK1TrackerState state;

    dk1_estimator_init(&estimator);
    check_int_equal(
        "eye_height_valid",
        (uint64_t)dk1_estimator_set_eye_height(&estimator, 1.25),
        (uint64_t)DK1_OK
    );
    dk1_estimator_get_state(&estimator, &state);

    check_vec3_near("height_eye_center_world", state.eye_center_world, 0.0, 1.35, -0.16, 1e-12);
    check_vec3_near("height_left_eye_world", state.left_eye_world, -0.032, 1.35, -0.16, 1e-12);
    check_vec3_near("height_right_eye_world", state.right_eye_world, 0.032, 1.35, -0.16, 1e-12);
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
    calibration.correction_interval_samples = DK1_ESTIMATOR_MAX_NORTH_WINDOW_SAMPLES + 1;
    check_int_equal(
        "invalid_large_window",
        (uint64_t)dk1_estimator_set_mag_calibration(&estimator, &calibration),
        (uint64_t)DK1_ERROR_INVALID_ARGUMENT
    );

    calibration.correction_interval_samples = 20;
    check_int_equal(
        "valid_axis_order",
        (uint64_t)dk1_estimator_set_mag_calibration(&estimator, &calibration),
        (uint64_t)DK1_OK
    );
}

static void test_north_average_uses_twenty_samples(void) {
    DK1Estimator estimator;
    DK1TrackerState state;
    DK1Sample sample = stationary_sample(0);
    DK1MagCalibration calibration = {
        .hard_iron = {0.0, 0.0, 0.0},
        .axis_order = {0, 1, 2},
        .axis_signs = {1.0, 1.0, 1.0},
        .correction_rate = 0.0,
        .correction_interval_samples = 20
    };
    double inv_sqrt_2 = 1.0 / sqrt(2.0);

    dk1_estimator_init(&estimator);
    dk1_estimator_set_mag_calibration(&estimator, &calibration);
    dk1_estimator_update(&estimator, &sample);

    for (uint16_t i = 1; i < 10; ++i) {
        sample.timestamp = i;
        sample.mag = (DK1Vector3){1.0, 0.0, 0.0};
        dk1_estimator_update(&estimator, &sample);
    }

    for (uint16_t i = 10; i < 20; ++i) {
        sample.timestamp = i;
        sample.mag = (DK1Vector3){0.0, 0.0, 1.0};
        dk1_estimator_update(&estimator, &sample);
    }

    dk1_estimator_get_state(&estimator, &state);
    check_int_equal("north_average_sample_index", state.sample_index, 20);
    check_int_equal("north_average_window_count", state.north_window_sample_count, 20);
    check_vec3_near(
        "north_average_observed",
        state.observed_north_world,
        inv_sqrt_2,
        0.0,
        inv_sqrt_2,
        1e-12
    );
    check_near("north_average_heading_residual", state.heading_residual_deg, -45.0, 1e-12);
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
    check_int_equal("correction_window_count", state.north_window_sample_count, 20);
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
    test_pivot_position_uses_trapezoidal_integration();
    test_pivot_inference_can_be_disabled();
    test_pivot_integration_skips_oversized_report_group();
    test_head_neck_config_sets_recent_fit_geometry();
    test_eye_height_offsets_eye_positions();
    test_mag_calibration_validation();
    test_north_average_uses_twenty_samples();
    test_yaw_correction_uses_twenty_sample_cadence();

    if (failures != 0) {
        fprintf(stderr, "%d estimator test failure(s)\n", failures);
        return 1;
    }

    printf("estimator tests passed\n");
    return 0;
}
