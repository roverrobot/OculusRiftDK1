#include "DK1Tracker/DK1Tracker.h"

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

static void check_int_equal(const char *name, int actual, int expected) {
    if (actual != expected) {
        fprintf(stderr, "%s: expected %d, got %d\n", name, expected, actual);
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

static DK1TrackerState base_state(void) {
    DK1TrackerState state = {0};
    state.initialized = 1;
    state.orientation = (DK1Quaternion){1.0, 0.0, 0.0, 0.0};
    state.look_dir_world = (DK1Vector3){0.0, 0.0, -1.0};
    state.pivot_position_world = (DK1Vector3){1.0, 2.0, 3.0};
    state.pivot_position_reliable = 1;
    state.left_eye_world = (DK1Vector3){0.968, 2.10, 2.84};
    state.right_eye_world = (DK1Vector3){1.032, 2.10, 2.84};
    return state;
}

static void test_zero_dt_copies_pose(void) {
    DK1TrackerState state = base_state();
    DK1PredictedState prediction;

    int result = dk1_predict_state(&state, 0.0, &prediction);

    check_int_equal("zero_dt_result", result, DK1_OK);
    check_vec3_near("zero_dt_look", prediction.look_dir_world, 0.0, 0.0, -1.0, 1e-12);
    check_vec3_near("zero_dt_left", prediction.left_eye_world, 0.968, 2.10, 2.84, 1e-12);
    check_vec3_near("zero_dt_right", prediction.right_eye_world, 1.032, 2.10, 2.84, 1e-12);
}

static void test_yaw_prediction_rotates_pose(void) {
    DK1TrackerState state = base_state();
    DK1PredictedState prediction;

    state.pivot_position_world = (DK1Vector3){0.0, 0.0, 0.0};
    state.left_eye_world = (DK1Vector3){-0.032, 0.10, -0.16};
    state.right_eye_world = (DK1Vector3){0.032, 0.10, -0.16};
    state.unbiased_gyro = (DK1Vector3){0.0, M_PI * 0.5, 0.0};

    int result = dk1_predict_state(&state, 1.0, &prediction);

    check_int_equal("yaw_result", result, DK1_OK);
    check_vec3_near("yaw_look", prediction.look_dir_world, -1.0, 0.0, 0.0, 1e-9);
    check_vec3_near("yaw_left", prediction.left_eye_world, -0.16, 0.10, 0.032, 1e-9);
    check_vec3_near("yaw_right", prediction.right_eye_world, -0.16, 0.10, -0.032, 1e-9);
}

static void test_reliable_pivot_translation_predicts_eye_positions(void) {
    DK1TrackerState state = base_state();
    DK1PredictedState prediction;

    state.left_eye_world = (DK1Vector3){1.0, 2.0, 2.0};
    state.right_eye_world = (DK1Vector3){2.0, 2.0, 2.0};
    state.pivot_velocity_world = (DK1Vector3){0.5, 0.0, 0.0};
    state.pivot_accel_world = (DK1Vector3){2.0, 0.0, 0.0};

    int result = dk1_predict_state(&state, 0.25, &prediction);

    check_int_equal("pivot_translation_result", result, DK1_OK);
    check_vec3_near("pivot_translation_left", prediction.left_eye_world, 1.1875, 2.0, 2.0, 1e-12);
    check_vec3_near("pivot_translation_right", prediction.right_eye_world, 2.1875, 2.0, 2.0, 1e-12);
}

static void test_unreliable_pivot_holds_translation(void) {
    DK1TrackerState state = base_state();
    DK1PredictedState prediction;

    state.pivot_position_reliable = 0;
    state.pivot_velocity_world = (DK1Vector3){100.0, 0.0, 0.0};
    state.pivot_accel_world = (DK1Vector3){100.0, 0.0, 0.0};

    int result = dk1_predict_state(&state, 1.0, &prediction);

    check_int_equal("unreliable_pivot_result", result, DK1_OK);
    check_vec3_near("unreliable_pivot_left", prediction.left_eye_world, 0.968, 2.10, 2.84, 1e-12);
    check_vec3_near("unreliable_pivot_right", prediction.right_eye_world, 1.032, 2.10, 2.84, 1e-12);
}

static void test_invalid_arguments(void) {
    DK1TrackerState state = base_state();
    DK1PredictedState prediction;

    check_int_equal(
        "null_state",
        dk1_predict_state(NULL, 0.0, &prediction),
        DK1_ERROR_INVALID_ARGUMENT
    );
    check_int_equal(
        "null_prediction",
        dk1_predict_state(&state, 0.0, NULL),
        DK1_ERROR_INVALID_ARGUMENT
    );
    check_int_equal(
        "negative_dt",
        dk1_predict_state(&state, -0.001, &prediction),
        DK1_ERROR_INVALID_ARGUMENT
    );
}

int main(void) {
    test_zero_dt_copies_pose();
    test_yaw_prediction_rotates_pose();
    test_reliable_pivot_translation_predicts_eye_positions();
    test_unreliable_pivot_holds_translation();
    test_invalid_arguments();

    if (failures) {
        fprintf(stderr, "%d prediction test(s) failed\n", failures);
        return 1;
    }

    printf("prediction tests passed\n");
    return 0;
}
