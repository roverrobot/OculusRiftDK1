#include "DK1HeadModel.h"

#include <math.h>
#include <stdio.h>

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

static void check_vec3_near(
    const char *name,
    DK1Vector3 actual,
    double expected_x,
    double expected_y,
    double expected_z
) {
    char component_name[128];
    snprintf(component_name, sizeof(component_name), "%s.x", name);
    check_near(component_name, actual.x, expected_x, 1e-9);
    snprintf(component_name, sizeof(component_name), "%s.y", name);
    check_near(component_name, actual.y, expected_y, 1e-9);
    snprintf(component_name, sizeof(component_name), "%s.z", name);
    check_near(component_name, actual.z, expected_z, 1e-9);
}

static void test_default_geometry(void) {
    DK1HeadModel model;
    dk1_head_model_init_default(&model);

    check_vec3_near("neck_to_tracker", model.neck_to_tracker, 0.0, 0.10, 0.10);
    check_vec3_near(
        "neck_to_head_center",
        model.neck_to_head_center,
        0.0,
        0.08,
        0.08
    );
    check_vec3_near("head_center_to_eye", model.head_center_to_eye, 0.0, 0.02, 0.08);
    check_near("ipd_m", model.ipd_m, 0.064, 1e-12);

    DK1Vector3 eye_center = dk1_head_model_neck_to_eye_center(&model);
    check_vec3_near("eye_center", eye_center, 0.0, 0.10, 0.16);

    DK1Vector3 left_eye;
    DK1Vector3 right_eye;
    dk1_head_model_eye_offsets(&model, &left_eye, &right_eye);

    check_vec3_near("left_eye_from_neck", left_eye, -0.032, 0.10, 0.16);
    check_vec3_near("right_eye_from_neck", right_eye, 0.032, 0.10, 0.16);
    check_near("eye_separation_x", right_eye.x - left_eye.x, model.ipd_m, 1e-12);
    check_near("eye_midpoint_x", (left_eye.x + right_eye.x) * 0.5, eye_center.x, 1e-12);
    check_near("eye_midpoint_y", (left_eye.y + right_eye.y) * 0.5, eye_center.y, 1e-12);
    check_near("eye_midpoint_z", (left_eye.z + right_eye.z) * 0.5, eye_center.z, 1e-12);
}

static void test_null_model_outputs(void) {
    DK1Vector3 center = dk1_head_model_neck_to_eye_center(NULL);
    check_vec3_near("null_eye_center", center, 0.0, 0.0, 0.0);

    DK1Vector3 left_eye = {1.0, 2.0, 3.0};
    DK1Vector3 right_eye = {4.0, 5.0, 6.0};
    dk1_head_model_eye_offsets(NULL, &left_eye, &right_eye);
    check_vec3_near("null_left_eye", left_eye, 0.0, 0.0, 0.0);
    check_vec3_near("null_right_eye", right_eye, 0.0, 0.0, 0.0);

    DK1Vector3 omega = {1.0, 2.0, 3.0};
    DK1Vector3 alpha = {4.0, 5.0, 6.0};
    DK1Vector3 accel = dk1_head_model_tracker_rotational_accel(NULL, omega, alpha);
    check_vec3_near("null_rotational_accel", accel, 0.0, 0.0, 0.0);
}

static void test_quaternion_rotation(void) {
    DK1Vector3 v = {1.0, 2.0, 3.0};

    check_vec3_near(
        "identity_rotation",
        dk1_quat_rotate_vec3((DK1Quaternion){1.0, 0.0, 0.0, 0.0}, v),
        1.0,
        2.0,
        3.0
    );
    check_vec3_near(
        "normalized_identity_rotation",
        dk1_quat_rotate_vec3((DK1Quaternion){2.0, 0.0, 0.0, 0.0}, v),
        1.0,
        2.0,
        3.0
    );
    check_vec3_near(
        "zero_quaternion_rotation",
        dk1_quat_rotate_vec3((DK1Quaternion){0.0, 0.0, 0.0, 0.0}, v),
        1.0,
        2.0,
        3.0
    );

    double s = sqrt(0.5);
    DK1Quaternion z90 = {s, 0.0, 0.0, s};
    DK1Vector3 x_axis = {1.0, 0.0, 0.0};
    check_vec3_near("z90_rotation", dk1_quat_rotate_vec3(z90, x_axis), 0.0, 1.0, 0.0);

    DK1Quaternion non_unit_z90 = {2.0 * s, 0.0, 0.0, 2.0 * s};
    check_vec3_near(
        "non_unit_z90_rotation",
        dk1_quat_rotate_vec3(non_unit_z90, x_axis),
        0.0,
        1.0,
        0.0
    );
}

static void test_eye_positions_world(void) {
    DK1HeadModel model;
    dk1_head_model_init_default(&model);

    DK1Vector3 neck_world = {1.0, 2.0, 3.0};
    DK1Vector3 left_eye;
    DK1Vector3 right_eye;

    dk1_head_model_eye_positions_world(
        &model,
        (DK1Quaternion){1.0, 0.0, 0.0, 0.0},
        neck_world,
        &left_eye,
        &right_eye
    );
    check_vec3_near("identity_left_eye_world", left_eye, 0.968, 2.10, 3.16);
    check_vec3_near("identity_right_eye_world", right_eye, 1.032, 2.10, 3.16);

    double s = sqrt(0.5);
    dk1_head_model_eye_positions_world(
        &model,
        (DK1Quaternion){s, 0.0, 0.0, s},
        neck_world,
        &left_eye,
        &right_eye
    );
    check_vec3_near("z90_left_eye_world", left_eye, 0.90, 1.968, 3.16);
    check_vec3_near("z90_right_eye_world", right_eye, 0.90, 2.032, 3.16);
}

static void test_tracker_rotational_accel(void) {
    DK1HeadModel model;
    dk1_head_model_init_default(&model);

    DK1Vector3 omega_zero = {0.0, 0.0, 0.0};
    DK1Vector3 alpha_z = {0.0, 0.0, 2.0};
    DK1Vector3 tangential = dk1_head_model_tracker_rotational_accel(
        &model,
        omega_zero,
        alpha_z
    );
    check_vec3_near("tangential_accel", tangential, -0.20, 0.0, 0.0);

    DK1Vector3 omega_z = {0.0, 0.0, 2.0};
    DK1Vector3 alpha_zero = {0.0, 0.0, 0.0};
    DK1Vector3 centripetal = dk1_head_model_tracker_rotational_accel(
        &model,
        omega_z,
        alpha_zero
    );
    check_vec3_near("centripetal_accel", centripetal, 0.0, -0.40, 0.0);

    DK1Vector3 combined = dk1_head_model_tracker_rotational_accel(
        &model,
        omega_z,
        alpha_z
    );
    check_vec3_near("combined_rotational_accel", combined, -0.20, -0.40, 0.0);
}

int main(void) {
    test_default_geometry();
    test_null_model_outputs();
    test_quaternion_rotation();
    test_eye_positions_world();
    test_tracker_rotational_accel();

    if (failures != 0) {
        fprintf(stderr, "%d head model test failure(s)\n", failures);
        return 1;
    }

    printf("head model tests passed\n");
    return 0;
}
