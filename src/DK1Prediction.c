#include "DK1Tracker/DK1Tracker.h"
#include "DK1HeadModel.h"

#include <math.h>

static DK1Vector3 vec3_add(DK1Vector3 a, DK1Vector3 b) {
    return (DK1Vector3){a.x + b.x, a.y + b.y, a.z + b.z};
}

static DK1Vector3 vec3_sub(DK1Vector3 a, DK1Vector3 b) {
    return (DK1Vector3){a.x - b.x, a.y - b.y, a.z - b.z};
}

static DK1Vector3 vec3_scale(DK1Vector3 v, double scale) {
    return (DK1Vector3){v.x * scale, v.y * scale, v.z * scale};
}

static double vec3_dot(DK1Vector3 a, DK1Vector3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static DK1Vector3 vec3_cross(DK1Vector3 a, DK1Vector3 b) {
    return (DK1Vector3){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

static double vec3_norm(DK1Vector3 v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

static int vec3_is_finite(DK1Vector3 v) {
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static int vec3_normalize_checked(DK1Vector3 v, DK1Vector3 *out_v) {
    double norm = vec3_norm(v);
    if (norm <= 1e-12 || !isfinite(norm)) return 0;
    *out_v = vec3_scale(v, 1.0 / norm);
    return 1;
}

static DK1Quaternion quat_identity(void) {
    return (DK1Quaternion){1.0, 0.0, 0.0, 0.0};
}

static DK1Quaternion quat_normalize(DK1Quaternion q) {
    double norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm <= 0.0 || !isfinite(norm)) return quat_identity();
    double inv = 1.0 / norm;
    return (DK1Quaternion){q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

static DK1Quaternion quat_conjugate(DK1Quaternion q) {
    return (DK1Quaternion){q.w, -q.x, -q.y, -q.z};
}

static DK1Quaternion quat_mul(DK1Quaternion a, DK1Quaternion b) {
    return (DK1Quaternion){
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
}

static DK1Quaternion quat_from_body_rate_step(DK1Vector3 body_rate, double dt_s) {
    double rate = vec3_norm(body_rate);
    if (rate <= 0.0 || !isfinite(rate) || dt_s == 0.0) return quat_identity();

    double angle = rate * dt_s;
    double half_angle = 0.5 * angle;
    double axis_scale = sin(half_angle) / rate;
    return quat_normalize((DK1Quaternion){
        cos(half_angle),
        body_rate.x * axis_scale,
        body_rate.y * axis_scale,
        body_rate.z * axis_scale
    });
}

static DK1Vector3 predict_pivot_position(const DK1TrackerState *state, double dt_s) {
    DK1Vector3 pivot = state->pivot_position_world;
    if (!state->pivot_position_reliable) return pivot;

    double damping = state->pivot_damping_per_second;
    if (damping > 0.0 && isfinite(damping)) {
        double decay = exp(-damping * dt_s);
        double velocity_scale = (1.0 - decay) / damping;
        double accel_scale = dt_s / damping - (1.0 - decay) / (damping * damping);
        return vec3_add(
            pivot,
            vec3_add(
                vec3_scale(state->pivot_velocity_world, velocity_scale),
                vec3_scale(state->pivot_accel_world, accel_scale)
            )
        );
    }

    return vec3_add(
        pivot,
        vec3_add(
            vec3_scale(state->pivot_velocity_world, dt_s),
            vec3_scale(state->pivot_accel_world, 0.5 * dt_s * dt_s)
        )
    );
}

int dk1_predict_state(
    const DK1TrackerState *state,
    double dt_s,
    DK1PredictedState *out_prediction
) {
    if (!state || !out_prediction) return DK1_ERROR_INVALID_ARGUMENT;
    if (dt_s < 0.0 || !isfinite(dt_s)) return DK1_ERROR_INVALID_ARGUMENT;

    DK1Quaternion current_orientation = quat_normalize(state->orientation);
    DK1Vector3 average_gyro = vec3_add(
        state->unbiased_gyro,
        vec3_scale(state->angular_accel, 0.5 * dt_s)
    );
    DK1Quaternion orientation_step = quat_from_body_rate_step(average_gyro, dt_s);
    DK1Quaternion predicted_orientation = quat_normalize(
        quat_mul(current_orientation, orientation_step)
    );
    DK1Quaternion relative_rotation = quat_normalize(
        quat_mul(predicted_orientation, quat_conjugate(current_orientation))
    );

    DK1Vector3 pivot_now = state->pivot_position_world;
    DK1Vector3 pivot_predicted = predict_pivot_position(state, dt_s);
    DK1Vector3 left_from_pivot = vec3_sub(state->left_eye_world, pivot_now);
    DK1Vector3 right_from_pivot = vec3_sub(state->right_eye_world, pivot_now);

    out_prediction->look_dir_world = dk1_quat_rotate_vec3(
        relative_rotation,
        state->look_dir_world
    );
    out_prediction->left_eye_world = vec3_add(
        pivot_predicted,
        dk1_quat_rotate_vec3(relative_rotation, left_from_pivot)
    );
    out_prediction->right_eye_world = vec3_add(
        pivot_predicted,
        dk1_quat_rotate_vec3(relative_rotation, right_from_pivot)
    );
    return DK1_OK;
}

static void write_node_transform(
    DK1Vector3 position,
    DK1Vector3 right,
    DK1Vector3 up,
    DK1Vector3 back,
    double out_transform[16]
) {
    out_transform[0] = right.x;
    out_transform[1] = right.y;
    out_transform[2] = right.z;
    out_transform[3] = 0.0;

    out_transform[4] = up.x;
    out_transform[5] = up.y;
    out_transform[6] = up.z;
    out_transform[7] = 0.0;

    out_transform[8] = back.x;
    out_transform[9] = back.y;
    out_transform[10] = back.z;
    out_transform[11] = 0.0;

    out_transform[12] = position.x;
    out_transform[13] = position.y;
    out_transform[14] = position.z;
    out_transform[15] = 1.0;
}

static void write_view_transform(
    DK1Vector3 position,
    DK1Vector3 right,
    DK1Vector3 up,
    DK1Vector3 back,
    double out_transform[16]
) {
    out_transform[0] = right.x;
    out_transform[1] = up.x;
    out_transform[2] = back.x;
    out_transform[3] = 0.0;

    out_transform[4] = right.y;
    out_transform[5] = up.y;
    out_transform[6] = back.y;
    out_transform[7] = 0.0;

    out_transform[8] = right.z;
    out_transform[9] = up.z;
    out_transform[10] = back.z;
    out_transform[11] = 0.0;

    out_transform[12] = -vec3_dot(right, position);
    out_transform[13] = -vec3_dot(up, position);
    out_transform[14] = -vec3_dot(back, position);
    out_transform[15] = 1.0;
}

static void write_eye_settings(
    DK1Vector3 position,
    DK1Vector3 right,
    DK1Vector3 up,
    DK1Vector3 forward,
    DK1SCNCameraEyeSettings *out_eye
) {
    DK1Vector3 back = vec3_scale(forward, -1.0);

    out_eye->position_world = position;
    out_eye->right_world = right;
    out_eye->up_world = up;
    out_eye->forward_world = forward;
    write_node_transform(
        position,
        right,
        up,
        back,
        out_eye->node_transform_column_major
    );
    write_view_transform(
        position,
        right,
        up,
        back,
        out_eye->view_transform_column_major
    );
}

int dk1_scn_camera_settings_from_prediction(
    const DK1PredictedState *prediction,
    DK1SCNCameraSettings *out_settings
) {
    if (!prediction || !out_settings) return DK1_ERROR_INVALID_ARGUMENT;
    if (
        !vec3_is_finite(prediction->look_dir_world) ||
        !vec3_is_finite(prediction->left_eye_world) ||
        !vec3_is_finite(prediction->right_eye_world)
    ) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }

    DK1Vector3 forward;
    if (!vec3_normalize_checked(prediction->look_dir_world, &forward)) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }

    DK1Vector3 back = vec3_scale(forward, -1.0);
    DK1Vector3 right_hint;
    DK1Vector3 up;
    if (!vec3_normalize_checked(
        vec3_sub(prediction->right_eye_world, prediction->left_eye_world),
        &right_hint
    )) {
        right_hint = (DK1Vector3){1.0, 0.0, 0.0};
    }

    if (!vec3_normalize_checked(vec3_cross(back, right_hint), &up)) {
        DK1Vector3 fallback_up = fabs(back.y) < 0.99
            ? (DK1Vector3){0.0, 1.0, 0.0}
            : (DK1Vector3){1.0, 0.0, 0.0};
        DK1Vector3 fallback_right;
        if (!vec3_normalize_checked(vec3_cross(fallback_up, back), &fallback_right)) {
            fallback_right = (DK1Vector3){1.0, 0.0, 0.0};
        }
        right_hint = fallback_right;
        if (!vec3_normalize_checked(vec3_cross(back, right_hint), &up)) {
            up = (DK1Vector3){0.0, 1.0, 0.0};
        }
    }

    DK1Vector3 right;
    if (!vec3_normalize_checked(vec3_cross(up, back), &right)) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }

    write_eye_settings(
        prediction->left_eye_world,
        right,
        up,
        forward,
        &out_settings->eyes[DK1_EYE_LEFT]
    );
    write_eye_settings(
        prediction->right_eye_world,
        right,
        up,
        forward,
        &out_settings->eyes[DK1_EYE_RIGHT]
    );
    return DK1_OK;
}
