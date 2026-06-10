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

static double vec3_norm(DK1Vector3 v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
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
