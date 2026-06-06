#include "DK1Estimator.h"
#include "DK1Tracker/DK1Error.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DK1_G_MPS2 9.80665
#define DK1_DEFAULT_SAMPLE_DT_S 0.001
#define DK1_DEVICE_TIMESTAMP_TICK_S 0.001
#define DK1_MAX_DT_S 0.02
#define DK1_DEFAULT_YAW_CORRECTION_INTERVAL 20u

static const DK1Vector3 DK1_WORLD_UP = {0.0, 1.0, 0.0};
static const DK1Vector3 DK1_G_WORLD = {0.0, -DK1_G_MPS2, 0.0};

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
    return sqrt(vec3_dot(v, v));
}

static DK1Vector3 vec3_normalize_or(DK1Vector3 v, DK1Vector3 fallback) {
    double norm = vec3_norm(v);
    if (norm <= 1e-12) return fallback;
    return vec3_scale(v, 1.0 / norm);
}

static DK1Quaternion quat_normalize(DK1Quaternion q) {
    double norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm <= 1e-12) return (DK1Quaternion){1.0, 0.0, 0.0, 0.0};
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

static DK1Quaternion quat_from_axis_angle(DK1Vector3 axis, double angle) {
    axis = vec3_normalize_or(axis, DK1_WORLD_UP);
    double half_angle = 0.5 * angle;
    double s = sin(half_angle);
    return quat_normalize((DK1Quaternion){
        cos(half_angle),
        axis.x * s,
        axis.y * s,
        axis.z * s
    });
}

static DK1Quaternion quat_from_two_vectors(DK1Vector3 a, DK1Vector3 b) {
    a = vec3_normalize_or(a, DK1_WORLD_UP);
    b = vec3_normalize_or(b, DK1_WORLD_UP);

    double dot = vec3_dot(a, b);
    if (dot > 1.0) dot = 1.0;
    if (dot < -1.0) dot = -1.0;

    if (dot > 1.0 - 1e-10) {
        return (DK1Quaternion){1.0, 0.0, 0.0, 0.0};
    }

    if (dot < -1.0 + 1e-10) {
        DK1Vector3 axis = vec3_cross((DK1Vector3){1.0, 0.0, 0.0}, a);
        if (vec3_norm(axis) < 1e-8) {
            axis = vec3_cross((DK1Vector3){0.0, 1.0, 0.0}, a);
        }
        return quat_from_axis_angle(axis, M_PI);
    }

    DK1Vector3 cross = vec3_cross(a, b);
    return quat_normalize((DK1Quaternion){1.0 + dot, cross.x, cross.y, cross.z});
}

static DK1Quaternion quat_from_body_rate_step(DK1Vector3 omega, double dt) {
    double omega_norm = vec3_norm(omega);
    double angle = omega_norm * dt;
    if (angle < 1e-12) {
        return quat_normalize((DK1Quaternion){
            1.0,
            0.5 * omega.x * dt,
            0.5 * omega.y * dt,
            0.5 * omega.z * dt
        });
    }

    DK1Vector3 axis = vec3_scale(omega, 1.0 / omega_norm);
    double half_angle = 0.5 * angle;
    double s = sin(half_angle);
    return (DK1Quaternion){cos(half_angle), axis.x * s, axis.y * s, axis.z * s};
}

static DK1Vector3 quat_rotate_inverse(DK1Quaternion q, DK1Vector3 v) {
    return dk1_quat_rotate_vec3(quat_conjugate(q), v);
}

static DK1Vector3 horizontal_component(DK1Vector3 v) {
    return vec3_sub(v, vec3_scale(DK1_WORLD_UP, vec3_dot(v, DK1_WORLD_UP)));
}

static double signed_heading_error_rad(DK1Vector3 expected, DK1Vector3 observed) {
    double sin_angle = vec3_dot(DK1_WORLD_UP, vec3_cross(expected, observed));
    double cos_angle = vec3_dot(expected, observed);
    return atan2(sin_angle, cos_angle);
}

static double radians_to_degrees(double radians) {
    return radians * (180.0 / M_PI);
}

static int mag_axis_order_is_valid(const int axis_order[3]) {
    if (!axis_order) return 0;

    int seen[3] = {0, 0, 0};
    for (int i = 0; i < 3; ++i) {
        if (axis_order[i] < 0 || axis_order[i] > 2) return 0;
        if (seen[axis_order[i]]) return 0;
        seen[axis_order[i]] = 1;
    }
    return 1;
}

static DK1Vector3 calibrate_magnetometer(
    DK1Vector3 raw_mag,
    const DK1MagCalibration *calibration
) {
    double centered[3] = {
        raw_mag.x - calibration->hard_iron.x,
        raw_mag.y - calibration->hard_iron.y,
        raw_mag.z - calibration->hard_iron.z
    };
    double signs[3] = {
        calibration->axis_signs.x,
        calibration->axis_signs.y,
        calibration->axis_signs.z
    };

    return (DK1Vector3){
        signs[0] * centered[calibration->axis_order[0]],
        signs[1] * centered[calibration->axis_order[1]],
        signs[2] * centered[calibration->axis_order[2]]
    };
}

static int observed_north_world(
    DK1Quaternion orientation,
    DK1Vector3 mag_calibrated,
    DK1Vector3 fallback,
    DK1Vector3 *out_north
) {
    if (vec3_norm(mag_calibrated) <= 1e-12) return 0;

    DK1Vector3 mag_world = dk1_quat_rotate_vec3(orientation, mag_calibrated);
    DK1Vector3 horizontal = horizontal_component(mag_world);
    if (vec3_norm(horizontal) <= 1e-12) return 0;

    *out_north = vec3_normalize_or(horizontal, fallback);
    return 1;
}

static DK1Vector3 predicted_specific_force(DK1Quaternion orientation) {
    return vec3_scale(quat_rotate_inverse(orientation, DK1_G_WORLD), -1.0);
}

static double clamp_dt(double dt) {
    if (dt <= 0.0 || !isfinite(dt)) return DK1_DEFAULT_SAMPLE_DT_S;
    if (dt > DK1_MAX_DT_S) return DK1_MAX_DT_S;
    return dt;
}

static double estimator_sample_dt(DK1Estimator *est, const DK1Sample *sample) {
    double fallback = est->last_report_sample_dt_s > 0.0
        ? est->last_report_sample_dt_s
        : DK1_DEFAULT_SAMPLE_DT_S;

    if (sample->timestamp != est->last_report_timestamp) {
        uint16_t delta_ticks = (uint16_t)(sample->timestamp - est->last_report_timestamp);
        uint8_t report_sample_count = sample->sample_count ? sample->sample_count : 1;
        double report_dt = (double)delta_ticks * DK1_DEVICE_TIMESTAMP_TICK_S;
        double sample_dt = report_dt / (double)report_sample_count;
        sample_dt = clamp_dt(sample_dt);
        est->last_report_sample_dt_s = sample_dt;
        est->last_report_timestamp = sample->timestamp;
        return sample_dt;
    }

    return clamp_dt(fallback);
}

static void update_heading_residual(DK1Estimator *est, DK1Vector3 mag_calibrated) {
    DK1Vector3 observed = {0.0, 0.0, 0.0};
    if (!observed_north_world(
            est->orientation,
            mag_calibrated,
            est->state.expected_north_world,
            &observed
        )) {
        return;
    }

    if (!est->state.expected_north_initialized) {
        est->state.expected_north_world = observed;
        est->state.expected_north_initialized = 1;
    }

    double error = signed_heading_error_rad(est->state.expected_north_world, observed);
    est->state.heading_residual_deg = radians_to_degrees(error);
}

static void maybe_apply_yaw_correction(DK1Estimator *est, DK1Vector3 mag_calibrated) {
    const DK1MagCalibration *calibration = &est->mag_calibration;
    uint32_t interval = calibration->correction_interval_samples;
    if (interval == 0) interval = DK1_DEFAULT_YAW_CORRECTION_INTERVAL;

    if (calibration->correction_rate <= 0.0) return;
    if (!est->state.expected_north_initialized) return;
    if (est->state.sample_index % (uint64_t)interval != 0u) return;

    DK1Vector3 observed = {0.0, 0.0, 0.0};
    if (!observed_north_world(
            est->orientation,
            mag_calibrated,
            est->state.expected_north_world,
            &observed
        )) {
        return;
    }

    double error = signed_heading_error_rad(est->state.expected_north_world, observed);
    double dt_since_correction = est->state.time_s - est->last_mag_correction_time_s;
    if (dt_since_correction <= 0.0 || !isfinite(dt_since_correction)) {
        dt_since_correction = est->state.dt_s * (double)interval;
    }

    double correction_fraction = 1.0 - exp(-calibration->correction_rate * dt_since_correction);
    double heading_step = error * correction_fraction;
    DK1Quaternion correction = quat_from_axis_angle(DK1_WORLD_UP, -heading_step);

    est->orientation = quat_normalize(quat_mul(correction, est->orientation));
    est->last_mag_correction_time_s = est->state.time_s;
    est->state.mag_correction_update_count++;
}

static void update_derived_state(
    DK1Estimator *est,
    const DK1Sample *sample,
    DK1Vector3 unbiased_gyro,
    DK1Vector3 angular_accel,
    DK1Vector3 mag_calibrated
) {
    DK1Vector3 neck_world = {0.0, 0.0, 0.0};
    DK1Vector3 eye_center_body = dk1_head_model_neck_to_eye_center(&est->head_model);
    DK1Vector3 tracker_rot_accel = dk1_head_model_tracker_rotational_accel(
        &est->head_model,
        unbiased_gyro,
        angular_accel
    );

    est->state.device_timestamp = sample->timestamp;
    est->state.orientation = est->orientation;
    est->state.unbiased_gyro = unbiased_gyro;
    est->state.angular_accel = angular_accel;
    est->state.look_dir_world = dk1_head_model_looking_direction_world(
        &est->head_model,
        est->orientation
    );
    est->state.eye_center_world = dk1_quat_rotate_vec3(est->orientation, eye_center_body);
    dk1_head_model_eye_positions_world(
        &est->head_model,
        est->orientation,
        neck_world,
        &est->state.left_eye_world,
        &est->state.right_eye_world
    );

    est->state.predicted_accel = predicted_specific_force(est->orientation);
    est->state.accel_residual = vec3_sub(sample->accel, est->state.predicted_accel);
    est->state.pivot_accel_body = vec3_sub(est->state.accel_residual, tracker_rot_accel);
    est->state.pivot_accel_world = dk1_quat_rotate_vec3(
        est->orientation,
        est->state.pivot_accel_body
    );

    est->state.mag_calibrated = mag_calibrated;
    est->state.mag_correction_rate = est->mag_calibration.correction_rate;
    est->state.mag_correction_interval_samples =
        est->mag_calibration.correction_interval_samples;
    update_heading_residual(est, mag_calibrated);
}

void dk1_estimator_init(DK1Estimator *est) {
    if (!est) return;

    memset(est, 0, sizeof(*est));
    est->orientation = (DK1Quaternion){1.0, 0.0, 0.0, 0.0};
    est->state.orientation = est->orientation;
    est->state.expected_north_world = (DK1Vector3){1.0, 0.0, 0.0};
    est->state.mag_correction_interval_samples = DK1_DEFAULT_YAW_CORRECTION_INTERVAL;
    est->mag_calibration.hard_iron = (DK1Vector3){0.0, 0.0, 0.0};
    est->mag_calibration.axis_order[0] = 0;
    est->mag_calibration.axis_order[1] = 1;
    est->mag_calibration.axis_order[2] = 2;
    est->mag_calibration.axis_signs = (DK1Vector3){1.0, 1.0, 1.0};
    est->mag_calibration.correction_rate = 0.0;
    est->mag_calibration.correction_interval_samples =
        DK1_DEFAULT_YAW_CORRECTION_INTERVAL;
    est->last_report_sample_dt_s = DK1_DEFAULT_SAMPLE_DT_S;
    dk1_head_model_init_default(&est->head_model);
}

void dk1_estimator_update(DK1Estimator *est, const DK1Sample *sample) {
    if (!est || !sample) return;

    DK1Vector3 mag_calibrated = calibrate_magnetometer(sample->mag, &est->mag_calibration);
    DK1Vector3 unbiased_gyro = vec3_sub(sample->gyro, est->gyro_bias);

    if (!est->state.initialized) {
        est->orientation = quat_from_two_vectors(
            vec3_normalize_or(sample->accel, DK1_WORLD_UP),
            DK1_WORLD_UP
        );
        est->state.initialized = 1;
        est->state.sample_index = 1;
        est->state.time_s = 0.0;
        est->state.dt_s = 0.0;
        est->last_report_timestamp = sample->timestamp;
        est->last_report_sample_dt_s = DK1_DEFAULT_SAMPLE_DT_S;
        est->previous_unbiased_gyro = unbiased_gyro;
        est->have_previous_sample = 1;
        update_derived_state(
            est,
            sample,
            unbiased_gyro,
            (DK1Vector3){0.0, 0.0, 0.0},
            mag_calibrated
        );
        return;
    }

    double dt = estimator_sample_dt(est, sample);
    est->state.sample_index++;
    est->state.dt_s = dt;
    est->state.time_s += dt;

    DK1Vector3 averaged_gyro = vec3_scale(
        vec3_add(est->previous_unbiased_gyro, unbiased_gyro),
        0.5
    );
    DK1Quaternion delta = quat_from_body_rate_step(averaged_gyro, dt);
    est->orientation = quat_normalize(quat_mul(est->orientation, delta));

    DK1Vector3 angular_accel = vec3_scale(
        vec3_sub(unbiased_gyro, est->previous_unbiased_gyro),
        1.0 / dt
    );

    update_heading_residual(est, mag_calibrated);
    maybe_apply_yaw_correction(est, mag_calibrated);
    update_derived_state(est, sample, unbiased_gyro, angular_accel, mag_calibrated);

    est->previous_unbiased_gyro = unbiased_gyro;
    est->have_previous_sample = 1;
}

void dk1_estimator_get_state(const DK1Estimator *est, DK1TrackerState *out_state) {
    if (!est || !out_state) return;
    *out_state = est->state;
}

void dk1_estimator_set_gyro_bias(DK1Estimator *est, DK1Vector3 bias) {
    if (!est) return;
    est->gyro_bias = bias;
}

int dk1_estimator_set_mag_calibration(
    DK1Estimator *est,
    const DK1MagCalibration *calibration
) {
    if (!est || !calibration) return DK1_ERROR_INVALID_ARGUMENT;
    if (!mag_axis_order_is_valid(calibration->axis_order)) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }
    if (calibration->correction_rate < 0.0 || !isfinite(calibration->correction_rate)) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }
    if (calibration->correction_interval_samples == 0) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }

    est->mag_calibration = *calibration;
    est->state.mag_correction_rate = calibration->correction_rate;
    est->state.mag_correction_interval_samples =
        calibration->correction_interval_samples;
    return DK1_OK;
}
