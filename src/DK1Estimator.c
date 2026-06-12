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
#define DK1_MAX_DT_S DK1_DEFAULT_HEAD_NECK_MAX_DT_S
#define DK1_DEFAULT_YAW_CORRECTION_INTERVAL 20u
#define DK1_DEFAULT_PIVOT_DAMPING_PER_SECOND \
    DK1_DEFAULT_HEAD_NECK_PIVOT_DAMPING_PER_SECOND
#define DK1_DEFAULT_MAX_REPORT_SAMPLE_COUNT \
    DK1_DEFAULT_HEAD_NECK_MAX_REPORT_SAMPLE_COUNT

typedef struct SampleTiming {
    double raw_dt_s;
    double dt_s;
    int pivot_integrate;
} SampleTiming;

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

static DK1HeadNeckConfig default_head_neck_config(void) {
    return (DK1HeadNeckConfig){
        DK1_DEFAULT_HEAD_NECK_H_M,
        DK1_DEFAULT_HEAD_NECK_ELL_M,
        (double)DK1_DEFAULT_IPD_MM * 0.001,
        DK1_DEFAULT_PIVOT_DAMPING_PER_SECOND,
        DK1_MAX_DT_S,
        DK1_DEFAULT_MAX_REPORT_SAMPLE_COUNT
    };
}

static void apply_head_neck_config(DK1Estimator *est, const DK1HeadNeckConfig *config) {
    est->head_neck_config = *config;
    est->head_model.neck_to_tracker = (DK1Vector3){0.0, config->h_m, -config->ell_m};
    est->head_model.neck_to_head_center = (DK1Vector3){0.0, config->h_m, 0.0};
    est->head_model.head_center_to_eye = (DK1Vector3){0.0, 0.0, -config->ell_m};
    est->head_model.ipd_m = config->ipd_m;
    est->head_model.look_dir_head = (DK1Vector3){0.0, 0.0, -1.0};
    est->state.pivot_damping_per_second = config->pivot_damping_per_second;
    est->state.timing_max_dt_s = config->max_dt_s;
}

static int head_neck_config_is_valid(const DK1HeadNeckConfig *config) {
    if (!config) return 0;
    if (config->h_m < 0.0 || !isfinite(config->h_m)) return 0;
    if (config->ell_m < 0.0 || !isfinite(config->ell_m)) return 0;
    if (config->ipd_m <= 0.0 || !isfinite(config->ipd_m)) return 0;
    if (
        config->pivot_damping_per_second < 0.0 ||
        !isfinite(config->pivot_damping_per_second)
    ) {
        return 0;
    }
    if (config->max_dt_s <= 0.0 || !isfinite(config->max_dt_s)) return 0;
    if (config->max_report_sample_count == 0) return 0;
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

static uint32_t north_window_size(const DK1Estimator *est) {
    uint32_t window_size = est->mag_calibration.correction_interval_samples;
    if (window_size == 0) window_size = DK1_DEFAULT_YAW_CORRECTION_INTERVAL;
    if (window_size > DK1_ESTIMATOR_MAX_NORTH_WINDOW_SAMPLES) {
        window_size = DK1_ESTIMATOR_MAX_NORTH_WINDOW_SAMPLES;
    }
    return window_size;
}

static void update_north_window_state(DK1Estimator *est) {
    if (est->north_window_count == 0) return;

    DK1Vector3 observed = vec3_normalize_or(
        est->north_window_sum,
        est->state.expected_north_world
    );
    est->state.observed_north_world = observed;

    if (!est->state.expected_north_initialized) {
        est->state.expected_north_world = observed;
        est->state.expected_north_initialized = 1;
    }

    double error = signed_heading_error_rad(est->state.expected_north_world, observed);
    est->state.heading_residual_deg = radians_to_degrees(error);
    est->state.north_window_sample_count = est->north_window_count;
}

static void reset_north_window(DK1Estimator *est) {
    memset(est->north_window, 0, sizeof(est->north_window));
    est->north_window_sum = (DK1Vector3){0.0, 0.0, 0.0};
    est->north_window_index = 0;
    est->north_window_count = 0;
    est->state.north_window_sample_count = 0;
    est->state.observed_north_world = est->state.expected_north_world;
}

static void rotate_north_window(DK1Estimator *est, DK1Quaternion rotation) {
    for (uint32_t i = 0; i < est->north_window_count; ++i) {
        est->north_window[i] = dk1_quat_rotate_vec3(rotation, est->north_window[i]);
    }
    est->north_window_sum = dk1_quat_rotate_vec3(rotation, est->north_window_sum);
    update_north_window_state(est);
}

static void update_north_window(DK1Estimator *est, DK1Vector3 mag_calibrated) {
    uint32_t window_size = north_window_size(est);
    DK1Vector3 observed = {0.0, 0.0, 0.0};
    if (!observed_north_world(
            est->orientation,
            mag_calibrated,
            est->state.expected_north_world,
            &observed
        )) {
        return;
    }

    if (est->north_window_count < window_size) {
        est->north_window[est->north_window_index] = observed;
        est->north_window_sum = vec3_add(est->north_window_sum, observed);
        est->north_window_count++;
    } else {
        DK1Vector3 old = est->north_window[est->north_window_index];
        est->north_window[est->north_window_index] = observed;
        est->north_window_sum = vec3_add(vec3_sub(est->north_window_sum, old), observed);
    }

    est->north_window_index++;
    if (est->north_window_index >= window_size) {
        est->north_window_index = 0;
    }

    update_north_window_state(est);
}

static DK1Vector3 predicted_specific_force(DK1Quaternion orientation) {
    return vec3_scale(quat_rotate_inverse(orientation, DK1_G_WORLD), -1.0);
}

static double configured_max_dt(const DK1Estimator *est) {
    double max_dt = est->head_neck_config.max_dt_s;
    if (max_dt <= 0.0 || !isfinite(max_dt)) return DK1_MAX_DT_S;
    return max_dt;
}

static double clamp_dt_configured(DK1Estimator *est, double dt) {
    double max_dt = configured_max_dt(est);
    if (dt <= 0.0 || !isfinite(dt)) return DK1_DEFAULT_SAMPLE_DT_S;
    if (dt > max_dt) return max_dt;
    return dt;
}

static void update_report_group_timing(DK1Estimator *est, const DK1Sample *sample) {
    if (est->timing_have_group && sample->timestamp == est->timing_last_group_timestamp) {
        return;
    }

    est->timing_have_group = 1;
    est->timing_last_group_timestamp = sample->timestamp;
    est->state.timing_report_group_count++;
    est->state.timing_last_report_sample_count = sample->sample_count;
    if (sample->sample_count > est->state.timing_max_report_sample_count) {
        est->state.timing_max_report_sample_count = sample->sample_count;
    }
    if (sample->sample_count > 1) {
        est->state.timing_repeated_group_count++;
    }
    if (sample->sample_count > est->head_neck_config.max_report_sample_count) {
        est->state.timing_oversized_group_count++;
        est->current_report_pivot_skip = 1;
    } else {
        est->current_report_pivot_skip = 0;
    }
}

static SampleTiming estimator_sample_timing(DK1Estimator *est, const DK1Sample *sample) {
    double fallback = est->last_report_sample_dt_s > 0.0
        ? est->last_report_sample_dt_s
        : DK1_DEFAULT_SAMPLE_DT_S;
    SampleTiming timing = {
        fallback,
        clamp_dt_configured(est, fallback),
        !est->current_report_pivot_skip
    };

    if (sample->timestamp != est->last_report_timestamp) {
        uint16_t delta_ticks = (uint16_t)(sample->timestamp - est->last_report_timestamp);
        uint8_t report_sample_count = sample->sample_count ? sample->sample_count : 1;
        double report_dt = (double)delta_ticks * DK1_DEVICE_TIMESTAMP_TICK_S;
        double sample_dt = report_dt / (double)report_sample_count;
        timing.raw_dt_s = sample_dt;

        if (sample_dt <= 0.0 || !isfinite(sample_dt)) {
            est->state.timing_nonpositive_dt_count++;
            timing.dt_s = DK1_DEFAULT_SAMPLE_DT_S;
            timing.pivot_integrate = 0;
        } else {
            double max_dt = configured_max_dt(est);
            timing.dt_s = sample_dt;
            if (sample_dt > max_dt) {
                est->state.timing_capped_dt_count++;
                timing.dt_s = max_dt;
                timing.pivot_integrate = 0;
            }
        }

        if (est->current_report_pivot_skip) {
            timing.pivot_integrate = 0;
        }

        est->last_report_sample_dt_s = timing.dt_s;
        est->last_report_raw_dt_s = sample_dt;
        est->state.timing_last_raw_dt_s = sample_dt;
        if (sample_dt > est->max_raw_dt_s) {
            est->max_raw_dt_s = sample_dt;
            est->state.timing_max_raw_dt_s = sample_dt;
        }
        est->last_report_timestamp = sample->timestamp;
        return timing;
    }

    if (est->current_report_pivot_skip) {
        timing.pivot_integrate = 0;
    }
    est->state.timing_last_raw_dt_s = fallback;
    return timing;
}

static void maybe_apply_yaw_correction(DK1Estimator *est) {
    const DK1MagCalibration *calibration = &est->mag_calibration;
    uint32_t interval = north_window_size(est);

    if (calibration->correction_rate <= 0.0) return;
    if (!est->state.expected_north_initialized) return;
    if (est->north_window_count < interval) return;
    if (est->state.sample_index % (uint64_t)interval != 0u) return;

    double error = signed_heading_error_rad(
        est->state.expected_north_world,
        est->state.observed_north_world
    );
    double correction_fraction = 1.0 - exp(-calibration->correction_rate);
    if (!isfinite(correction_fraction)) correction_fraction = 1.0;
    if (correction_fraction < 0.0) correction_fraction = 0.0;
    if (correction_fraction > 1.0) correction_fraction = 1.0;

    double heading_step = error * correction_fraction;
    DK1Quaternion correction = quat_from_axis_angle(DK1_WORLD_UP, -heading_step);

    est->orientation = quat_normalize(quat_mul(correction, est->orientation));
    rotate_north_window(est, correction);
    est->state.heading_residual_deg = radians_to_degrees(error - heading_step);
    est->state.mag_correction_update_count++;
}

static void reset_pivot_integration(DK1Estimator *est) {
    est->pivot_velocity_world = (DK1Vector3){0.0, 0.0, 0.0};
    est->pivot_position_world = (DK1Vector3){0.0, 0.0, 0.0};
    est->previous_pivot_accel_world = (DK1Vector3){0.0, 0.0, 0.0};
    est->have_previous_pivot_accel = 0;
    est->state.pivot_velocity_world = est->pivot_velocity_world;
    est->state.pivot_position_world = est->pivot_position_world;
    est->state.pivot_position_reliable = 1;
}

static void integrate_pivot_position(
    DK1Estimator *est,
    DK1Vector3 pivot_accel_world,
    double dt,
    int integrate
) {
    if (!est->have_previous_pivot_accel) {
        est->previous_pivot_accel_world = pivot_accel_world;
        est->have_previous_pivot_accel = 1;
        est->state.pivot_velocity_world = est->pivot_velocity_world;
        est->state.pivot_position_world = est->pivot_position_world;
        est->state.pivot_position_reliable = 1;
        return;
    }

    if (!integrate || dt <= 0.0 || !isfinite(dt)) {
        est->previous_pivot_accel_world = pivot_accel_world;
        est->state.pivot_velocity_world = est->pivot_velocity_world;
        est->state.pivot_position_world = est->pivot_position_world;
        est->state.pivot_position_reliable = 0;
        est->state.pivot_integration_skipped_count++;
        return;
    }

    DK1Vector3 previous_velocity = est->pivot_velocity_world;
    DK1Vector3 accel_avg = vec3_scale(
        vec3_add(est->previous_pivot_accel_world, pivot_accel_world),
        0.5
    );

    double damping = est->head_neck_config.pivot_damping_per_second;
    if (damping > 0.0) {
        double half_damping_dt = 0.5 * damping * dt;
        DK1Vector3 numerator = vec3_add(
            vec3_scale(previous_velocity, 1.0 - half_damping_dt),
            vec3_scale(accel_avg, dt)
        );
        est->pivot_velocity_world = vec3_scale(
            numerator,
            1.0 / (1.0 + half_damping_dt)
        );
    } else {
        est->pivot_velocity_world = vec3_add(
            previous_velocity,
            vec3_scale(accel_avg, dt)
        );
    }

    est->pivot_position_world = vec3_add(
        est->pivot_position_world,
        vec3_scale(
            vec3_add(previous_velocity, est->pivot_velocity_world),
            0.5 * dt
        )
    );

    est->previous_pivot_accel_world = pivot_accel_world;
    est->state.pivot_velocity_world = est->pivot_velocity_world;
    est->state.pivot_position_world = est->pivot_position_world;
    est->state.pivot_position_reliable = 1;
}

static void update_derived_state(
    DK1Estimator *est,
    const DK1Sample *sample,
    DK1Vector3 unbiased_gyro,
    DK1Vector3 angular_accel,
    DK1Vector3 mag_calibrated,
    double dt,
    int integrate_pivot
) {
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
    est->state.predicted_accel = predicted_specific_force(est->orientation);
    est->state.accel_residual = vec3_sub(sample->accel, est->state.predicted_accel);
    est->state.pivot_accel_body = vec3_sub(est->state.accel_residual, tracker_rot_accel);
    est->state.pivot_accel_world = dk1_quat_rotate_vec3(
        est->orientation,
        est->state.pivot_accel_body
    );
    integrate_pivot_position(est, est->state.pivot_accel_world, dt, integrate_pivot);

    est->state.eye_center_world = vec3_add(
        est->pivot_position_world,
        dk1_quat_rotate_vec3(est->orientation, eye_center_body)
    );
    dk1_head_model_eye_positions_world(
        &est->head_model,
        est->orientation,
        est->pivot_position_world,
        &est->state.left_eye_world,
        &est->state.right_eye_world
    );

    est->state.mag_calibrated = mag_calibrated;
    est->state.mag_correction_rate = est->mag_calibration.correction_rate;
    est->state.mag_correction_interval_samples =
        est->mag_calibration.correction_interval_samples;
    est->state.pivot_damping_per_second =
        est->head_neck_config.pivot_damping_per_second;
    est->state.timing_max_dt_s = configured_max_dt(est);
}

void dk1_estimator_init(DK1Estimator *est) {
    if (!est) return;

    memset(est, 0, sizeof(*est));
    est->orientation = (DK1Quaternion){1.0, 0.0, 0.0, 0.0};
    est->state.orientation = est->orientation;
    est->state.expected_north_world = (DK1Vector3){1.0, 0.0, 0.0};
    est->state.observed_north_world = est->state.expected_north_world;
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
    DK1HeadNeckConfig head_neck_config = default_head_neck_config();
    apply_head_neck_config(est, &head_neck_config);
    reset_pivot_integration(est);
}

void dk1_estimator_update(DK1Estimator *est, const DK1Sample *sample) {
    if (!est || !sample) return;

    DK1Vector3 mag_calibrated = calibrate_magnetometer(sample->mag, &est->mag_calibration);
    DK1Vector3 unbiased_gyro = vec3_sub(sample->gyro, est->gyro_bias);
    update_report_group_timing(est, sample);

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
        update_north_window(est, mag_calibrated);
        update_derived_state(
            est,
            sample,
            unbiased_gyro,
            (DK1Vector3){0.0, 0.0, 0.0},
            mag_calibrated,
            0.0,
            0
        );
        return;
    }

    SampleTiming timing = estimator_sample_timing(est, sample);
    double dt = timing.dt_s;
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

    update_north_window(est, mag_calibrated);
    maybe_apply_yaw_correction(est);
    update_derived_state(
        est,
        sample,
        unbiased_gyro,
        angular_accel,
        mag_calibrated,
        dt,
        timing.pivot_integrate
    );

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
    if (
        calibration->correction_interval_samples == 0 ||
        calibration->correction_interval_samples > DK1_ESTIMATOR_MAX_NORTH_WINDOW_SAMPLES
    ) {
        return DK1_ERROR_INVALID_ARGUMENT;
    }

    est->mag_calibration = *calibration;
    est->state.mag_correction_rate = calibration->correction_rate;
    est->state.mag_correction_interval_samples =
        calibration->correction_interval_samples;
    reset_north_window(est);
    return DK1_OK;
}

int dk1_estimator_set_head_neck_config(
    DK1Estimator *est,
    const DK1HeadNeckConfig *config
) {
    if (!est || !config) return DK1_ERROR_INVALID_ARGUMENT;
    if (!head_neck_config_is_valid(config)) return DK1_ERROR_INVALID_ARGUMENT;

    apply_head_neck_config(est, config);
    reset_pivot_integration(est);
    DK1Vector3 eye_center_body = dk1_head_model_neck_to_eye_center(&est->head_model);
    est->state.look_dir_world = dk1_head_model_looking_direction_world(
        &est->head_model,
        est->orientation
    );
    est->state.eye_center_world = vec3_add(
        est->pivot_position_world,
        dk1_quat_rotate_vec3(est->orientation, eye_center_body)
    );
    dk1_head_model_eye_positions_world(
        &est->head_model,
        est->orientation,
        est->pivot_position_world,
        &est->state.left_eye_world,
        &est->state.right_eye_world
    );
    est->state.pivot_damping_per_second =
        est->head_neck_config.pivot_damping_per_second;
    est->state.timing_max_dt_s = configured_max_dt(est);
    return DK1_OK;
}

void dk1_estimator_get_head_neck_config(
    const DK1Estimator *est,
    DK1HeadNeckConfig *out_config
) {
    if (!est || !out_config) return;
    *out_config = est->head_neck_config;
}
