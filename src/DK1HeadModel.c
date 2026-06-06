#include "DK1HeadModel.h"

#include <math.h>

static DK1Vector3 vec3_add(DK1Vector3 a, DK1Vector3 b) {
    return (DK1Vector3){
        a.x + b.x,
        a.y + b.y,
        a.z + b.z
    };
}

static DK1Vector3 vec3_scale(DK1Vector3 v, double scale) {
    return (DK1Vector3){
        v.x * scale,
        v.y * scale,
        v.z * scale
    };
}

static DK1Vector3 vec3_cross(DK1Vector3 a, DK1Vector3 b) {
    return (DK1Vector3){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

void dk1_head_model_init_default(DK1HeadModel *model) {
    if (!model) return;

    /* Placeholder Gamma-shaped geometry in meters. With the y-up convention,
     * positive y is above the neck pivot and negative z is the provisional
     * looking direction. These defaults should be fitted or user-configured
     * later. */
    model->neck_to_tracker = (DK1Vector3){0.0, 0.10, -0.16};
    model->neck_to_head_center = (DK1Vector3){0.0, 0.10, 0.0};
    model->head_center_to_eye = (DK1Vector3){0.0, 0.0, -0.16};
    model->ipd_m = 0.064;
    model->look_dir_head = (DK1Vector3){0.0, 0.0, -1.0};
}

DK1Vector3 dk1_head_model_neck_to_eye_center(
    const DK1HeadModel *model
) {
    if (!model) return (DK1Vector3){0.0, 0.0, 0.0};

    return vec3_add(model->neck_to_head_center, model->head_center_to_eye);
}

void dk1_head_model_eye_offsets(
    const DK1HeadModel *model,
    DK1Vector3 *out_left_eye_from_neck,
    DK1Vector3 *out_right_eye_from_neck
) {
    if (!model) {
        if (out_left_eye_from_neck) {
            *out_left_eye_from_neck = (DK1Vector3){0.0, 0.0, 0.0};
        }
        if (out_right_eye_from_neck) {
            *out_right_eye_from_neck = (DK1Vector3){0.0, 0.0, 0.0};
        }
        return;
    }

    DK1Vector3 eye_center = dk1_head_model_neck_to_eye_center(model);
    DK1Vector3 half_ipd = (DK1Vector3){model->ipd_m * 0.5, 0.0, 0.0};

    if (out_left_eye_from_neck) {
        *out_left_eye_from_neck = vec3_add(eye_center, vec3_scale(half_ipd, -1.0));
    }
    if (out_right_eye_from_neck) {
        *out_right_eye_from_neck = vec3_add(eye_center, half_ipd);
    }
}

DK1Vector3 dk1_quat_rotate_vec3(
    DK1Quaternion q,
    DK1Vector3 v
) {
    double norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (norm_sq == 0.0) return v;

    double inv_norm = 1.0 / sqrt(norm_sq);
    q.w *= inv_norm;
    q.x *= inv_norm;
    q.y *= inv_norm;
    q.z *= inv_norm;

    DK1Vector3 qv = (DK1Vector3){q.x, q.y, q.z};
    DK1Vector3 t = vec3_scale(vec3_cross(qv, v), 2.0);
    return vec3_add(v, vec3_add(vec3_scale(t, q.w), vec3_cross(qv, t)));
}

void dk1_head_model_eye_positions_world(
    const DK1HeadModel *model,
    DK1Quaternion orientation,
    DK1Vector3 neck_position_world,
    DK1Vector3 *out_left_eye_world,
    DK1Vector3 *out_right_eye_world
) {
    DK1Vector3 left_eye_from_neck = {0.0, 0.0, 0.0};
    DK1Vector3 right_eye_from_neck = {0.0, 0.0, 0.0};
    dk1_head_model_eye_offsets(
        model,
        &left_eye_from_neck,
        &right_eye_from_neck
    );

    if (out_left_eye_world) {
        *out_left_eye_world = vec3_add(
            neck_position_world,
            dk1_quat_rotate_vec3(orientation, left_eye_from_neck)
        );
    }
    if (out_right_eye_world) {
        *out_right_eye_world = vec3_add(
            neck_position_world,
            dk1_quat_rotate_vec3(orientation, right_eye_from_neck)
        );
    }
}

DK1Vector3 dk1_head_model_looking_direction_world(
    const DK1HeadModel *model,
    DK1Quaternion orientation
) {
    DK1Vector3 look_dir = model
        ? model->look_dir_head
        : (DK1Vector3){0.0, 0.0, -1.0};
    return dk1_quat_rotate_vec3(orientation, look_dir);
}

DK1Vector3 dk1_head_model_tracker_rotational_accel(
    const DK1HeadModel *model,
    DK1Vector3 omega,
    DK1Vector3 alpha
) {
    if (!model) return (DK1Vector3){0.0, 0.0, 0.0};

    DK1Vector3 r = model->neck_to_tracker;
    DK1Vector3 tangential = vec3_cross(alpha, r);
    DK1Vector3 centripetal = vec3_cross(omega, vec3_cross(omega, r));
    return vec3_add(tangential, centripetal);
}
