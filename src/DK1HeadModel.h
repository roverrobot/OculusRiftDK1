#ifndef DK1_HEAD_MODEL_H
#define DK1_HEAD_MODEL_H

#include "DK1Tracker/DK1Types.h"

typedef struct DK1HeadModel {
    DK1Vector3 neck_to_tracker;
    DK1Vector3 neck_to_head_center;
    DK1Vector3 head_center_to_eye;
    double ipd_m;
    DK1Vector3 look_dir_head;
} DK1HeadModel;

void dk1_head_model_init_default(DK1HeadModel *model);

DK1Vector3 dk1_head_model_neck_to_eye_center(
    const DK1HeadModel *model
);

void dk1_head_model_eye_offsets(
    const DK1HeadModel *model,
    DK1Vector3 *out_left_eye_from_neck,
    DK1Vector3 *out_right_eye_from_neck
);

DK1Vector3 dk1_quat_rotate_vec3(
    DK1Quaternion q,
    DK1Vector3 v
);

void dk1_head_model_eye_positions_world(
    const DK1HeadModel *model,
    DK1Quaternion orientation,
    DK1Vector3 neck_position_world,
    DK1Vector3 *out_left_eye_world,
    DK1Vector3 *out_right_eye_world
);

DK1Vector3 dk1_head_model_looking_direction_world(
    const DK1HeadModel *model,
    DK1Quaternion orientation
);

DK1Vector3 dk1_head_model_tracker_rotational_accel(
    const DK1HeadModel *model,
    DK1Vector3 omega,
    DK1Vector3 alpha
);

#endif
