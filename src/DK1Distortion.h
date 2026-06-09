#ifndef DK1_DISTORTION_H
#define DK1_DISTORTION_H

#include "DK1Tracker/DK1Types.h"

int dk1_distortion_mesh_build(
    DK1DistortionMesh *mesh,
    DK1Eye eye,
    const DK1Config *config
);

void dk1_distortion_mesh_destroy(DK1DistortionMesh *mesh);

#endif // DK1_DISTORTION_H
