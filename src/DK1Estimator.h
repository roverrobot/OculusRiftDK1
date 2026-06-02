#ifndef DK1_ESTIMATOR_H
#define DK1_ESTIMATOR_H

#include "DK1Tracker/DK1Types.h"

typedef struct {
    DK1Quaternion orientation;
} DK1Estimator;

void dk1_estimator_init(DK1Estimator *est);
void dk1_estimator_update(DK1Estimator *est, const DK1Sample *sample);

#endif
