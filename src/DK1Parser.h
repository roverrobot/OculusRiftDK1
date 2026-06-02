#ifndef DK1_PARSER_H
#define DK1_PARSER_H

#include "DK1Tracker/DK1Types.h"
#include <stdint.h>
#include <stddef.h>

int dk1_parse_input_report(
    const uint8_t *data,
    size_t length,
    DK1Sample *samples,
    size_t max_samples,
    size_t *out_count
);

#endif
