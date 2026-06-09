#ifndef DK1_CONFIG_H
#define DK1_CONFIG_H

#include "DK1Tracker/DK1Types.h"
#include <stddef.h>

void dk1_config_set_defaults(DK1Config *config);
int dk1_config_validate(const DK1Config *config);
int dk1_config_default_path(char *buffer, size_t buffer_size);
int dk1_config_load_path(const char *path, DK1Config *out_config);
int dk1_config_load_default(DK1Config *out_config);

#endif // DK1_CONFIG_H
