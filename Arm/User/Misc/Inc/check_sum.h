#ifndef CHECK_SUM_H
#define CHECK_SUM_H

#include <stdint.h>

uint8_t generate_check_sum(uint8_t *packet, uint32_t data_length);

#endif