#include "check_sum.h"

uint8_t generate_check_sum(uint8_t *packet, uint32_t data_length, uint32_t offset) {
    uint32_t sum = 0;

    for (uint32_t i = 0; i < data_length; i++)  
        sum += packet[i + offset];

    return (uint8_t)(~sum);
}