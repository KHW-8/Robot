#ifndef RESPONSETYPE_H
#define RESPONSETYPE_H

#include <stdint.h>

#pragma pack(1)

typedef struct {
    uint8_t cmd;
    uint8_t result;
    uint8_t servo_count;
    struct {
        uint8_t servo_id;
        int16_t angle;
    } servos[];
} BusServoAngleResponse;

#pragma pack(0)

#endif