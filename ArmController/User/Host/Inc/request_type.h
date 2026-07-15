#ifndef REQUREST_TYPE_H
#define REQUREST_TYPE_H

#include <stdint.h>

#pragma pack(1)

typedef struct {
    uint8_t cmd;
    uint8_t servo_count;
    uint8_t servos_id[];
} BusServoAngleQueryRequest;

typedef struct {
    uint8_t cmd;
    uint8_t servo_count;
    struct {
        uint8_t servo_id;
        int16_t angle;
        uint16_t duration;
    } servos[];
} BusServoAngleSettingRequest;

#pragma pack(0)


#endif