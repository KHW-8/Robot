#ifndef REQUREST_TYPE_H
#define REQUREST_TYPE_H

#include <stdint.h>

#pragma pack(1)

/* BUS Servo */
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

/* Buzzer */
typedef struct {
    uint16_t frequency;
    uint16_t on_duration;
    uint16_t off_duration;
    uint16_t repeat_count;
} BuzzerRequest;

/* LED */
typedef struct {
    uint8_t led_count;
    struct {
        uint8_t led_id;
        uint16_t on_duration;
        uint16_t off_duration;
        uint16_t repeat_count;
    } leds[];
} LEDRequest;

#pragma pack(0)


#endif