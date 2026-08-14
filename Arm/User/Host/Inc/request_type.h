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
        uint32_t duration;
    } servos[];
} BusServoAngleSettingRequest;

/* Buzzer */
typedef struct {
    uint16_t frequency;
    uint32_t on_duration;   // millisecond
    uint32_t off_duration;  // millisecond
    uint16_t repeat_count;
} BuzzerRequest;

/* LED */
typedef struct {
    uint8_t led_count;
    struct {
        uint8_t led_id;
        uint32_t on_duration;   // millisecond
        uint32_t off_duration;  // millisecond
        uint16_t repeat_count;
    } leds[];
} LEDRequest;

#pragma pack()


#endif