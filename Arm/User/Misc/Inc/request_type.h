#ifndef REQUREST_TYPE_H
#define REQUREST_TYPE_H

#include <stdint.h>

#pragma pack(1)

/* BUS Servo */
typedef struct {
    uint8_t cmd;
    uint8_t servo_count;
    uint8_t servos_id[];
} BusServoQueryRequest;

typedef struct {
    uint8_t cmd;
    uint8_t servo_count;
    struct {
        uint8_t servo_id;
        uint8_t angle;
        float duration;
    } servos[];
} BusServoAngleSettingRequest;

/* Buzzer */
typedef struct {
    uint16_t frequency;
    float on_duration;   // millisecond
    float off_duration;  // millisecond
    uint16_t repeat_count;
} BuzzerRequest;

/* LED */
typedef struct {
    uint8_t led_count;
    struct {
        uint8_t led_id;
        float on_duration;   // millisecond
        float off_duration;  // millisecond
        uint16_t repeat_count;
    } leds[];
} LEDRequest;

#pragma pack()


#endif