#ifndef LED_TASK_H
#define LED_TASK_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Task */
#define LED_COUNT 2
#define LED_TASK_QUEUE_CAPACITY 5

typedef enum {
    READY_TO_TURN_ON_LED,
    LED_ON,
    LED_OFF,
    COMPLETE_A_LED_BLINK,
    LED_IDLE_MODE
} LEDTaskState;

typedef struct {
    struct {
        uint8_t led_id;
        uint16_t on_duration;
        uint16_t off_duration;
        uint16_t repeat_count;
        
        LEDTaskState state;
        uint32_t tick_count;
    } leds[LED_COUNT];

    uint8_t led_count;
} LEDTask;

void initialize_led_task(LEDTask *task);


/* Task Queue */
typedef struct _LEDTaskQueue LEDTaskQueue;
struct _LEDTaskQueue{
    // Member
    int32_t head;
    int32_t tail; 
    uint32_t size;

    LEDTask items[LED_TASK_QUEUE_CAPACITY];

    // Function
    bool (*isEmpty)(LEDTaskQueue *self);

    bool (*isFull)(LEDTaskQueue *self);

    bool (*push)(LEDTaskQueue *queue, LEDTask task);

    bool (*pop)(LEDTaskQueue *queue, LEDTask *task);

    bool (*peak)(LEDTaskQueue *queue, LEDTask *task);
};

void initialize_led_task_queue(LEDTaskQueue *queue);

bool is_led_task_queue_empty(LEDTaskQueue *queue);

bool is_led_task_queue_full(LEDTaskQueue *queue);

bool push_item_into_led_task_queue(LEDTaskQueue *queue, LEDTask task);

bool pop_item_from_led_task_queue(LEDTaskQueue *queue, LEDTask *task);

bool led_task_queue_peak(LEDTaskQueue *queue, LEDTask *task);

#endif