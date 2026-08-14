#ifndef BUZZER_TASK_H
#define BUZZER_TASK_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Task */
#define BUZZER_TASK_QUEUE_CAPACITY 5

typedef enum {
    READY_TO_TURN_ON_BUZZER,
    BUZZER_ON,
    BUZZER_OFF,
    COMPLETE_A_BUZZER_OPERATION,
    BUZZER_IDLE_MODE
} BuzzerTaskState;

typedef struct {
    uint16_t frequency;
    uint32_t on_duration;   // millisecond
    uint32_t off_duration;  // millisecond
    uint16_t repeat_count;
    
    BuzzerTaskState state;
    uint32_t tick_count;
} BuzzerTask;

/* Task Queue*/
typedef struct _BuzzerTaskQueue BuzzerTaskQueue;
struct _BuzzerTaskQueue{
    // Member
    int32_t head;
    int32_t tail; 
    uint32_t size;

    BuzzerTask items[BUZZER_TASK_QUEUE_CAPACITY];

    // Function
    bool (*isEmpty)(BuzzerTaskQueue *self);

    bool (*isFull)(BuzzerTaskQueue *self);

    bool (*push)(BuzzerTaskQueue *queue, BuzzerTask task);

    bool (*pop)(BuzzerTaskQueue *queue, BuzzerTask *task);

    bool (*peak)(BuzzerTaskQueue *queue, BuzzerTask *task);
};

void initialize_buzzer_task_queue(BuzzerTaskQueue *queue);

bool is_buzzer_task_queue_empty(BuzzerTaskQueue *queue);

bool is_buzzer_task_queue_full(BuzzerTaskQueue *queue);

bool push_item_into_buzzer_task_queue(BuzzerTaskQueue *queue, BuzzerTask task);

bool pop_item_from_buzzer_task_queue(BuzzerTaskQueue *queue, BuzzerTask *task);

bool buzzer_task_queue_peak(BuzzerTaskQueue *queue, BuzzerTask *task);

#endif