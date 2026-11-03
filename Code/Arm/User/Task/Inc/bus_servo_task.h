#ifndef BUS_SERVO_TASK_H
#define BUS_SERVO_TASK_H

#include <stdint.h>
#include <stdbool.h>

/////* Macro */////
#define BUS_SERVO_COUNT 6
#define BUS_SERVO_TASK_QUEUE_CAPACITY 10
///////////////////

typedef enum {
    BUS_SERVO_READY_MODE,
    BUS_SERVO_SUSPEND_MODE
} BusServoTaskState;

typedef struct {
    uint8_t cmd;
    uint8_t servo_count;
    bool read_only;
    struct {
        uint8_t servo_id;
        uint8_t angle;
        uint16_t duration;

        BusServoTaskState state;
    } servos[BUS_SERVO_COUNT];
} BusServoTask;

typedef struct _BusServoTaskQueue BusServoTaskQueue;
struct _BusServoTaskQueue{
    // Member
    int32_t head;
    int32_t tail; 
    uint32_t size;

    BusServoTask items[BUS_SERVO_TASK_QUEUE_CAPACITY];

    // Function
    bool (*isEmpty)(BusServoTaskQueue *self);

    bool (*isFull)(BusServoTaskQueue *self);

    bool (*push)(BusServoTaskQueue *queue, BusServoTask task);

    bool (*pop)(BusServoTaskQueue *queue, BusServoTask *task);

    bool (*peak)(BusServoTaskQueue *queue, BusServoTask *task);
};

void initialize_bus_servo_task_queue(BusServoTaskQueue *queue);

bool is_bus_servo_task_queue_empty(BusServoTaskQueue *queue);

bool is_bus_servo_task_queue_full(BusServoTaskQueue *queue);

bool push_item_into_bus_servo_task_queue(BusServoTaskQueue *queue, BusServoTask task);

bool pop_item_from_bus_servo_task_queue(BusServoTaskQueue *queue, BusServoTask *task);

bool bus_servo_task_queue_peak(BusServoTaskQueue *queue, BusServoTask *task);

#endif