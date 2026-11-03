#include "bus_servo_task.h"

void initialize_bus_servo_task_queue(BusServoTaskQueue *queue) {
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;

    queue->isEmpty = is_bus_servo_task_queue_empty;
    queue->isFull = is_bus_servo_task_queue_full;
    queue->push = push_item_into_bus_servo_task_queue;
    queue->pop = pop_item_from_bus_servo_task_queue;
    queue->peak = bus_servo_task_queue_peak;
}


bool is_bus_servo_task_queue_empty(BusServoTaskQueue *queue) {
    return queue->size == 0;
}

bool is_bus_servo_task_queue_full(BusServoTaskQueue *queue) {
    return queue->size == BUS_SERVO_TASK_QUEUE_CAPACITY;
}

bool push_item_into_bus_servo_task_queue(BusServoTaskQueue *queue, BusServoTask task) {
    if (queue->isFull(queue))
        return false;

    queue->items[queue->tail] = task;
    queue->tail = (queue->tail + 1) % BUS_SERVO_TASK_QUEUE_CAPACITY;
    queue->size++;

    return true;
}

bool pop_item_from_bus_servo_task_queue(BusServoTaskQueue *queue, BusServoTask *task) {
    if (queue->isEmpty(queue)) 
        return false;

    *task = queue->items[queue->head];
    queue->head = (queue->head + 1) % BUS_SERVO_TASK_QUEUE_CAPACITY;
    queue->size--;

    return true;
}

bool bus_servo_task_queue_peak(BusServoTaskQueue *queue, BusServoTask *task) {
    if (queue->isEmpty(queue)) 
        return false;

    *task = queue->items[queue->head];

    return true;
}