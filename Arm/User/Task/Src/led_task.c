#include "led_task.h"

void initialize_led_task(LEDTask *task) {
    task->led_count = 0;
}

void initialize_led_task_queue(LEDTaskQueue *queue) {
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;

    queue->isEmpty = is_led_task_queue_empty;
    queue->isFull = is_led_task_queue_full;
    queue->push = push_item_into_led_task_queue;
    queue->pop = pop_item_from_led_task_queue;
    queue->peak = led_task_queue_peak;
}


bool is_led_task_queue_empty(LEDTaskQueue *queue) {
    return queue->size == 0;
}

bool is_led_task_queue_full(LEDTaskQueue *queue) {
    return queue->size == LED_TASK_QUEUE_CAPACITY;
}

bool push_item_into_led_task_queue(LEDTaskQueue *queue, LEDTask task) {
    if (queue->isFull(queue))
        return false;

    queue->items[queue->tail] = task;
    queue->tail = (queue->tail + 1) % LED_TASK_QUEUE_CAPACITY;
    queue->size++;

    return true;
}

bool pop_item_from_led_task_queue(LEDTaskQueue *queue, LEDTask *task) {
    if (queue->isEmpty(queue)) 
        return false;

    if (task != NULL)
        *task = queue->items[queue->head];
    queue->head = (queue->head + 1) % LED_TASK_QUEUE_CAPACITY;
    queue->size--;

    return true;
}

bool led_task_queue_peak(LEDTaskQueue *queue, LEDTask *task) {
    if (queue->isEmpty(queue)) 
        return false;

    *task = queue->items[queue->head];

    return true;
}