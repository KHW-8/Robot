#include "buzzer_task.h"

void initialize_buzzer_task_queue(BuzzerTaskQueue *queue) {
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;

    queue->isEmpty = is_buzzer_task_queue_empty;
    queue->isFull = is_buzzer_task_queue_full;
    queue->push = push_item_into_buzzer_task_queue;
    queue->pop = pop_item_from_buzzer_task_queue;
    queue->peak = buzzer_task_queue_peak;
}


bool is_buzzer_task_queue_empty(BuzzerTaskQueue *queue) {
    return queue->size == 0;
}

bool is_buzzer_task_queue_full(BuzzerTaskQueue *queue) {
    return queue->size == BUZZER_TASK_QUEUE_CAPACITY;
}

bool push_item_into_buzzer_task_queue(BuzzerTaskQueue *queue, BuzzerTask task) {
    if (queue->isFull(queue))
        return false;

    queue->items[queue->tail] = task;
    queue->tail = (queue->tail + 1) % BUZZER_TASK_QUEUE_CAPACITY;
    queue->size++;

    return true;
}

bool pop_item_from_buzzer_task_queue(BuzzerTaskQueue *queue, BuzzerTask *task) {
    if (queue->isEmpty(queue)) 
        return false;

    *task = queue->items[queue->head];
    queue->head = (queue->head + 1) % BUZZER_TASK_QUEUE_CAPACITY;
    queue->size--;

    return true;
}

bool buzzer_task_queue_peak(BuzzerTaskQueue *queue, BuzzerTask *task) {
    if (queue->isEmpty(queue)) 
        return false;

    *task = queue->items[queue->head];

    return true;
}