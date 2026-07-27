#include "buzzer.h"

#include "main.h"

static BuzzerTaskQueue task_queue;

static BuzzerTask current_task;

static uint32_t last_tick = 0;

void test_buzzer(void) {
    // Beep
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);

    //  Mute
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
}

void initialize_buzzer(void) {
    turn_buzzer_off();
}

Res add_buzzer_task(BuzzerTask task) {
    if (task_queue.isFull(&task_queue) == true)
        return ERR;

    if (task_queue.push(&task_queue, task) == false)
        return ERR;

    return OK;
}

bool check_buzzer_task_finished(BuzzerTask *task) {
    return task->state == BUZZER_IDLE_MODE;
}

void execute_buzzer_task() {
    uint32_t current_tick = HAL_GetTick();
    uint32_t duration = current_tick - last_tick;

    if (duration <= 0) {
        last_tick = current_tick;
        return;
    }

    if (check_buzzer_task_finished(&current_task)) {
        if (task_queue.pop(&task_queue, &current_task) == false)
            return;
    }

    switch (current_task.state) {
        case READY_TO_TURN_ON_BUZZER: {
            if (current_task.repeat_count > 0) {
                if (current_task.on_duration > 0) { // Buzzer on duration is great than 0
                    turn_buzzer_on();
                    
                    current_task.state = BUZZER_ON;
                } else { // Buzzer on duration is less than or equal to 0
                    turn_buzzer_off();

                    current_task.state = BUZZER_OFF;
                }
            } else {
                turn_buzzer_off();

                current_task.state = BUZZER_IDLE_MODE;
            }

            current_task.tick_count = 0;
        } break;
        case BUZZER_ON: {
            current_task.tick_count += duration;

            if (current_task.tick_count >= current_task.on_duration) {
                    turn_buzzer_off();

                current_task.tick_count = 0;
                current_task.state = BUZZER_OFF;
            }
        } break;
        case BUZZER_OFF:
            current_task.tick_count += duration;

            if (current_task.tick_count >= current_task.off_duration) {
                current_task.tick_count = 0;
                current_task.state = COMPLETE_A_BUZZER_OPERATION;
            }
            break;
        case COMPLETE_A_BUZZER_OPERATION: {
            current_task.repeat_count--;

            if (current_task.repeat_count != 0) 
                current_task.state = READY_TO_TURN_ON_BUZZER;
            else
                current_task.state = BUZZER_IDLE_MODE;
        } break;
        case BUZZER_IDLE_MODE:
            break;
        default:
            break;
    }
        
    last_tick = current_tick;
}

void turn_buzzer_on() {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

void turn_buzzer_off() {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}