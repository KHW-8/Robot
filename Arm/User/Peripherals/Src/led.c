#include "led.h"

//////////* Headers *//////////
/* ArmController */
//// Core
#include "main.h"
//// User
// Host
#include "host.h"
///////////////////////////////

static LEDTaskQueue task_queue;

static LEDTask current_task;

static uint32_t last_tick = 0;

/** 
 * @brief 
 * @param
 *      @arg task
 * @retval
 */
Res add_led_task(LEDTask task) {
    if (task_queue.isFull(&task_queue) == true)
        return ERR;

    if (task_queue.push(&task_queue, task) == false)
        return ERR;

    return OK;
}

/** 
 * @brief 
 * @param
 *      @arg task
 */
bool check_led_task_finished(LEDTask *task) {
    for (uint8_t i = 0; i < task->led_count; i++) {
        if (task->leds[i].state != LED_IDLE_MODE)
            return false;
    }

    return true;
}

/** 
 * @brief 
 */
void execute_led_task() {
    uint32_t current_tick = HAL_GetTick();
    uint32_t duration = current_tick - last_tick;

    if (duration <= 0) {
        last_tick = current_tick;
        return;
    }

    if (check_led_task_finished(&current_task)) {
        if (task_queue.pop(&task_queue, &current_task) == false)
            return;
    }

    for (uint8_t i = 0; i < current_task.led_count; i++) {
        switch (current_task.leds[i].state) {
            case READY_TO_TURN_ON_LED: {
                if (current_task.leds[i].repeat_count > 0) {
                    if (current_task.leds[i].on_duration > 0) { // LED on duration is great than 0
                        turn_led_on(current_task.leds[i].led_id);
                        
                        current_task.leds[i].state = LED_ON;
                    } else { // LED on duration is less than or equal to 0
                        turn_led_off(current_task.leds[i].led_id);

                        current_task.leds[i].state = LED_OFF;
                    }
                } else {
                    turn_led_off(current_task.leds[i].led_id);

                    current_task.leds[i].state = LED_IDLE_MODE;
                }

                current_task.leds[i].tick_count = 0;
            } break;
            case LED_ON: {
                current_task.leds[i].tick_count += duration;

                if (current_task.leds[i].tick_count >= current_task.leds[i].on_duration) {
                    turn_led_off(current_task.leds[i].led_id);

                    current_task.leds[i].tick_count = 0;
                    current_task.leds[i].state = LED_OFF;
                }
            } break;
            case LED_OFF:
                current_task.leds[i].tick_count += duration;

                if (current_task.leds[i].tick_count >= current_task.leds[i].off_duration) {
                    current_task.leds[i].tick_count = 0;
                    current_task.leds[i].state = COMPLETE_A_LED_BLINK;
                }
                break;
            case COMPLETE_A_LED_BLINK: {
                current_task.leds[i].repeat_count--;

                if (current_task.leds[i].repeat_count != 0) 
                    current_task.leds[i].state = READY_TO_TURN_ON_LED;
                else
                    current_task.leds[i].state = LED_IDLE_MODE;
            } break;
            case LED_IDLE_MODE:
                break;
            default:
                break;
        }
    }
        
    last_tick = current_tick;
}

void turn_led_on(uint8_t id) {
    switch (id) {
        case _LED: break;
        case _LED1: HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); break;
        default: break;
    }
}

void turn_led_off(uint8_t id) {
    switch (id) {
        case _LED: break;
        case _LED1: HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); break;
        default: break;
    }
}


void initialize_led() {
    initialize_led_task_queue(&task_queue);

    turn_led_off(_LED);
    turn_led_off(_LED1);
}