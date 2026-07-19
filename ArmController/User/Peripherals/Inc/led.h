/** 
 * @brief Control LEDs
 */
#ifndef LED_H_
#define LED_H_

//////////* Headers *//////////
/* STD */
#include <stdint.h>
#include <stdbool.h>
/* ArmController */
//// Task
#include "led_task.h"
//// Global
#include "global.h"
///////////////////////////////

typedef enum {
    _LED,
    _LED1,
} LED_ID;

// Initialize
void initialize_led();

// Task
Res add_led_task(LEDTask task);
bool check_led_task_finished(LEDTask *task);
void execute_led_task();

// Turn LED on/off 
void turn_led_on(uint8_t id);
void turn_led_off(uint8_t id);

// Test
void test_led1(void);

#endif 