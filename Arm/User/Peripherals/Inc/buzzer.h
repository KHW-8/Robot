/** 
 * @brief Control buzzers
 */
#ifndef BUZZER_H
#define BUZZER_H

//////////* Headers *//////////
/* STD */
/* Arm */
//// Task
#include "buzzer_task.h"
//// Global
#include "global.h"
///////////////////////////////

void test_buzzer(void);

// Initializing
void initialize_buzzer(void);

// Task
Res add_buzzer_task(BuzzerTask task);
bool check_buzzer_task_finished(BuzzerTask *task);
void execute_buzzer_task();

// Turn buzzer on/off
void turn_buzzer_on();
void turn_buzzer_off();

#endif