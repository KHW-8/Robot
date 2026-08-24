#include "core.h"

//////////* Headers *//////////
/* STD */
#include <stdio.h>
#include <string.h>
/* Arm */
//// Core
#include "usart.h"
//// User
// Core
#include "request_type.h"
// Peripherals
#include "bus_servo.h"
#include "buzzer.h"
#include "led.h"
//// Global
#include "global.h"
///////////////////////////////

//////////* Extern *//////////
///////////////////////////////

///////////////* Macro *///////////////
#define USART_CORE USART2
///////////////////////////////////////

///////////////* Global Variable *///////////////
CorePacketController core_packet_controller;

static UART_HandleTypeDef *huart_core = &huart2;

static uint8_t rx_buf[CORE_PACKET_DATA_MAX_LENGTH];
/////////////////////////////////////////////////


////////////////////* Functions *////////////////////

Res transmit_packet_to_core(CorePacket *packet) {
    uint8_t  packet_length = PACKET_HEADER_COUNT + 2 + packet->data_length;

    uint8_t *pPacket = (uint8_t*)packet;

    for (uint16_t i = 0; i < packet_length; i++) {
        // Wait until TDR is empty (TXE flag is set)
        uint32_t initial_tick = HAL_GetTick();
        while (!__HAL_UART_GET_FLAG(huart_core, UART_FLAG_TXE)) {
            if (HAL_GetTick() - initial_tick > 10) 
                return ERR;
        }
        
        huart_core->Instance->TDR = (pPacket[i] & 0xFF);
    }


    // Wait until TDR is empty (TC flag is set)
    uint32_t initial_tick = HAL_GetTick();
    while (!__HAL_UART_GET_FLAG(huart_core, UART_FLAG_TC)) {
        if (HAL_GetTick() - initial_tick > 10) 
            return ERR;
    }

    return OK;
}

void initialize_core_packet(CorePacket *packet, uint8_t peripheral) {
    packet->header1 = CORE_PACKET_HEADER;
    packet->header2 = CORE_PACKET_HEADER;
    packet->peripheral = peripheral;
    packet->data_length = 0;
    memset(packet->data, 0, sizeof(packet->data));
}

void initialize_core() {
    core_packet_controller.rx_finished = false;
    core_packet_controller.rx_state = OK;

    HAL_UARTEx_ReceiveToIdle_DMA(huart_core, rx_buf, CORE_PACKET_DATA_MAX_LENGTH);
}

Res transmit_byte_to_core(uint8_t byte) {
    HAL_UART_Transmit_DMA(huart_core, (uint8_t*)&byte, 1);

    return OK;
}

Res transmit_msg_to_core(const char *buf) {
    char msg[128];

    int len = snprintf(msg, sizeof(msg), "%s\r\n", buf);

    HAL_UART_Transmit(huart_core, (uint8_t*)msg, len, HAL_MAX_DELAY);

    return OK;
}

void receive_packet_from_core() {
    if (!core_packet_controller.rx_finished)
        return;

    if (core_packet_controller.rx_state == OK)
        handle_core_packet(&core_packet_controller.rx_packet);

    core_packet_controller.rx_finished = false;
    HAL_UARTEx_ReceiveToIdle_DMA(huart_core, rx_buf, CORE_PACKET_DATA_MAX_LENGTH);
}

/** 
 * @brief
 */
Res handle_core_rx_buffer(uint8_t packet_len) {
    // Write data from rx_buf to rx_packet
    uint8_t *pPacket = (uint8_t*)&core_packet_controller.rx_packet;

    for (uint32_t i = 0; i < packet_len; i++) 
        pPacket[i] = rx_buf[i];

    if (core_packet_controller.rx_packet.header1 != CORE_PACKET_HEADER &&
        core_packet_controller.rx_packet.header2 != CORE_PACKET_HEADER)
        return ERR;

    return OK;
}

/** 
 * @brief
 */
void handle_core_packet(CorePacket *packet) {
    switch (packet->peripheral) {
        case BUS_SERVO: handle_bus_servo(packet); break;
        case BUZZER: handle_buzzer(packet); break;
        case LED: handle_led(packet); break;
        default: break;
    }
}

/** 
 * @brief None
 */
void handle_bus_servo(CorePacket *packet) {
    // Create a task
    BusServoTask task;

    switch (packet->data[0]) {
    case SET_BUS_SERVO_ROTAION_ANGLE_AND_DURATION: {
        // Parse request
        BusServoAngleSettingRequest *request = (BusServoAngleSettingRequest*)packet->data;
        task.cmd = request->cmd;
        task.servo_count = request->servo_count;
        task.servo_count = request->cmd;
        task.read_only = false;

        for (uint8_t i = 0; i < request->servo_count; i++) {
            // Check duration
            uint16_t duration = (uint16_t)(request->servos[i].duration * 1000);
            duration = duration > 30000 ? 30000 : duration;

            task.servos[i].servo_id = request->servos[i].servo_id;
            task.servos[i].angle = request->servos[i].angle;
            task.servos[i].duration = duration;
        }
    } break;
    case READ_BUS_SERVO_ANGLE: {
        // Parse request
        BusServoQueryRequest *request = (BusServoQueryRequest*)packet->data;
        task.cmd = request->cmd;
        task.servo_count = request->cmd;
        task.servo_count = request->servo_count;
        task.read_only = true;

        for (uint8_t i = 0; i < request->servo_count; i++) 
            task.servos_id[i] = request->servos_id[i];
    } break;
    default:
        return;
    }
   
    add_bus_servo_task(task);
}

/** 
 * @brief
 * @retval None
 */
void handle_buzzer(CorePacket *packet) {
    // Parse request
    BuzzerRequest *request = (BuzzerRequest*)packet->data;

    // Check on duration
    uint32_t on_duration = (uint32_t)(request->on_duration * 1000);
    // Check off duration
    uint32_t off_duration = (uint32_t)(request->off_duration * 1000);

    // Create a task
    BuzzerTask task;
    task.frequency = request->frequency;
    task.on_duration = on_duration;
    task.off_duration = off_duration;
    task.repeat_count = request->repeat_count;
    task.state = READY_TO_TURN_ON_BUZZER;
    
    add_buzzer_task(task);
}


/** 
 * @brief
 * @retval None
 */
void handle_led(CorePacket *packet) {
    // Parse request
    LEDRequest *request = (LEDRequest*)packet->data;

    // Create a task
    LEDTask task;
    task.led_count = request->led_count;

    for (uint8_t i = 0; i < request->led_count; i++) {
        // Check on duration
        uint32_t on_duration = (uint32_t)(request->leds[i].on_duration * 1000);
        // Check off duration
        uint32_t off_duration = (uint32_t)(request->leds[i].off_duration * 1000);

        task.leds[i].led_id = request->leds[i].led_id;
        task.leds[i].on_duration = on_duration; 
        task.leds[i].off_duration = off_duration; 
        task.leds[i].repeat_count = request->leds[i].repeat_count;

        task.leds[i].state = READY_TO_TURN_ON_LED;
        task.leds[i].tick_count = 0;
    }

    add_led_task(task);
}


/** 
 * @brief Handle received data from core
 * @param
 *      @arg huart
 *      @arg Size
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART_CORE) {
        if (handle_core_rx_buffer(Size) == OK)  
            core_packet_controller.rx_state = OK;
        else
            core_packet_controller.rx_state = ERR;
        
        core_packet_controller.rx_finished = true;        
    } 
}

/////////////////////////////////////////////////////