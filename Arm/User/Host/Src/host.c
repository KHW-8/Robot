#include "host.h"

//////////* Headers *//////////
/* STD */
#include <string.h>
#include <stdio.h>
/* Arm */
//// Core
#include "usart.h"
//// User
// Host
#include "request_type.h"
#include "response_type.h"
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
#define USART_HOST USART2
///////////////////////////////////////

///////////////* Global Variable *///////////////
static UART_HandleTypeDef *huart_host = &huart2;

static HostPacketController packet_controller;

static uint8_t rx_buf[HOST_PACKET_DATA_MAX_LENGTH];
/////////////////////////////////////////////////


//////////* Functions *//////////
Res transmit_packet_to_host(HostPacket *packet) {
    uint8_t  packet_length = PACKET_HEADER_COUNT + 2 + packet->data_length;

    uint8_t *pPacket = (uint8_t*)packet;

    for (uint16_t i = 0; i < packet_length; i++) {
        // Wait until TDR is empty (TXE flag is set)
        uint32_t initial_tick = HAL_GetTick();
        while (!__HAL_UART_GET_FLAG(huart_host, UART_FLAG_TXE)) {
            if (HAL_GetTick() - initial_tick > 10) 
                return ERR;
        }
        
        huart_host->Instance->TDR = (pPacket[i] & 0xFF);
    }


    // Wait until TDR is empty (TC flag is set)
    uint32_t initial_tick = HAL_GetTick();
    while (!__HAL_UART_GET_FLAG(huart_host, UART_FLAG_TC)) {
        if (HAL_GetTick() - initial_tick > 10) 
            return ERR;
    }

    return OK;
}

void initialize_host_packet(HostPacket *packet, uint8_t peripheral) {
    packet->header1 = HOST_PACKET_HEADER;
    packet->header2 = HOST_PACKET_HEADER;
    packet->peripheral = peripheral;
    packet->data_length = 0;
    memset(packet->data, 0, sizeof(packet->data));
}

void initialize_host() {
    packet_controller.rx_finished = false;
    packet_controller.rx_state = ERR;

    HAL_UARTEx_ReceiveToIdle_DMA(huart_host, rx_buf, HOST_PACKET_DATA_MAX_LENGTH);
}

Res transmit_byte_to_host(uint8_t byte) {
    HAL_UART_Transmit_DMA(huart_host, (uint8_t*)&byte, 1);

    return OK;
}

Res transmit_msg_to_host(const char *buf) {
    char msg[128];

    int len = snprintf(msg, sizeof(msg), "%s\r\n", buf);

    HAL_UART_Transmit(huart_host, (uint8_t*)msg, len, HAL_MAX_DELAY);

    return OK;
}

void receive_packet_from_host() {
    if (!packet_controller.rx_finished)
        return;

    if (packet_controller.rx_state == OK)
        handle_host_packet(&packet_controller.rx_packet);

    packet_controller.rx_finished = false;
    HAL_UARTEx_ReceiveToIdle_DMA(huart_host, rx_buf, HOST_PACKET_DATA_MAX_LENGTH);
}

/** 
 * @brief
 */
Res handle_host_rx_buffer(uint8_t packet_len) {
    // Write data from rx_buf to rx_packet
    uint8_t *pPacket = (uint8_t*)&packet_controller.rx_packet;

    for (uint32_t i = 0; i < packet_len; i++) 
        pPacket[i] = rx_buf[i];

    if (packet_controller.rx_packet.header1 != HOST_PACKET_HEADER &&
        packet_controller.rx_packet.header2 != HOST_PACKET_HEADER)
        return ERR;

    return OK;
}

/** 
 * @brief
 */
void handle_host_packet(HostPacket *packet) {
    switch (packet->peripheral) {
        case BUS_SERVO: handle_bus_servo(packet); break;
        case BUZZER: handle_buzzer(packet); break;
        case LED: handle_led(packet); break;
        default: break;
    }
}

/** 
 * @brief
 */
void handle_bus_servo(HostPacket *packet) {
    // Init a packet to tranmit to the host
    initialize_host_packet(&packet_controller.tx_packet, BUS_SERVO);

    switch ((int32_t)packet->data[0]) {
    case SET_BUS_SERVO_ROTAION_ANGLE_AND_DURATION: {
        BusServoAngleSettingRequest *request = (BusServoAngleSettingRequest*)&packet->data;

        for (uint8_t i = 0; i < request->servo_count; i++) {
            set_bus_servo_angle_and_duration(
                request->servos[i].servo_id,
                request->servos[i].angle,
                request->servos[i].duration
            );
        }
    } break;
    case READ_BUS_SERVO_ANGLE: {
        // Parse request
        BusServoAngleQueryRequest *request = (BusServoAngleQueryRequest*)&packet->data;
        
        // Execute task
        int16_t angle = 0; 

        // Create a report to transmit to the host
        BusServoAngleResponse *report = (BusServoAngleResponse*)&packet_controller.tx_packet.data;
        report->cmd = packet->data[0];
        report->result = OK;

        for (uint8_t i = 0; i < request->servo_count; i++) {
            if (read_bus_servo_angle(request->servos_id[i], &angle) == OK) {
                report->servos[report->servo_count].servo_id = request->servos_id[i];
                report->servos[report->servo_count++].angle = angle;
            }
        }

        // Set host packet data length
        packet_controller.tx_packet.data_length = 3 + sizeof(report->servos[0]) * report->servo_count;
    } break;
    default: break;
    }

    // Transmit packet to host
    transmit_packet_to_host(&packet_controller.tx_packet);
}

/** 
 * @brief
 * @retval
 */
void handle_buzzer(HostPacket *packet) {
    BuzzerRequest *request = (BuzzerRequest*)&packet->data;

    BuzzerTask task;
    task.frequency = request->frequency;
    task.on_duration = request->on_duration;
    task.off_duration = request->off_duration;
    task.repeat_count = request->repeat_count;
    task.state = READY_TO_TURN_ON_BUZZER;
    
    add_buzzer_task(task);
}


/** 
 * @brief
 * @retval None
 */
void handle_led(HostPacket *packet) {
    LEDRequest *request = (LEDRequest*)&packet->data;

    LEDTask task;
    task.led_count = request->led_count;

    for (uint8_t i = 0; i < request->led_count; i++) {
        task.leds[i].led_id = request->leds[i].led_id;
        task.leds[i].on_duration = request->leds[i].on_duration; 
        task.leds[i].off_duration = request->leds[i].off_duration; 
        task.leds[i].repeat_count = request->leds[i].repeat_count;

        task.leds[i].state = READY_TO_TURN_ON_LED;
        task.leds[i].tick_count = 0;
    }

    add_led_task(task);
}


/** 
 * @brief Handle received data from host
 * @param
 *      @arg huart
 *      @arg Size
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART_HOST) {
        if (handle_host_rx_buffer(Size) == OK)  
            packet_controller.rx_state = OK;
        
        packet_controller.rx_finished = true;        
    } 
}

/////////////////////////////////