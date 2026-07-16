#include "host.h"

//////////* Headers *//////////
/* STD */
#include <string.h>
#include <stdio.h>
/* ArmController */
//// Core
#include "global.h"
#include "usart.h"
//// User
// Host
#include "request_type.h"
#include "response_type.h"
// Peripherals
#include "bus_servo.h"
#include "buzzer.h"
#include "led.h"
///////////////////////////////

HostPacketController host_packet_controller;

uint8_t host_rx_buf[HOST_PACKET_DATA_MAX_LENGTH];

extern void print_host_packet_info(HostPacket *packet);

Res transmit_packet_to_host(HostPacket *packet) {
    uint8_t  packet_length = PACKET_HEADER_COUNT + 2 + packet->data_length;

    uint8_t *pPacket = (uint8_t*)packet;
    
    for (uint16_t i = 0; i < packet_length; i++) {
        // Wait until TDR is empty (TXE flag is set)
        uint32_t start_time = HAL_GetTick();
        while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)) {
            if (HAL_GetTick() - start_time > 10) 
                return ERR;
        }
        
        huart1.Instance->DR = (pPacket[i] & 0xFF);
    }


    // Wait until TDR is empty (TC flag is set)
    uint32_t start_time = HAL_GetTick();
    while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)) {
        if (HAL_GetTick() - start_time > 10) 
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
    host_packet_controller.rx_finished = false;
    host_packet_controller.rx_state = ERR;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, host_rx_buf, HOST_PACKET_DATA_MAX_LENGTH);
}

Res transmit_msg_to_host(const char *buf) {
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\n\r", 2, HAL_MAX_DELAY);

    return OK;
}

void receive_packet_from_host() {
    if (!host_packet_controller.rx_finished)
        return;

    if (host_packet_controller.rx_state == OK)
        handle_host_packet(&host_packet_controller.rx_packet);

    host_packet_controller.rx_finished = false;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, host_rx_buf, HOST_PACKET_DATA_MAX_LENGTH);
}

/** 
 * @brief
 */
Res handle_host_rx_buffer(uint8_t packet_len) {
    // Write data from rx_buf to rx_packet
    uint8_t *pPacket = (uint8_t*)&host_packet_controller.rx_packet;

    for (uint32_t i = 0; i < packet_len; i++) 
        pPacket[i] = host_rx_buf[i];

    if (host_packet_controller.rx_packet.header1 != HOST_PACKET_HEADER &&
        host_packet_controller.rx_packet.header2 != HOST_PACKET_HEADER)
        return ERR;

    return OK;
}

/** 
 * @brief
 */
void handle_host_packet(HostPacket *packet) {
    switch (packet->peripheral) {
        case BUS_SERVO: handle_bus_servo(packet); break;
        case BUZZER: break;
        case LED: break;
        default: break;
    }
}

/** 
 * @brief
 */
void handle_bus_servo(HostPacket *packet) {
    // Init a packet to tranmit to the host
    initialize_host_packet(&host_packet_controller.tx_packet, BUS_SERVO);

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
        BusServoAngleResponse *report = (BusServoAngleResponse*)&host_packet_controller.tx_packet.data;
        report->cmd = packet->data[0];
        report->result = OK;

        for (uint8_t i = 0; i < request->servo_count; i++) {
            if (read_bus_servo_angle(request->servos_id[i], &angle) == OK) {
                report->servos[report->servo_count].servo_id = request->servos_id[i];
                report->servos[report->servo_count++].angle = angle;
            }
        }

        // Set host packet data length
        host_packet_controller.tx_packet.data_length = 3 + sizeof(report->servos[0]) * report->servo_count;
    } break;
    default: break;
    }

    // Transmit packet to host
    transmit_packet_to_host(&host_packet_controller.tx_packet);
}


/** 
 * @brief Handle received data from host
 * @param
 *      @arg huart
 *      @arg Size
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        if (handle_host_rx_buffer(Size) == OK)  
            host_packet_controller.rx_state = OK;
        
        host_packet_controller.rx_finished = true;        
    } 
}