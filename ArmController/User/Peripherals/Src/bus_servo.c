#include "bus_servo.h"

//////////* Headers *//////////
/* STD */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* ArmController */
// Core
#include "main.h"
#include "usart.h"
// User
#include "host.h"
// Misc
#include "check_sum.h"
///////////////////////////////

static BusServoPacketController packet_controller;

static uint8_t rx_buf;

/**
 * @brief Initialize bus servo packet
 */
void initialize_bus_servo_packet(BusServoPacket *packet,
                                    uint8_t servo_id, 
                                    uint8_t data_length,
                                    uint8_t cmd) {
    packet->header1 = PACKET_HEADER;
    packet->header2 = PACKET_HEADER;
    packet->servo_id = servo_id;
    packet->data_length = data_length;
    packet->cmd = cmd;
    memset(packet->params, 0, sizeof(packet->params));
    packet->chksum = 0;
}

/** 
 * @brief Transmit packet to bus servo
 * @param
 *      @arg packet
 * @retval
 */
Res transmit_packet_to_bus_servo(BusServoPacket *packet, bool tx_only){
    // Set to write mode (reset bus servo pin)
    HAL_GPIO_WritePin(BUS_SERVO_EN_GPIO_Port, BUS_SERVO_EN_Pin, GPIO_PIN_RESET);

    // Transmit packet
    uint8_t  packet_length = PACKET_HEADER_COUNT + packet->data_length + 1;

    uint8_t *pPacket = (uint8_t*)packet;
    
    for (uint16_t i = 0; i < packet_length; i++) {
        // Wait until TDR is empty (TXE flag is set)
        uint32_t initial_tick = HAL_GetTick();
        while (!__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE)) {
            if (HAL_GetTick() - initial_tick > 10) {
                HAL_GPIO_WritePin(BUS_SERVO_EN_GPIO_Port, BUS_SERVO_EN_Pin, GPIO_PIN_SET);
                return ERR;
            }   
        }
        
        if (i == packet_length - 1) 
            huart2.Instance->DR = (packet->chksum & 0xFF);
        else 
            huart2.Instance->DR = (pPacket[i] & 0xFF);
    }


    // Wait until TDR is empty (TC flag is set)
    uint32_t initial_tick = HAL_GetTick();
    while (!__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC)) {
        if (HAL_GetTick() - initial_tick > 10) {
            HAL_GPIO_WritePin(BUS_SERVO_EN_GPIO_Port, BUS_SERVO_EN_Pin, GPIO_PIN_SET);
            return ERR;
        }   
    }

    // Set to read mode (set bus servo pin)
    HAL_GPIO_WritePin(BUS_SERVO_EN_GPIO_Port, BUS_SERVO_EN_Pin, GPIO_PIN_SET);

    if (!tx_only)
        receive_packet_from_bus_servo();

    return OK;
}

Res receive_packet_from_bus_servo() {
    packet_controller.rx_state = RECEIVING_HEADER1;
    packet_controller.rx_finished = false;                     
    packet_controller.param_index = 0;

    HAL_UART_Receive_IT(&huart2, &rx_buf, 1);

    uint32_t initial_tick = HAL_GetTick();

    while (!packet_controller.rx_finished) {
        if (HAL_GetTick() - initial_tick > packet_controller.time_out)
            break;
    }

    HAL_UART_AbortReceive_IT(&huart2);

    return packet_controller.rx_finished == true ? OK : ERR;
}

void initialize_bus_servo() {
    // Init bus servo packet controller
    packet_controller.time_out = 100;

    HAL_GPIO_WritePin(BUS_SERVO_EN_GPIO_Port, BUS_SERVO_EN_Pin, GPIO_PIN_SET);
}


Res read_bus_servo_id() {
    // Create packet
    initialize_bus_servo_packet(&packet_controller.tx_packet, 1, 3, READ_BUS_SERVO_ID);

    packet_controller.tx_packet.chksum = generate_check_sum((uint8_t*)&packet_controller.tx_packet, packet_controller.tx_packet.data_length);

    // Send packet
    uint32_t ret = transmit_packet_to_bus_servo(&packet_controller.tx_packet, false);
    if (ret != OK)
        return ERR;

    return OK;
}

Res read_bus_servo_angle(uint32_t id, int16_t *angle) {
    // Create packet
    initialize_bus_servo_packet(&packet_controller.tx_packet, id, 3, READ_BUS_SERVO_ANGLE);

    packet_controller.tx_packet.chksum = generate_check_sum((uint8_t*)&packet_controller.tx_packet, packet_controller.tx_packet.data_length);

    // Send packet
    uint32_t ret = transmit_packet_to_bus_servo(&packet_controller.tx_packet, false);
    if (ret != OK)
        return ERR;

    *angle = (int16_t)(*(int16_t*)packet_controller.rx_packet.params) * 240 / 1000;

    return OK;
}

Res set_bus_servo_angle_and_duration(uint32_t id, int16_t angle, uint32_t duration) {
    // Check arguments
    angle = angle > 240 ? 240 : angle;
    angle = angle * 1000 / 240;
    duration = duration > 30000 ? 30000 : duration;

    // Create packet
    initialize_bus_servo_packet(&packet_controller.tx_packet, id, 4, SET_BUS_SERVO_ROTAION_ANGLE_AND_DURATION);

    packet_controller.tx_packet.params[0] = (uint8_t)angle;
    packet_controller.tx_packet.params[1] = (uint8_t)(angle >> 8);
    packet_controller.tx_packet.params[2] = (uint8_t)duration;
    packet_controller.tx_packet.params[3] = (uint8_t)(duration >> 8);

    packet_controller.tx_packet.chksum = generate_check_sum((uint8_t*)&packet_controller.tx_packet, packet_controller.tx_packet.data_length);

    // Send packet
    uint32_t ret = transmit_packet_to_bus_servo(&packet_controller.tx_packet, false);
    if (ret != OK)
        return ERR;

    return OK;
}

/** 
 * @brief Parse rx_buf and write data into packet_controller.rx_packet
 * @param
 *      @arg rx_buf
 * @return 
 */
bool handle_rx_buffer(uint8_t rx_buf) {
    switch (packet_controller.rx_state) {
        case RECEIVING_HEADER1: {
            packet_controller.rx_state = rx_buf == PACKET_HEADER ? RECEIVING_HEADER2 : RECEIVING_HEADER1;
            packet_controller.rx_packet.header1 = PACKET_HEADER;
            return false;
        }
        case RECEIVING_HEADER2: {
            packet_controller.rx_state = rx_buf == PACKET_HEADER ? RECEIVING_SERVO_ID : RECEIVING_HEADER2;
            packet_controller.rx_packet.header2 = PACKET_HEADER;
            return false;
        }
        case RECEIVING_SERVO_ID: {
            packet_controller.rx_state = rx_buf == packet_controller.tx_packet.servo_id ? RECEIVING_DATA_LENGTH : RECEIVING_SERVO_ID;
            packet_controller.rx_packet.servo_id = rx_buf;
            return false;
        }
        case RECEIVING_DATA_LENGTH: {
            if (rx_buf > 7) {
                packet_controller.rx_state = RECEIVING_HEADER1;
            } else {
                packet_controller.rx_state = RECEIVING_COMMAND;
                packet_controller.rx_packet.data_length = rx_buf;
            }
            return false;
        }
        case RECEIVING_COMMAND: {
            packet_controller.rx_state = packet_controller.tx_packet.data_length > 3 ?  RECEIVING_CHECKSUM : RECEIVING_PARAMETERS;
            packet_controller.rx_packet.cmd = rx_buf;
            return false;
        } 
        case RECEIVING_PARAMETERS: {
            packet_controller.rx_packet.params[packet_controller.param_index++] =  rx_buf;
            if (packet_controller.param_index + 3 == packet_controller.rx_packet.data_length) 
                packet_controller.rx_state = RECEIVING_CHECKSUM;
            return false;
        } 
        case RECEIVING_CHECKSUM: {
            // Set receiving state
            packet_controller.rx_state = RECEIVING_HEADER1;

            // Generate the received packet checksum
            packet_controller.rx_packet.chksum = generate_check_sum((uint8_t*)&packet_controller.rx_packet, packet_controller.rx_packet.data_length);

            /*
             * Verify checksum.
             * If checksum is identical, it indicates that the packet has been received completely.
             * If checksum is not identical, it indicates that the packet has error.
             */
            return packet_controller.rx_packet.chksum != rx_buf;
        } 
        default: 
            return false;
    }
}

/** 
 * @brief Handle received data from bus servos
 * @param
 *      @arg huart
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART2) {
        if (handle_rx_buffer(rx_buf)) {
            packet_controller.rx_finished = true;
        } else {
            if (packet_controller.rx_finished == false) 
                HAL_UART_Receive_IT(&huart2, &rx_buf, 1);
        }
    }
}