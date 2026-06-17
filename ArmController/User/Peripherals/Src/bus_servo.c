#include "bus_servo.h"

// STD
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
// ArmController
#include "main.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "global.h"
#include "host.h"

BusServoPacketController bus_servo_packet_controller;
uint8_t rx_buf = 0;

/**
 * @brief Create bus servo packet
 */
BusServoPacket createBusServoPacket(uint8_t servo_id, 
                                    uint8_t cmd,
                                    uint8_t param1,
                                    uint8_t param2,
                                    uint8_t param3,
                                    uint8_t param4,
                                    uint8_t paramNum) {
    BusServoPacket packet;
    packet.header1 = BUS_SERVO_PACKET_HEADER;
    packet.header2 = BUS_SERVO_PACKET_HEADER;
    packet.servo_id = servo_id;
    packet.data_length = 3 + paramNum;
    packet.cmd = cmd;
    packet.params[0] = param1;
    packet.params[1] = param2;
    packet.params[2] = param3;
    packet.params[3] = param4;
    setChksum(&packet);
    packet.packet_length = BUS_SERVO_PACKET_HEADER_COUNT + packet.data_length + 1;

    return packet;
}

void setChksum(BusServoPacket *packet) {
    uint8_t *p = (uint8_t*)packet;
    uint32_t sum = 0;

    for (uint32_t i = 0; i < packet->data_length; i++)  
        sum += p[i + BUS_SERVO_PACKET_HEADER_COUNT];

    packet->chksum = (uint8_t)(~sum);
}

/** 
 * @brief Handle received bus servo packet
 * @param
 *      @arg rx_buf
 */
bool bus_servo_packet_handler(uint8_t rx_buf) {
    switch (bus_servo_packet_controller.rx_state) {
        case RECEIVING_HEADER1: {
            bus_servo_packet_controller.rx_state = rx_buf == BUS_SERVO_PACKET_HEADER ? RECEIVING_HEADER2 : RECEIVING_HEADER1;
            bus_servo_packet_controller.rx_packet.header1 = BUS_SERVO_PACKET_HEADER;
            return false;
        }
        case RECEIVING_HEADER2: {
            bus_servo_packet_controller.rx_state = rx_buf == BUS_SERVO_PACKET_HEADER ? RECEIVING_SERVO_ID : RECEIVING_HEADER2;
            bus_servo_packet_controller.rx_packet.header2 = BUS_SERVO_PACKET_HEADER;
            return false;
        }
        case RECEIVING_SERVO_ID: {
            bus_servo_packet_controller.rx_state = rx_buf == bus_servo_packet_controller.tx_packet.servo_id ? RECEIVING_DATA_LENGTH : RECEIVING_SERVO_ID;
            bus_servo_packet_controller.rx_packet.servo_id = rx_buf;
            return false;
        }
        case RECEIVING_DATA_LENGTH: {
            if (rx_buf > 7) {
                bus_servo_packet_controller.rx_state = RECEIVING_HEADER1;
            } else {
                bus_servo_packet_controller.rx_state = RECEIVING_COMMAND;
                bus_servo_packet_controller.rx_packet.data_length = rx_buf;
            }
            return false;
        }
        case RECEIVING_COMMAND: {
            bus_servo_packet_controller.rx_state = bus_servo_packet_controller.tx_packet.data_length > 3 ?  RECEIVING_CHECKSUM : RECEIVING_PARAMETERS;
            bus_servo_packet_controller.rx_packet.cmd = rx_buf;
            return false;
        } 
        case RECEIVING_PARAMETERS: {
            bus_servo_packet_controller.rx_packet.params[bus_servo_packet_controller.param_index++] =  rx_buf;
            if (bus_servo_packet_controller.param_index + 3 == bus_servo_packet_controller.rx_packet.data_length) 
                bus_servo_packet_controller.rx_state = RECEIVING_CHECKSUM;
            return false;
        } 
        case RECEIVING_CHECKSUM: {
            // Set receiving state
            bus_servo_packet_controller.rx_state = RECEIVING_HEADER1;

            // Set packet length
            bus_servo_packet_controller.rx_packet.packet_length = bus_servo_packet_controller.rx_packet.data_length + 3;

            // Generate the received packet checksum
            setChksum(&bus_servo_packet_controller.rx_packet);

            /*
             * Verify checksum.
             * If checksum is identical, it indicates that the packet has been received completely.
             * If checksum is not identical, it indicates that the packet has error.
             */
            return bus_servo_packet_controller.rx_packet.chksum != rx_buf;
        } 
        default: 
            return false;
    }
}

/** 
 * @brief Transmit packet to bus servo
 * @param
 *      @arg packet
 */
int transmit_packet_to_bus_servo(BusServoPacket *packet){
    // reset bus servo pin
    HAL_GPIO_WritePin(BUS_SERVO_EN_GPIO_Port, BUS_SERVO_EN_Pin, GPIO_PIN_RESET);

    // Transmit packet
    uint8_t *pPacket = (uint8_t*)packet;

    for (uint16_t i = 0; i < packet->packet_length; i++) {
        // Wait until TDR is empty (TXE flag is set)
        uint32_t start_time = HAL_GetTick();
        while (!__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE)) {
            if (HAL_GetTick() - start_time > 10) {
                HAL_GPIO_WritePin(BUS_SERVO_EN_GPIO_Port, BUS_SERVO_EN_Pin, GPIO_PIN_SET);
                return ERR;
            }   
        }
        
        if (i == packet->packet_length - 1) 
            huart2.Instance->DR = (packet->chksum & 0xFF);
        else 
            huart2.Instance->DR = (pPacket[i] & 0xFF);
    }


    // Wait until TDR is empty (TC flag is set)
    uint32_t start_time = HAL_GetTick();
    while (!__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC)) {
        if (HAL_GetTick() - start_time > 10) {
            HAL_GPIO_WritePin(BUS_SERVO_EN_GPIO_Port, BUS_SERVO_EN_Pin, GPIO_PIN_SET);
            return ERR;
        }   
    }

    // Set bus servo pin
    HAL_GPIO_WritePin(BUS_SERVO_EN_GPIO_Port, BUS_SERVO_EN_Pin, GPIO_PIN_SET);

    return OK;
}

int receive_packet_from_bus_servo(void) {
    bus_servo_packet_controller.rx_state = RECEIVING_HEADER1;
    bus_servo_packet_controller.rx_finished = false;                     
    bus_servo_packet_controller.param_index = 0;

    HAL_UART_Receive_IT(&huart2, &rx_buf, 1);


    uint32_t start_time = HAL_GetTick();
    while (!bus_servo_packet_controller.rx_finished) {
        if (HAL_GetTick() - start_time > bus_servo_packet_controller.time_out) {
            break;
        }
    }

    HAL_UART_AbortReceive_IT(&huart2);

    return OK;
}

void init_bus_servo(void) {
    // Init bus servo packet controller
    bus_servo_packet_controller.time_out = 100;

    HAL_GPIO_WritePin(BUS_SERVO_EN_GPIO_Port, BUS_SERVO_EN_Pin, GPIO_PIN_SET);
}


void read_bus_servo_id(void) {
    // Create packet
    bus_servo_packet_controller.tx_packet = createBusServoPacket(
        1, 
        READ_BUS_SERVO_ID, 
        0, 
        0, 
        0, 
        0,
        0
    );

    // Send packet
    uint32_t ret = transmit_packet_to_bus_servo(&bus_servo_packet_controller.tx_packet);
    if (ret != OK)
        return;

    // Receive packet
    receive_packet_from_bus_servo();
}

void read_bus_servo_pos(uint32_t id) {
    // Create packet
    bus_servo_packet_controller.tx_packet = createBusServoPacket(
        id, 
        READ_BUS_SERVO_POS, 
        0, 
        0, 
        0, 
        0,
        0
    );

    // Send packet
    uint32_t ret = transmit_packet_to_bus_servo(&bus_servo_packet_controller.tx_packet);
    if (ret != OK)
        return;

    // Receive packet
    ret = receive_packet_from_bus_servo();
    if (ret != OK) 
        return;
}

void set_bus_servo_pos(uint32_t id, int16_t position, uint32_t duration) {
    // Create packet
    uint8_t param1 = (uint8_t)position;
    uint8_t param2 = (uint8_t)(position >> 8);
    uint8_t param3 = (uint8_t)duration;
    uint8_t param4 = (uint8_t)(duration >> 8);
    bus_servo_packet_controller.tx_packet = createBusServoPacket(
        id, 
        SET_BUS_SERVO_ROTAION_ANGLE_AND_DURATION, 
        param1, 
        param2, 
        param3, 
        param4,
        4
    );

    // Send packet
    uint32_t ret = transmit_packet_to_bus_servo(&bus_servo_packet_controller.tx_packet);
    if (ret != OK)
        return;

    // Receive packet
    ret = receive_packet_from_bus_servo();
    if (ret != OK) 
        return;
}

void print_packet_info(BusServoPacket *packet) {
    char buf[20];

    send_msg_to_host("==============================");

    sprintf(buf, "Header1: %X", packet->header1);
    send_msg_to_host(buf);
    sprintf(buf, "Header2: %X", packet->header2);
    send_msg_to_host(buf);
    sprintf(buf, "Servo ID: %X", packet->servo_id);
    send_msg_to_host(buf);
    sprintf(buf, "Data length: %X", packet->data_length);
    send_msg_to_host(buf);
    sprintf(buf, "Command: %X", packet->cmd);
    send_msg_to_host(buf);
    if (packet->data_length > 3 && packet->data_length < 7) {
        for (uint32_t i = 0; i < packet->data_length - 3; i++) {
                sprintf(buf, "Param %lu: %X", (i + 1), packet->params[i]);
                send_msg_to_host(buf);
            }
    }
    sprintf(buf, "Checksum: %X", packet->chksum);
    send_msg_to_host(buf);

    send_msg_to_host("******************************");
    sprintf(buf, "Packet length: %d", packet->packet_length);
    send_msg_to_host(buf);

    send_msg_to_host("==============================");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (bus_servo_packet_handler(rx_buf)) {
            bus_servo_packet_controller.rx_finished = true;
        } else {
            if (bus_servo_packet_controller.rx_finished == false) {
                HAL_UART_Receive_IT(&huart2, &rx_buf, 1);
            }
        }
    }
}