#include "host.h"
// STD
#include <stdio.h>
#include <string.h>
// ArmController
#include "global.h"
#include "usart.h"

HostPacketController host_packet_controller;
uint32_t last_cndtr = 0;
uint32_t last_tick = 0;

bool host_packet_handler(uint8_t packet_len) {
    host_packet_controller.rx_packet.packet_length = packet_len;
    uint8_t *buf = (uint8_t*)&host_packet_controller.rx_buf;
    uint8_t *pPacket = (uint8_t*)&host_packet_controller.rx_packet;

    for (uint32_t i = 0; i < host_packet_controller.rx_packet.packet_length; i++) 
        pPacket[i] = buf[i];
    
    if (host_packet_controller.rx_packet.header1 != HOST_PACKET_HEADER &&
        host_packet_controller.rx_packet.header2 != HOST_PACKET_HEADER)
        return false;

    return true;
}

void transmit_packet_to_host(HostPacket *packet) {
 
}

void init_host(void) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, host_packet_controller.rx_buf, HOST_RX_BUFFER_SIZE);
}

int send_msg_to_host(const char *buf) {
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\n\r", 2, HAL_MAX_DELAY);

    return OK;
}

void print_host_packet_info(HostPacket *packet) {
    char buf[20];

    send_msg_to_host("==============================");

    sprintf(buf, "Header1: %X", packet->header1);
    send_msg_to_host(buf);
    sprintf(buf, "Header2: %X", packet->header2);
    send_msg_to_host(buf);
    sprintf(buf, "Peripheral: %X", packet->peripheral);
    send_msg_to_host(buf);
    sprintf(buf, "Data length: %X", packet->data_length);
    send_msg_to_host(buf);
    sprintf(buf, "Checksum: %X", packet->chksum);
    send_msg_to_host(buf);

    send_msg_to_host("******************************");
    sprintf(buf, "Packet length: %d", packet->packet_length);
    send_msg_to_host(buf);

    send_msg_to_host("==============================");
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        if (host_packet_handler(Size)) {

        } 
        
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, host_packet_controller.rx_buf, HOST_RX_BUFFER_SIZE);
    }
}