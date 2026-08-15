// STD
#include <stdio.h>
// Arm
#include "host.h"
#include "bus_servo.h"

void print_host_packet_info(HostPacket *packet) {
    char buf[20];

    transmit_msg_to_host("==============================");

    sprintf(buf, "Header1: %X", packet->header1);
    transmit_msg_to_host(buf);
    sprintf(buf, "Header2: %X", packet->header2);
    transmit_msg_to_host(buf);
    sprintf(buf, "Peripheral: %X", packet->peripheral);
    transmit_msg_to_host(buf);
    sprintf(buf, "Data Length: %X", packet->data_length);
    transmit_msg_to_host(buf);
    for (uint32_t i = 0; i < packet->data_length; i++) {
        sprintf(buf, "Data %u: %X", (i + 1), packet->data[i]);
        transmit_msg_to_host(buf);
    }

    transmit_msg_to_host("==============================");
}

void print_bus_servo_packet_info(BusServoPacket *packet) {
    char buf[20];

    transmit_msg_to_host("==============================");

    sprintf(buf, "Header1: %X", packet->header1);
    transmit_msg_to_host(buf);
    sprintf(buf, "Header2: %X", packet->header2);
    transmit_msg_to_host(buf);
    sprintf(buf, "Servo ID: %X", packet->servo_id);
    transmit_msg_to_host(buf);
    sprintf(buf, "Data length: %X", packet->data_length);
    transmit_msg_to_host(buf);
    sprintf(buf, "Command: %X", packet->cmd);
    transmit_msg_to_host(buf);
    if (packet->data_length > 3 && packet->data_length < 7) {
        for (uint32_t i = 0; i < packet->data_length - 3; i++) {
            sprintf(buf, "Param %u: %X", (i + 1), packet->params[i]);
            transmit_msg_to_host(buf);
        }
    }
    sprintf(buf, "Checksum: %X", packet->chksum);
    transmit_msg_to_host(buf);
    
    transmit_msg_to_host("==============================");
}