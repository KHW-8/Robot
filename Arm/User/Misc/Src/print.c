// STD
#include <stdio.h>
// Arm
#include "core.h"
#include "bus_servo.h"

void print_core_packet_info(CorePacket *packet) {
    char buf[20];

    transmit_msg_to_core("==============================");

    sprintf(buf, "Header1: %X", packet->header1);
    transmit_msg_to_core(buf);
    sprintf(buf, "Header2: %X", packet->header2);
    transmit_msg_to_core(buf);
    sprintf(buf, "Peripheral: %X", packet->peripheral);
    transmit_msg_to_core(buf);
    sprintf(buf, "Data Length: %X", packet->data_length);
    transmit_msg_to_core(buf);
    for (uint32_t i = 0; i < packet->data_length; i++) {
        sprintf(buf, "Data %u: %X", (i + 1), packet->data[i]);
        transmit_msg_to_core(buf);
    }

    transmit_msg_to_core("==============================");
}

void print_bus_servo_packet_info(BusServoPacket *packet) {
    char buf[20];

    transmit_msg_to_core("==============================");

    sprintf(buf, "Header1: %X", packet->header1);
    transmit_msg_to_core(buf);
    sprintf(buf, "Header2: %X", packet->header2);
    transmit_msg_to_core(buf);
    sprintf(buf, "Servo ID: %X", packet->servo_id);
    transmit_msg_to_core(buf);
    sprintf(buf, "Data length: %X", packet->data_length);
    transmit_msg_to_core(buf);
    sprintf(buf, "Command: %X", packet->cmd);
    transmit_msg_to_core(buf);
    if (packet->data_length > 3 && packet->data_length < 7) {
        for (uint32_t i = 0; i < packet->data_length - 3; i++) {
            sprintf(buf, "Param %u: %X", (i + 1), packet->params[i]);
            transmit_msg_to_core(buf);
        }
    }
    sprintf(buf, "Checksum: %X", packet->chksum);
    transmit_msg_to_core(buf);
    
    transmit_msg_to_core("==============================");
}