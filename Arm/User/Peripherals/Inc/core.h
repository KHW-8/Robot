/** 
 * @brief Core
 */
#ifndef CORE_H_
#define CORE_H_

//////////* Headers *//////////
/* STD */
#include <stdint.h>
#include <stdbool.h>
/* Arm */
#include "global.h"
///////////////////////////////

// Core packet
#define CORE_PACKET_HEADER 0x55
#define CORE_PACKET_HEADER_COUNT 2
#define CORE_PACKET_DATA_MAX_LENGTH 1024

typedef enum {
    NO_PERIPHERAL,
    BUS_SERVO,
    BUZZER,
    LED,
} Peripheral;

#pragma pack(1)

typedef struct {
    uint8_t header1;
    uint8_t header2;
    uint8_t peripheral;
    uint8_t data_length;
    uint8_t data[CORE_PACKET_DATA_MAX_LENGTH];
} CorePacket;

#pragma pack()

typedef struct {
    // Transmit
    CorePacket tx_packet;

    // Receive
    CorePacket rx_packet;
    bool rx_finished;
    Res rx_state;
} CorePacketController;


// Initiation
void initialize_core();
void initialize_core_packet(CorePacket *packet, uint8_t peripheral);

// Transmit/Receive
Res transmit_packet_to_core(CorePacket *packet);
Res transmit_byte_to_core(uint8_t byte);
Res transmit_msg_to_core(const char* buf);

void receive_packet_from_core();

// Handle
Res handle_core_rx_buffer(uint8_t packet_len);
void handle_core_packet(CorePacket *packet);
void handle_bus_servo(CorePacket *packet);
void handle_buzzer(CorePacket *packet);
void handle_led(CorePacket *packet);

#endif