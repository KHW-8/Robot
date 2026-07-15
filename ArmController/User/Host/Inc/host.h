/** 
 * @brief Host
 */
#ifndef HOST_H_
#define HOST_H_

//////////* Headers *//////////
/* STD */
#include <stdint.h>
#include <stdbool.h>
/* ArmController */
#include "global.h"
///////////////////////////////

// Host packet
#define HOST_PACKET_HEADER 0x55
#define HOST_PACKET_HEADER_COUNT 2
#define HOST_PACKET_DATA_MAX_LENGTH 1024

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
    uint8_t data[HOST_PACKET_DATA_MAX_LENGTH];
} HostPacket;

#pragma pack(0)

typedef struct {
    // Transmit
    HostPacket tx_packet;

    // Receive
    HostPacket rx_packet;
    bool rx_finished;
    Res rx_state;
} HostPacketController;


// Initiation
void initialize_host();
void initialize_host_packet(HostPacket *packet, uint8_t peripheral);

// Transmit/Receive
Res transmit_packet_to_host(HostPacket *packet);
Res transmit_msg_to_host(const char* buf);

void receive_packet_from_host();

// Handle
Res handle_host_rx_buffer(uint8_t packet_len);
void handle_host_packet(HostPacket *packet);
void handle_bus_servo(HostPacket *packet);

#endif