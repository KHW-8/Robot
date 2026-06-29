#ifndef HOST_H_
#define HOST_H_

#include <stdint.h>
#include <stdbool.h>

// Host packet
#define HOST_PACKET_HEADER 0x55
#define HOST_PACKET_HEADER_COUNT 2
#define HOST_RX_BUFFER_SIZE 255

typedef enum {
    RECEIVING_HEADER1_,
    RECEIVING_HEADER2_,
    RECEIVING_DATA_LENGTH_,
    RECEIVING_DATA_,
    RECEIVING_CHECKSUM_
} HostPacketRecvState;

typedef enum {
    NONE,
    BUS_SERVO,
    BUZZER,
    LED,
} Peripheral;

typedef struct {
    uint8_t header1;
    uint8_t header2;
    uint8_t peripheral;
    uint8_t data_length;
    uint8_t chksum;
    uint8_t packet_length;
} HostPacket;

typedef struct {
    // Transmit
    HostPacket tx_packet;

    // Receive
    HostPacket rx_packet;
    uint8_t rx_buf[HOST_RX_BUFFER_SIZE];
    HostPacketRecvState rx_state;
    bool rx_finished;

    // Misc
    uint32_t time_out;
} HostPacketController;

// Initiation
void init_host(void);

// Transmit/Receive
void transmit_packet_to_host(HostPacket *packet);
void receive_packet_from_host(void);

int send_msg_to_host(const char* buf);

// Misc
void print_host_packet_info(HostPacket *packet);

#endif