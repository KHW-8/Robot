/** 
 * @brief Control servos
 */
#ifndef BUS_SERVO_H_
#define BUS_SERVO_H_

#include <stdint.h>
#include <stdbool.h>

// Packet
#define BUS_SERVO_PACKET_HEADER 0x55
#define BUS_SERVO_PACKET_HEADER_COUNT 2

typedef enum {
    SET_BUS_SERVO_ROTAION_ANGLE_AND_DURATION = 1,
    READ_BUS_SERVO_ROTAION_ANGLE_AND_DURATION = 2,
    SET_BUS_SERVO_ROTAION_ANGLE_AND_DURATION_WITH_WAITING_STATE  = 7,
    READ_BUS_SERVO_ROTAION_ANGLE_AND_DURATION_WITH_WAITING_STATE = 8,
    START_BUS_SERVO_ROTATION = 11,
    STOP_BUS_SERVO_ROTATION = 12,
    SET_BUS_SERVO_ID = 13,
    READ_BUS_SERVO_ID = 14,
    READ_BUS_SERVO_POS = 28
} BusServoCMD;

typedef enum {
    RECEIVING_HEADER1,
    RECEIVING_HEADER2,
    RECEIVING_SERVO_ID,
    RECEIVING_DATA_LENGTH,
    RECEIVING_COMMAND,
    RECEIVING_PARAMETERS,
    RECEIVING_CHECKSUM
} BusServoPacketRecvState;

typedef struct {
    uint8_t header1;
    uint8_t header2;
    uint8_t servo_id;
    uint8_t data_length;
    uint8_t cmd;
    uint8_t params[4];
    uint8_t chksum;
    uint8_t packet_length;
} BusServoPacket;

typedef struct {
    // Transmit
    BusServoPacket tx_packet;

    // Receive
    BusServoPacket rx_packet;
    BusServoPacketRecvState rx_state;
    bool rx_finished;
    uint8_t param_index;

    // Misc
    uint32_t time_out;

} BusServoPacketController;

BusServoPacket createBusServoPacket(uint8_t servo_id, uint8_t cmd, uint8_t param1, uint8_t param2, uint8_t param3, uint8_t param4, uint8_t paramNum);
void setChksum(BusServoPacket *packet);
bool bus_servo_packet_handler(uint8_t rx_buf);

// Transmit/Receive packet
int transmit_packet_to_bus_servo(BusServoPacket *packet);
int receive_packet_from_bus_servo(void);

// Initiation
void init_bus_servo(void);

// Basic  Operation
void read_bus_servo_id(void);
void read_bus_servo_pos(uint32_t id);

void set_bus_servo_pos(uint32_t id, int16_t position, uint32_t duration);

// Test

// Misc
void print_packet_info(BusServoPacket *packet);

#endif