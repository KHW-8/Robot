/** 
 * @brief Control bus servo
 */
#ifndef BUS_SERVO_H_
#define BUS_SERVO_H_

//////////* Headers *//////////
/* STD */
#include <stdint.h>
#include <stdbool.h>
/* Arm */
#include "global.h"
///////////////////////////////

// Packet
#define BUS_SERVO_RX_BUFFER_SIZE 255

typedef enum {
    SET_BUS_SERVO_ROTAION_ANGLE_AND_DURATION = 1,
    READ_BUS_SERVO_ROTAION_ANGLE_AND_DURATION = 2,
    SET_BUS_SERVO_ROTAION_ANGLE_AND_DURATION_WITH_WAITING_STATE  = 7,
    READ_BUS_SERVO_ROTAION_ANGLE_AND_DURATION_WITH_WAITING_STATE = 8,
    START_BUS_SERVO_ROTATION = 11,
    STOP_BUS_SERVO_ROTATION = 12,
    SET_BUS_SERVO_ID = 13,
    READ_BUS_SERVO_ID = 14,
    SET_BUS_SERVO_ANGLE_DEVIATION = 17,
    SET_BUS_SERVO_ANGLE_DEVIATION_DURING_POWER_OFF = 18,
    READ_BUS_SERVO_ANGLE_DEVIATION = 19,
    SET_BUS_SERVO_ANGLE_LIMIT = 20,
    READ_BUS_SERVO_ANGLE_LIMIT = 21,
    SET_BUS_SERVO_MAXIMUM_VIN = 22,
    READ_BUS_SERVO_MAXIMUM_VIN = 23,
    SET_BUS_SERVO_MAXIMUM_TEMPRATURE = 24,
    READ_BUS_SERVO_MAXIMUM_TEMPRATURE = 25,
    READ_BUS_SERVO_TEMPRATURE = 26,
    READ_BUS_SERVO_VIN = 27,
    READ_BUS_SERVO_ANGLE = 28,
    /*
     * Mode
     * 0: Servo
     * 1: Motor
     */
    SET_BUS_SERVO_MODE = 29,
    READ_BUS_SERVO_MODE = 30,
    /*
     * Power State
     * 0: OFF
     * 1: ON
     */
    SET_BUS_SERVO_POWER_STATE = 31,
    READ_BUS_SERVO_POWER_STATE = 32,
    /*
     * LED State
     * 0: OFF
     * 1: ON
     */
    SET_BUS_SERVO_LED_STATE = 33,
    READ_BUS_SERVO_LED_STATE = 34,

    /*
     * Error
     * 0: 
     * 1: 
     */
    SET_BUS_SERVO_ERROR = 35,
    READ_BUS_SERVO_ERROR = 36,
    
    READ_BUS_SERVO_DISTANCE = 48
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

#pragma pack(1)

typedef struct {
    uint8_t header1;
    uint8_t header2;
    uint8_t servo_id;
    uint8_t data_length;
    uint8_t cmd;
    uint8_t params[4];
    uint8_t chksum;
} BusServoPacket;

#pragma pack()

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

void initialize_bus_servo_packet(BusServoPacket *packet, uint8_t servo_id, uint8_t data_length, uint8_t cmd);

// Transmit/Receive packet
Res transmit_packet_to_bus_servo(BusServoPacket *packet, bool tx_only);
Res receive_packet_from_bus_servo(); // It is not suggested to call this function manually

// Initiation
void initialize_bus_servo();

// Handle
bool handle_bus_servo_rx_buffer(uint8_t rx_buf);

// Basic  Operation
Res read_bus_servo_id();
Res read_bus_servo_angle(uint32_t id, int16_t *angle);

Res set_bus_servo_angle_and_duration(uint32_t id, int16_t angle, uint32_t duration);

#endif