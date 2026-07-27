#ifndef BUS_SERVO_H
#define BUS_SERVO_H

enum class BusServoCMD{
    SET_ROTAION_ANGLE_AND_DURATION = 1,
    READ_ROTAION_ANGLE_AND_DURATION = 2,
    SET_ROTAION_ANGLE_AND_DURATION_WITH_WAITING_STATE  = 7,
    READ_ROTAION_ANGLE_AND_DURATION_WITH_WAITING_STATE = 8,
    START_ROTATION = 11,
    STOP_ROTATION = 12,
    SET_ID = 13,
    READ_ID = 14,
    SET_ANGLE_DEVIATION = 17,
    SET_ANGLE_DEVIATION_DURING_POWER_OFF = 18,
    READ_ANGLE_DEVIATION = 19,
    SET_ANGLE_LIMIT = 20,
    READ_ANGLE_LIMIT = 21,
    SET_MAXIMUM_VIN = 22,
    READ_MAXIMUM_VIN = 23,
    SET_MAXIMUM_TEMPRATURE = 24,
    READ_MAXIMUM_TEMPRATURE = 25,
    READ_TEMPRATURE = 26,
    READ_VIN = 27,
    READ_ANGLE = 28,
    /*
     * Mode
     * 0: Servo
     * 1: Motor
     */
    SET_MODE = 29,
    READ_MODE = 30,
    /*
     * Power State
     * 0: OFF
     * 1: ON
     */
    SET_POWER_STATE = 31,
    READ_POWER_STATE = 32,
    /*
     * LED State
     * 0: OFF
     * 1: ON
     */
    SET_LED_STATE = 33,
    READ_LED_STATE = 34,

    /*
     * Error
     * 0: 
     * 1: 
     */
    SET_ERROR = 35,
    READ_ERROR = 36,
    READ_DISTANCE = 48
};

#endif