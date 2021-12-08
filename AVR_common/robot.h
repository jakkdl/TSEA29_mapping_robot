// This file contains definitions and stuff that all units will need
#ifndef AVR_COMMON_ROBOT_H_
#define AVR_COMMON_ROBOT_H_
#include <stdint.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

enum Address
{
    LIDAR_FORWARD, //0
    LIDAR_BACKWARD,
    IR_LEFTFRONT,
    IR_LEFTBACK, // 4
    IR_RIGHTFRONT,
    IR_RIGHTBACK,
    GYRO,
    ODOMETER, // 8
    POSITION,
    ADR_HEADING,
    MAP_UPDATE,
    COMMAND, // 12
    ADR_DEBUG,
    PD_KP, //13
    PD_KD, //14
    PARITY_ERROR //15
};

struct data_packet
{
    enum Address address;
    uint8_t      byte_count;
    uint8_t      bytes_read;
    uint8_t      bytes[7];
};

extern const uint8_t ADR_DATA_PACKETS[];
/*{
    2,  //LIDAR_FORWARD
    2,  //LIDAR_BACKWARD
    2,  //IR_LEFTFRONT
    2,  //IR_LEFTBACK
    2,  //IR_RIGHTFRONT
    2,  //IR_RIGHTBACK
    2,  //ODOMETER **DESIGN SPEC INCORRECT**
    2,  //GYRO
    4,  //POSITION
    2,  //HEADER
    3,  //MAP_UPDATE
    1,  //COMMAND
    7,  //DEBUG
    1,  //PD_KP
    1,  //PD_KD
    1   //PARITY_ERROR
};*/

enum directionID
{
    STOP,
    START,
    FORWARD,
    BACKWARD,
    FW_LEFT,
    FW_RIGHT,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_AROUND
};

enum CellState
{
    WALL,
    EMPTY,
    UNKNOWN
};

#endif // AVR_COMMON_ROBOT_H_
