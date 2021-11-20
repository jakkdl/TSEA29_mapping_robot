// This file contains definitions and stuff that all units will need
#ifndef AVR_COMMON_ROBOT_H_
#define AVR_COMMON_ROBOT_H_
#include <stdint.h>

enum Address
{
    LIDAR_FORWARD,
    LIDAR_BACKWARD,
    IR_LEFTFRONT,
    IR_LEFTBACK,
    IR_RIGHTFRONT,
    IR_RIGHTBACK,
    ODOMETER,
    GYRO,
    POSITION,
    HEADER,
    MAP_UPDATE,
    COMMAND,
    DEBUG,
    PD_KP,
    PD_KD,
    PARITY_ERROR
};

struct data_packet
{
    enum Address address;
    uint8_t      byte_count;
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
