// This file contains definitions and stuff that all units will need
#ifndef ROBOT_H
#define ROBOT_H
#include <stdint.h>

enum Address
{
    lidar_forward,
    lidar_backward,
    ir_leftfront,
    ir_leftback,
    ir_rightfront,
    ir_rightback,
    odometer,
    gyro,
    position,
    header,
    map_update,
    command,
    debug,
    pd_kp,
    pd_kd,
    parity_error
};

struct data_packet
{
    enum Address address;
    uint8_t byte_count;
    uint8_t bytes[7];
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
    stop,
    start,
    forward,
    backward,
    fw_left,
    fw_right,
    turn_left,
    turn_right
};

#endif
