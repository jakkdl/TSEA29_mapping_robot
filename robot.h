// This file contains definitions and stuff that all units will need
#ifndef ROBOT_H
#define ROBOT_H
#include <stdint.h>

// I don't know if we need to typedef this.
//typedef int8_t short;
//typedef int16_t int;

//TODO Enums
#define ADR_LIDAR_FORWARD   0
#define ADR_LIDAR_BACKWARD  1
#define ADR_IR_LEFTFRONT    2
#define ADR_IR_LEFTBACK     3
#define ADR_IR_RIGHTFRONT   4
#define ADR_IR_RIGHTBACK    5
#define ADR_ODOMETER        6
#define ADR_GYRO            7
#define ADR_POSITION        8
#define ADR_HEADER          9
#define ADR_MAP_UPDATE      0xA
#define ADR_COMMAND         0xB
#define ADR_DEBUG           0xC
#define ADR_PD_KP           0xD
#define ADR_PD_KD           0xE
#define ADR_PARITY_ERROR    0xF

const uint8_t ADR_DATA_PACKETS[] = {
    2,  //LIDAR_FORWARD
    2,  //LIDAR_BACKWARD
    2,  //IR_LEFTFRONT
    2,  //IR_LEFTBACK
    2,  //IR_RIGHTFRONT
    2,  //IR_RIGHTBACK
    1,  //ODOMETER
    2,  //GYRO
    4,  //POSITION
    2,  //HEADER
    3,  //MAP_UPDATE
    1,  //COMMAND
    7,  //DEBUG
    1,  //PD_KP
    1,  //PD_KD
    1   //PARITY_ERROR
};

#define ID_STOP         0
#define ID_START        1
#define ID_FORWARD      2
#define ID_BACKWARD     3
#define ID_FW_LEFT      4
#define ID_FW_RIGHT     5
#define ID_TURN_LEFT    6
#define ID_TURN_RIGHT   7

#endif
